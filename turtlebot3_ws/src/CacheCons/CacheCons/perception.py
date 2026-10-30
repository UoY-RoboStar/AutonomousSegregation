"""Onboard-camera puck observations and the passive gripper shared by CacheCons1-3."""
from collections import deque
from dataclasses import dataclass
import math
from pathlib import Path

import cv2
import numpy as np
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from interfaces.msg import ClusterData, PoseO, PuckData, VisibleClustersCC

from .interpolator import Calibration
from .perception_geometry import (COLOUR_NAMES, PUCK_RADIUS, Detection,
    cluster_observations, nearest_reachable_puck, pose_for_image)


def default_calibration_file():
    return str(Path(get_package_share_directory('CacheCons'))
               / 'data' / 'calibration' / 'OnboardCalibration_sim.csv')


def colour_masks(frame):
    """Red, green and blue HSV masks over non-overlapping hue intervals."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    red = (cv2.inRange(hsv, (0, 100, 70), (10, 255, 255))
           | cv2.inRange(hsv, (160, 100, 70), (180, 255, 255)))
    green = cv2.inRange(hsv, (25, 52, 50), (93, 255, 255))
    blue = cv2.inRange(hsv, (94, 80, 30), (130, 255, 255))
    return (red, green, blue)


@dataclass(frozen=True)
class Rectification:
    """Source pixels of a top-down grid, `scale` cells/m: row 0 is `max_range`
    ahead, column 0 is `max_range` to the left; uncalibrated cells have none."""
    calibration: Calibration
    map_x: np.ndarray
    map_y: np.ndarray
    max_range: float
    scale: float

    def local(self, column, row):
        return (self.max_range - (row + .5) / self.scale,
                self.max_range - (column + .5) / self.scale)


def rectification(calibration, max_range=1.5, scale=200.0):
    if not all(math.isfinite(v) and v > 0 for v in (max_range, scale)):
        raise ValueError('max_range and scale must be positive and finite')
    rows, columns = int(round(max_range * scale)), int(round(2 * max_range * scale))
    row_index, column_index = np.mgrid[0:rows, 0:columns]
    x = max_range - (row_index + .5) / scale
    y = max_range - (column_index + .5) / scale
    pixels = calibration.pixels_for(np.column_stack((x.ravel(), y.ravel())))
    map_x = pixels[:, 0].reshape(rows, columns)
    map_y = pixels[:, 1].reshape(rows, columns)
    outside = ~(np.isfinite(map_x) & np.isfinite(map_y))
    map_x[outside] = map_y[outside] = -1.0
    return Rectification(calibration, map_x.astype(np.float32),
                         map_y.astype(np.float32), max_range, scale)


def table_detections(masks, rectified):
    """Locate pucks as disks in the top-down view; touching pucks stay separate."""
    scale, max_range = rectified.scale, rectified.max_range
    detections = []
    for colour, mask in enumerate(masks, start=1):
        floor = cv2.remap(mask, rectified.map_x, rectified.map_y, cv2.INTER_NEAREST,
                          borderMode=cv2.BORDER_CONSTANT, borderValue=0)
        # Outside the grid is background, so an edge-touching silhouette has no border maximum.
        padded = cv2.copyMakeBorder(floor, 1, 1, 1, 1, cv2.BORDER_CONSTANT, value=0)
        distance = cv2.distanceTransform(padded, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)[1:-1, 1:-1]
        window = 2 * int(.7 * PUCK_RADIUS * scale) + 1
        maxima = cv2.dilate(distance, np.ones((window, window), np.uint8))
        peaks = ((distance >= maxima - 1e-5) & (distance >= .53 * PUCK_RADIUS * scale)
                 & (distance <= 1.4 * PUCK_RADIUS * scale)).astype(np.uint8)
        count, labels, _, centroids = cv2.connectedComponentsWithStats(peaks, 8)
        candidates = []
        for index in range(1, count):
            local = rectified.local(*centroids[index])
            pixel = rectified.calibration.to_pixel(local)
            if (pixel is None or not 1 <= pixel[0] < mask.shape[1] - 1
                    or not 1 <= pixel[1] < mask.shape[0] - 1
                    or math.hypot(*local) > max_range):
                continue
            peak = float(distance[labels == index].max())
            candidates.append((peak, local, pixel))
        # An elongated silhouette can give a second maximum within a diameter of its
        # centre; one 5 mm rectified cell of tolerance keeps touching pucks separate.
        accepted = []
        for _, local, pixel in sorted(candidates, reverse=True):
            if any(math.dist(local, prior) < 2 * PUCK_RADIUS - .005 for prior in accepted):
                continue
            accepted.append(local)
            detections.append(Detection(colour, local, pixel))
    return detections


def observation_message(detections, pose, distance):
    valid, points, clusters, degrees = cluster_observations(detections, pose, distance)
    message = VisibleClustersCC()
    message.n, message.m = len(valid), len(clusters)
    memberships = {index: cluster.cluster_id for cluster in clusters
                   for index in cluster.members}
    for index, point in enumerate(points):
        puck = PuckData()
        puck.puckid, puck.clusterid = [index + 1], [memberships[index]]
        puck.graphorder = [degrees[index]]
        puck.position = point
        puck.positionx, puck.positiony = point
        message.puckdata.append(puck)
    for observed in clusters:
        cluster = ClusterData()
        cluster.clusterid, cluster.clustertype = [observed.cluster_id], [observed.colour]
        cluster.clustersize, cluster.centroid = [len(observed.members)], observed.centroid
        message.clusterdata.append(cluster)
    return message


class Perception(Node):
    def __init__(self, name):
        super().__init__(name)
        # OpenCV otherwise starts a worker per core in every robot's perception node.
        cv2.setNumThreads(2)
        self.threshold = float(self.declare_parameter('cluster_distance', .25).value)
        if not math.isfinite(self.threshold) or self.threshold <= 0:
            raise ValueError('cluster_distance must be positive and finite')
        self.max_range = float(self.declare_parameter('perception_range', 1.5).value)
        self.pose_tolerance = float(self.declare_parameter('pose_tolerance', .3).value)
        table = str(self.declare_parameter('calibration_file', default_calibration_file()).value)
        self.calibration = Calibration.from_csv(table)
        self.rectified = rectification(self.calibration, self.max_range)
        self.poses = deque(maxlen=40)
        self.br = CvBridge()
        self.image_publisher = self.create_publisher(Image, 'camera/image_items', 10)
        self.Publish_VisibleClustersCC = self.create_publisher(VisibleClustersCC, 'VisibleClustersCC', 10)
        self.create_subscription(PoseO, 'PoseO', self.pose_callback, 10)
        self.create_subscription(Image, 'camera/image_raw', self.VisibleClustersCC_callback, qos_profile_sensor_data)
        self.get_logger().info(f'Colour observations: world frame, {self.threshold:.2f} m components, '
                               f'calibration table {table}')

    def pose_callback(self, msg):
        pose = tuple(msg.poseo)
        if len(pose) == 3 and all(map(math.isfinite, pose)):
            self.poses.append((self.get_clock().now().nanoseconds * 1e-9, pose))

    def VisibleClustersCC_callback(self, data):
        try:
            frame = self.br.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as error:
            self.get_logger().warning(f'Invalid camera frame: {error}')
            self.Publish_VisibleClustersCC.publish(VisibleClustersCC())
            return
        stamp = data.header.stamp.sec + data.header.stamp.nanosec * 1e-9
        pose = pose_for_image(tuple(self.poses), stamp, self.pose_tolerance)
        detections = []
        if pose is not None:
            detections = table_detections(colour_masks(frame), self.rectified)
        # Without a pose, publish an empty observation so no stale target survives.
        message = observation_message(detections, pose, self.threshold) if pose is not None else VisibleClustersCC()
        self.Publish_VisibleClustersCC.publish(message)
        if self.image_publisher.get_subscription_count() == 0:
            return
        for detection in detections:
            point = tuple(int(value) for value in detection.pixel)
            cv2.circle(frame, point, 8, (255, 255, 255), 2)
            cv2.putText(frame, COLOUR_NAMES[detection.colour], point,
                        cv2.FONT_HERSHEY_SIMPLEX, .45, (255, 255, 255), 1)
        image = self.br.cv2_to_imgmsg(frame, encoding='bgr8')
        image.header = data.header
        self.image_publisher.publish(image)


class PassiveGripper(Node):
    def __init__(self):
        super().__init__('passive_gripper_node')
        self.pub = self.create_publisher(Bool, 'pick', 10)
        self.pub2 = self.create_publisher(Bool, 'place', 10)
        self.pose = self.pose_at = None
        self.carried = False
        self.create_subscription(PoseO, 'PoseO', self.pose_callback, 10)
        self.create_subscription(VisibleClustersCC, 'VisibleClustersCC', self.VisibleClustersCC_callback, 10)
        self.create_subscription(Bool, 'DepositPuck', self.DepositPuck_callback, 10)
        self.create_subscription(Bool, 'PuckCarried', self.PuckCarried_callback, 10)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def pose_callback(self, msg):
        if len(msg.poseo) == 3 and all(map(math.isfinite, msg.poseo)):
            self.pose, self.pose_at = tuple(msg.poseo), self.now()

    def VisibleClustersCC_callback(self, msg):
        now = self.now()
        if self.carried or self.pose is None or not 0 <= now - self.pose_at <= .3:
            return
        puck = nearest_reachable_puck(msg.puckdata, msg.clusterdata, self.pose)
        if puck is not None:
            self.pub.publish(Bool(data=True))

    def DepositPuck_callback(self, msg):
        if msg.data:
            self.pub2.publish(Bool(data=True))

    def PuckCarried_callback(self, msg):
        self.carried = bool(msg.data)
