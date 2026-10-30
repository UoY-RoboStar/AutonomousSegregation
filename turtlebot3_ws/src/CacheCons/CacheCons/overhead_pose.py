"""Robot poses from the overhead camera: ArUco tag N on robot N, through the overhead table."""
import math
from pathlib import Path

import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from interfaces.msg import PoseO

from .interpolator import Calibration

DICTIONARY = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
# OpenCV 4.7 replaced the detection functions with ArucoDetector: Ubuntu 22.04 has
# OpenCV 4.5, the macOS build 4.14.
NEW_ARUCO_API = hasattr(cv2.aruco, 'ArucoDetector')
PARAMETERS = (cv2.aruco.DetectorParameters() if NEW_ARUCO_API
              else cv2.aruco.DetectorParameters_create())
# Tags span about 25 px in a 4096 px frame: accept candidates down to 1% of the frame
# width (OpenCV's default of 3% rejects them) and sample their cells finely.
PARAMETERS.minMarkerPerimeterRate = 0.01
PARAMETERS.perspectiveRemovePixelPerCell = 8
# ArucoDetector copies the parameters, so create it after setting them.
DETECTOR = cv2.aruco.ArucoDetector(DICTIONARY, PARAMETERS) if NEW_ARUCO_API else None
# The tag is centred above the lidar, behind the robot's origin, top edge facing its right.
TAG_OFFSET = (-0.032, 0.0)


def detect_tags(gray):
    """Corners (4x2, from the tag's top-left, clockwise) of each tag seen exactly once."""
    if DETECTOR is not None:
        corners, ids, _ = DETECTOR.detectMarkers(gray)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(gray, DICTIONARY, parameters=PARAMETERS)
    if ids is None:
        return {}
    found = {}
    for tag, points in zip(ids.ravel().tolist(), corners):
        found.setdefault(tag, []).append(points.reshape(4, 2))
    return {tag: points[0] for tag, points in found.items() if len(points) == 1}


def tag_pose(corners, calibration):
    """(x, y, yaw) of the robot carrying the tag with these image corners, or None."""
    centre = calibration.to_floor(tuple(corners.mean(axis=0)))
    top = calibration.to_floor(tuple((corners[0] + corners[1]) / 2))
    bottom = calibration.to_floor(tuple((corners[2] + corners[3]) / 2))
    if None in (centre, top, bottom):
        return None
    yaw = math.remainder(math.atan2(top[1] - bottom[1], top[0] - bottom[0]) + math.pi / 2,
                         2 * math.pi)
    c, s = math.cos(yaw), math.sin(yaw)
    return (centre[0] - c * TAG_OFFSET[0] + s * TAG_OFFSET[1],
            centre[1] - s * TAG_OFFSET[0] - c * TAG_OFFSET[1], yaw)


class OverheadPose(Node):
    def __init__(self):
        super().__init__('overhead_pose')
        table = str(self.declare_parameter('calibration_file', str(
            Path(get_package_share_directory('CacheCons'))
            / 'data' / 'calibration' / 'OverheadCalibration_sim.csv')).value)
        self.calibration = Calibration.from_csv(table)
        namespaces = [name for name in str(
            self.declare_parameter('robot_namespaces', 'T3A').value).split(',') if name]
        self.pose_publishers = {tag: self.create_publisher(PoseO, f'/{namespace}/PoseO', 10)
                                for tag, namespace in enumerate(namespaces, start=1)}
        self.br = CvBridge()
        # Reliable delivery: a best-effort reader can lose every fragment-split overhead frame.
        self.create_subscription(Image, '/overhead_camera/image_raw', self.image_callback,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))

    def image_callback(self, msg):
        try:
            gray = self.br.imgmsg_to_cv2(msg, 'mono8')
        except CvBridgeError as error:
            self.get_logger().warning(f'Invalid overhead frame: {error}')
            return
        for tag, corners in detect_tags(gray).items():
            publisher = self.pose_publishers.get(tag)
            pose = None if publisher is None else tag_pose(corners, self.calibration)
            if pose is not None:
                publisher.publish(PoseO(poseo=pose))


def main(args=None):
    rclpy.init(args=args)
    node = OverheadPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
