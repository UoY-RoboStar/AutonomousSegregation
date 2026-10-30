"""Record where one marker or ArUco tag appears for each position published on
calibration/position, by calibration_grid in simulation or by hand on a real robot."""
import math
import sys

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

from .perception import colour_masks
from .interpolator import write_table
from .overhead_pose import detect_tags


def single_marker_centroid(masks, min_area=4):
    """Centroid of the only whole blob of at least min_area pixels, else None."""
    union = np.zeros_like(masks[0])
    for mask in masks:
        union |= mask
    count, _, stats, centroids = cv2.connectedComponentsWithStats(union, 8)
    found = [index for index in range(1, count)
             if stats[index, cv2.CC_STAT_AREA] >= min_area]
    if len(found) != 1:
        return None
    left, top, width, height = (int(stats[found[0], field]) for field in
        (cv2.CC_STAT_LEFT, cv2.CC_STAT_TOP, cv2.CC_STAT_WIDTH, cv2.CC_STAT_HEIGHT))
    if (left <= 0 or top <= 0 or left + width >= union.shape[1]
            or top + height >= union.shape[0]):
        return None
    return (float(centroids[found[0]][0]), float(centroids[found[0]][1]))


class CameraCalibrator(Node):
    def __init__(self):
        super().__init__('camera_calibrator')
        self.output = str(self.declare_parameter('output', '').value)
        if not self.output:
            raise ValueError('camera_calibrator needs an output CSV path')
        # 'marker': one puck-coloured disc (onboard); 'aruco': tag `tag` (overhead).
        self.feature = str(self.declare_parameter('feature', 'marker').value)
        if self.feature not in ('marker', 'aruco'):
            raise ValueError('feature must be marker or aruco')
        self.tag = int(self.declare_parameter('tag', 1).value)
        self.min_area = int(self.declare_parameter('min_area', 4).value)
        self.settle_time = float(self.declare_parameter('settle_time', .25).value)
        if self.min_area < 1 or not math.isfinite(self.settle_time) or self.settle_time < 0:
            raise ValueError('min_area must be positive and settle_time nonnegative')
        self.br = CvBridge()
        self.rows = []
        self.pending = None
        self.captured = self.create_publisher(Bool, 'calibration/captured', 10)
        self.create_subscription(Point, 'calibration/position', self.position_callback, 10)
        # Reliable delivery: a best-effort reader can lose every fragment-split overhead frame.
        self.create_subscription(Image, 'camera/image_raw', self.image_callback,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.get_logger().info(f'Writing calibration samples to {self.output}')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def position_callback(self, msg):
        if not all(map(math.isfinite, (msg.x, msg.y))):
            self.get_logger().warning('Ignoring a non-finite calibration position')
            return
        if self.pending is not None:
            self.get_logger().warning('Replacing a position that was still being captured')
        # Use a frame rendered after the marker has settled.
        self.pending = (msg.x, msg.y, self.now() + self.settle_time)

    def image_callback(self, msg):
        if self.pending is None:
            return
        x, y, earliest = self.pending
        if msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 < earliest:
            return
        try:
            centroid = self.centre(msg)
        except CvBridgeError as error:
            self.get_logger().warning(f'Invalid camera frame: {error}')
            return
        self.pending = None
        if centroid is not None:
            self.rows.append((centroid[0], centroid[1], x, y))
            write_table(self.output, self.rows)
            self.get_logger().info(f'({x:.3f}, {y:.3f}) m at pixel '
                                   f'({centroid[0]:.1f}, {centroid[1]:.1f}); {len(self.rows)} samples')
        self.captured.publish(Bool(data=centroid is not None))

    def centre(self, msg):
        if self.feature == 'marker':
            return single_marker_centroid(colour_masks(self.br.imgmsg_to_cv2(msg, 'bgr8')),
                                          self.min_area)
        corners = detect_tags(self.br.imgmsg_to_cv2(msg, 'mono8')).get(self.tag)
        return None if corners is None else tuple(float(v) for v in corners.mean(axis=0))


def main(args=None):
    rclpy.init(args=args)
    node = CameraCalibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
