"""Move a marker (onboard) or the robot (overhead) through a Gazebo grid for camera_calibrator."""
import argparse
import math
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Pose
from rclpy.node import Node
from std_msgs.msg import Bool

from .gazebo_backend import GazeboBackend
from .overhead_pose import TAG_OFFSET

MARKER = 'calibration_marker'
MARKER_SDF = """<sdf version="1.6">
  <model name="calibration_marker">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <pose>0 0 0.002 0 0 0</pose>
        <geometry><cylinder><radius>0.03</radius><length>0.004</length></cylinder></geometry>
        <material><ambient>1 0 0 1</ambient><diffuse>1 0 0 1</diffuse>
          <specular>0 0 0 1</specular><emissive>0 0 0 1</emissive></material>
      </visual>
    </link>
  </model>
</sdf>"""


def calibration_grid(x_range, y_range, step):
    """Evenly spaced points covering both ranges, at most `step` apart."""
    if not (math.isfinite(step) and step > 0
            and all(math.isfinite(v) for v in (*x_range, *y_range))
            and x_range[0] <= x_range[1] and y_range[0] <= y_range[1]):
        raise ValueError('The grid needs ordered finite ranges and a positive step')
    xs, ys = (np.linspace(low, high, int(math.ceil(round((high - low) / step, 6))) + 1)
              for low, high in (x_range, y_range))
    return [(round(float(x), 4), round(float(y), 4)) for x in xs for y in ys]


# Onboard: ahead of the robot, in its base frame. Overhead: the arena interior less
# the robot's clearance from the walls, in the world frame.
GRIDS = {'onboard': ((.1, 1.5), (-.9, .9), .05),
         'overhead': ((-3.85, 2.85), (-2.85, 2.85), .25)}


def world_position(robot_pose, local):
    """Base-frame point in the world, using the robot's yaw."""
    q = robot_pose.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    c, s = math.cos(yaw), math.sin(yaw)
    return (robot_pose.position.x + c * local[0] - s * local[1],
            robot_pose.position.y + s * local[0] + c * local[1])


class CalibrationGrid(Node):
    def __init__(self, robot, camera, timeout):
        super().__init__('calibration_grid')
        self.robot, self.camera, self.timeout = robot, camera, timeout
        self.positions = calibration_grid(*GRIDS[camera])
        self.gazebo = GazeboBackend([robot])
        if camera == 'onboard':
            below_floor = Pose()
            below_floor.position.z, below_floor.orientation.w = -.5, 1.
            self.gazebo.spawn(MARKER, MARKER_SDF, below_floor)
        self.position_publisher = self.create_publisher(Point, 'calibration/position', 10)
        self.create_subscription(Bool, 'calibration/captured', self.captured_callback, 10)
        self.index, self.recorded, self.deadline, self.done = 0, 0, None, None
        self.started = time.monotonic()
        self.timer = self.create_timer(.05, self.step)

    def captured_callback(self, msg):
        if self.deadline is None:
            return
        self.recorded += int(msg.data)
        self.deadline, self.index = None, self.index + 1

    def step(self):
        if self.done is not None:
            return
        if self.deadline is not None:
            if time.monotonic() > self.deadline:
                self.get_logger().error(f'No capture result for {self.positions[self.index]}')
                self.done = 1
            return
        if self.index >= len(self.positions):
            self.get_logger().info(f'Recorded {self.recorded} of {len(self.positions)} positions')
            self.done = 0 if self.recorded >= 3 else 1
            return
        if self.position_publisher.get_subscription_count() == 0:
            if time.monotonic() - self.started > self.timeout:
                self.get_logger().error('No camera_calibrator is listening for positions')
                self.done = 1
            return
        position = self.positions[self.index]
        pose = Pose()
        pose.orientation.w = 1.
        if self.camera == 'onboard':
            robot = self.gazebo.get_pose(self.robot)
            pose.position.x, pose.position.y = world_position(robot, position)
            self.gazebo.set_pose(MARKER, 'world', pose)
        else:
            # The overhead table maps the tag centre, so record where the tag is.
            pose.position.x, pose.position.y, pose.position.z = *position, .01
            self.gazebo.set_pose(self.robot, 'world', pose)
            position = (position[0] + TAG_OFFSET[0], position[1] + TAG_OFFSET[1])
        self.position_publisher.publish(Point(x=position[0], y=position[1], z=0.))
        self.deadline = time.monotonic() + self.timeout


def main(args=sys.argv):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot', required=True, help='Gazebo model name of the robot')
    parser.add_argument('--camera', choices=sorted(GRIDS), required=True)
    parser.add_argument('--timeout', type=float, default=10., help='seconds to wait per capture')
    parsed = parser.parse_args(rclpy.utilities.remove_ros_args(args)[1:])
    node = CalibrationGrid(parsed.robot, parsed.camera, parsed.timeout)
    status = 1
    try:
        while rclpy.ok() and node.done is None:
            rclpy.spin_once(node, timeout_sec=.1)
        status = 1 if node.done is None else node.done
    except KeyboardInterrupt:
        pass
    finally:
        node.gazebo.close()
        node.destroy_node()
        rclpy.try_shutdown()
    return status


if __name__ == '__main__':
    sys.exit(main())
