"""Bound ROS clock traffic in simulation time, preserving resets and stamps."""
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock


CLOCK_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)


class ClockThrottle:
    def __init__(self, rate):
        if not math.isfinite(rate) or not 0 < rate <= 1000:
            raise ValueError('clock rate must be in (0, 1000] Hz')
        self.period_ns = round(1e9 / rate)
        self.last_seen = self.last_published = None

    def accept(self, stamp):
        reset = self.last_seen is not None and stamp < self.last_seen
        self.last_seen = stamp
        if (self.last_published is None or reset
                or stamp - self.last_published >= self.period_ns):
            self.last_published = stamp
            return True
        return False


class ClockRelay(Node):
    def __init__(self):
        super().__init__('simulation_clock_relay')
        if self.get_parameter('use_sim_time').value:
            raise ValueError('clock relay must not subscribe to its own simulation clock')
        self.throttle = ClockThrottle(float(self.declare_parameter('rate', 100.).value))
        self.output = self.create_publisher(Clock, '/clock', CLOCK_QOS)
        self.create_subscription(Clock, '/simulation/clock_raw', self.receive, CLOCK_QOS)

    def receive(self, message):
        stamp = message.clock.sec * 1000000000 + message.clock.nanosec
        if self.throttle.accept(stamp):
            self.output.publish(message)


def main():
    rclpy.init()
    node = ClockRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
