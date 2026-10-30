"""Exit 0 once every robot's scan, odom and camera topics (and optionally the overhead
camera and pucks) are live."""
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


def main(args=None):
    rclpy.init(args=args)
    node = Node('simulation_ready')
    namespaces = [ns for ns in node.declare_parameter('robot_namespaces', 'T3A').value.split(',') if ns]
    wait_items = node.declare_parameter('wait_for_items', True).value
    overhead_camera = node.declare_parameter('overhead_camera', False).value
    timeout = node.declare_parameter('timeout', 120.0).value
    if not timeout > 0:
        raise ValueError('timeout must be positive')
    sensors = [(f'/{ns}/{topic}', message) for ns in namespaces
               for topic, message in [('scan', LaserScan), ('odom', Odometry),
                                      ('camera/image_raw', Image)]]
    if overhead_camera:
        sensors.append(('/overhead_camera/image_raw', Image))
    required = {name for name, _ in sensors}
    if wait_items:
        required.add('/simulation/items_ready')
    seen = {}
    for name, message in sensors:
        # Reliable delivery: a best-effort reader can lose every fragment-split overhead frame.
        node.create_subscription(message, name, lambda msg, key=name: seen.update({key: time.monotonic()}),
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
    node.create_subscription(Bool, '/simulation/items_ready',
        lambda msg: seen.update({'/simulation/items_ready': float('inf')}) if msg.data else None,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    deadline = time.monotonic() + timeout
    status = 1
    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0.2)
            fresh = {name for name, stamp in seen.items() if time.monotonic()-stamp < 3.0}
            if required <= fresh:
                node.get_logger().info('Gazebo sensors and puck initialization are ready')
                status = 0
                break
            if not rclpy.ok() or time.monotonic() >= deadline:
                break
        if status:
            node.get_logger().error('Missing/stale simulation data: ' + ', '.join(sorted(required-fresh)))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return status
