# RandomWalk as a ROS 2 node (Humble), instrumented for the STC driver.
#
# Real behaviour:
#   subscribes to /DisableClusterWatch and /EnableClusterWatch (std_msgs/Bool; a message means "true")
#   publishes /RWMove (geometry_msgs/Point)
#   runs one cycle each time the driver publishes a new step on /stc/execute
#
# The instrumentation is every line that mentions `probe`. It only observes and never feeds back into the logic.
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import Bool

from driver_probe.probe import DriverProbe

from .random_walk_logic import RandomWalkLogic

# The order the inputs are sampled in, which is the registerRead order in the suite.
INPUTS = ("DisableClusterWatch", "EnableClusterWatch")


# The random_walk SUT: wraps RandomWalkLogic in a ROS 2 node.
class RandomWalk(Node):
    # Sets up the probe, the input subscriptions, the output publisher and the execute step that runs each cycle.
    def __init__(self):
        super().__init__("random_walk")

        # instrumentation
        self.probe = DriverProbe(self)
        self._logic = RandomWalkLogic()
        self._latched = {name: False for name in INPUTS}

        self._rwmove = self.create_publisher(Point, "/RWMove", 10)
        for name in INPUTS:
            self.create_subscription(Bool, f"/{name}", lambda msg, n=name: self._store_input_and_report_accepted(n, msg), 10)
        # instrumentation: each new step on /stc/execute runs one cycle
        self.probe.run_cycle_on_execute_step(self._run_one_cycle)
        self.get_logger().info("random_walk SUT up")

    # Remembers that an input arrived, and tells the driver it was received.
    def _store_input_and_report_accepted(self, name, msg):
        if msg.data:
            self._latched[name] = True
            # instrumentation
            self.probe.report_input_accepted(name)

    # Runs one cycle: samples the inputs, steps the logic, publishes any output and reports to the probe.
    def _run_one_cycle(self):
        # Take a snapshot of the inputs and clear them for the next cycle.
        sampled = dict(self._latched)
        for name in INPUTS:
            self._latched[name] = False

        # Publish each output the logic produces.
        for x, y in self._logic.run_one_cycle(sampled["DisableClusterWatch"], sampled["EnableClusterWatch"]):
            self._rwmove.publish(Point(x=float(x), y=float(y), z=0.0))
            # instrumentation
            self.probe.report_register_write("RWMove", x, y)

        # instrumentation
        self.probe.report_cycle_done()


# Starts the node and keeps it running until it is stopped.
def run_random_walk(args=None):
    rclpy.init(args=args)
    node = RandomWalk()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    run_random_walk()
