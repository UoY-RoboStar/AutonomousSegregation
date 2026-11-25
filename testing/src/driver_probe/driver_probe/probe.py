# DriverProbe: the few lines a SUT node adds so the STC driver can observe it.
# The probe only reports. It never reads anything back, so it cannot change the SUT's logic.
# Everything goes out on one topic (/sut/events) from one publisher, so the driver gets a total order of events, and a gap in `seq` shows that a message was lost.
#
# How to use it inside the SUT node:
#
#   self.probe = DriverProbe(self)                             # once, in __init__
#   self.probe.run_cycle_on_execute_step(self.cycle)           # also in __init__: the driver's step runs each cycle
#
#   def on_some_input(self, msg):
#       self.probe.report_input_accepted("EnableClusterWatch") # input received (add its value if it has one); the driver may now trigger the cycle
#
#   def cycle(self): # one call is one cycle
#       now = self.probe.sim_time_ns                           # this cycle's simulated time, sent with the step (if needed)
#       ...
#       self.probe.report_register_write("RWMove", x, y)       # each output, right where it is published
#                                                              # 0 values means none, 1 is that value, 2 or more make a tuple
#       self.probe.report_cycle_done()                         # last thing in the cycle
import enum
import threading

from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from driver_interface.msg import ExecuteStep, SutEvent

EVENTS_TOPIC = "/sut/events"
EXECUTE_TOPIC = "/stc/execute"

# Reliable and transient local, so a driver that connects a moment late still receives the history.
EVENT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1000,
)


# A bare name (an enum literal such as Chemical_Angle_Left). A plain str means a quoted string instead.
class Ident(str):
    pass


# Turns a value into the text the driver reads (the format is described in the SutEvent.msg comment).
def encode_value_as_text(v):
    if isinstance(v, Ident):
        return str(v)
    if isinstance(v, enum.Enum):
        return v.name
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, int):
        return str(v)
    if isinstance(v, float):
        return repr(v)
    if isinstance(v, str):
        return '"' + v + '"'
    if isinstance(v, tuple):
        return "(| " + ", ".join(encode_value_as_text(x) for x in v) + " |)"
    if isinstance(v, list):
        return "<" + ", ".join(encode_value_as_text(x) for x in v) + ">"
    raise TypeError(f"DriverProbe cannot report a value of type {type(v).__name__}: {v!r}")


# Turns the values given to read() or write() into one piece of text: nothing, one value, or a tuple of several.
def _encode_register_values(values):
    return "" if not values else encode_value_as_text(values[0] if len(values) == 1 else tuple(values))


# The observer that a SUT node creates once and calls at the key points of each cycle.
class DriverProbe:
    # Sets up the publisher for the events.
    def __init__(self, node):
        self._node = node
        self._last_step = 0
        # The simulated time of the current cycle in nanoseconds, taken from the driver's execute step.
        self.sim_time_ns = 0
        self._pub = node.create_publisher(SutEvent, EVENTS_TOPIC, EVENT_QOS)
        self._lock = threading.Lock()
        self._seq = 0

    # Runs `cycle()` once for each step the driver publishes on /stc/execute. Before each call it sets
    # `sim_time_ns` to the time sent with that step, so the cycle can read its simulated time.
    # A repeated step number is ignored, so a duplicated message can never run a cycle twice.
    def run_cycle_on_execute_step(self, cycle):
        # Runs the cycle only if this step is newer than the last one.
        def run_cycle_if_new_step(msg):
            if msg.step <= self._last_step:
                return
            self._last_step = msg.step
            self.sim_time_ns = msg.clock_ns
            cycle()

        self._execute_sub = self._node.create_subscription(
            ExecuteStep, EXECUTE_TOPIC, run_cycle_if_new_step,
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10))

    # Reports that an input was received, with its value if it has one (no value, one value, or several as a tuple).
    # The driver waits for this before it triggers the cycle, and reads an input that was not reported as false.
    def report_input_accepted(self, register, *values):
        self._publish_event(SutEvent.INPUT_ACCEPTED, register, _encode_register_values(values))

    # Reports an output register written this cycle: no value, one value, or several (sent as a tuple).
    def report_register_write(self, register, *values):
        self._publish_event(SutEvent.REGISTER_WRITE, register, _encode_register_values(values))

    # Reports that the cycle has finished. (This is only the end marker; the model's tock is the time moving on.)
    def report_cycle_done(self):
        self._publish_event(SutEvent.CYCLE_DONE)

    # Builds one event and publishes it. The lock means the sequence number and the publishing happen together,
    # so the order of `seq` is the order of publishing.
    def _publish_event(self, kind, register="", value=""):
        with self._lock:
            msg = SutEvent()
            msg.seq = self._seq
            msg.kind = kind
            msg.register_name = register
            msg.value = value
            self._seq += 1
            self._pub.publish(msg)
