# The ROS 2 transport: starts one SUT process per trace and drives it cycle by cycle.
# Each cycle:
#   1. publish the inputs that are true this cycle (topic and message type come from the config file)
#   2. wait until the SUT reports one INPUT_ACCEPTED (with the input's value) for each input; the reads of the cycle are
#      built from these reports, and an input with no report is read as false
#   3. publish an ExecuteStep on /stc/execute: the next step number and this cycle's simulated time, so the SUT
#      runs one cycle at that time (the time travels with the step, so it cannot arrive late or out of order)
#   4. read REGISTER_WRITE events from /sut/events until CYCLE_DONE
# It needs rclpy and driver_interface, so it is only imported for a real run.
import os
import queue
import signal
import subprocess
import threading
import time

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message

from driver_interface.msg import ExecuteStep, SutEvent

from .config import ConfigError, fill_message_from_values
from .harness import HarnessError, wait_for_queue_item
from .observations import Cycle
from .values import parse_value

EVENTS_TOPIC = "/sut/events"
EXECUTE_TOPIC = "/stc/execute"
EXECUTE_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)

# Must match the SUT's publisher: reliable and transient local.
EVENT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1000,
)


# Runs one SUT process and drives it a cycle at a time over ROS topics.
class RosTransport:
    # Stores the settings. `sut_cmd` is the command that starts the SUT, `period_ns` is how far simulated time
    # moves on after each cycle, and `inputs` maps a register name to its config.InputSpec.
    def __init__(self, sut_cmd, domain_id, period_ns, inputs, timeout=5.0, startup_timeout=15.0,
                 sut_stderr=False, hold=0.0):
        self.sut_cmd, self.domain_id, self.period_ns = sut_cmd, domain_id, period_ns
        self.inputs = inputs
        self.timeout, self.startup_timeout, self.sut_stderr = timeout, startup_timeout, sut_stderr
        # Seconds to keep the SUT and the driver alive after a finished test (for tools such as rqt_graph).
        self.hold = hold
        self._ready = False
        self._exec_pub = None
        self._step = 0
        self._ctx = self._node = self._exec = self._thread = self._proc = None
        self._in_pubs, self._in_cls = {}, {}
        self._q = queue.Queue()
        self._next_seq = 0
        self._now_ns = 0

    # Starts the ROS node, the publishers and the SUT, then waits until the SUT is ready.
    def start_sut_and_connect(self):
        try:
            # The domain id is read when ROS starts up, and the SUT gets it too.
            os.environ["ROS_DOMAIN_ID"] = str(self.domain_id)
            self._ctx = Context()
            self._ctx.init()
            self._node = rclpy.create_node("stc_driver", context=self._ctx)

            # Listen for the SUT's events, and create the publisher for the execute steps.
            self._node.create_subscription(SutEvent, EVENTS_TOPIC, self._q.put, EVENT_QOS)
            self._exec_pub = self._node.create_publisher(ExecuteStep, EXECUTE_TOPIC, EXECUTE_QOS)

            # One publisher per input register, using the message type from the config.
            for name, spec in self.inputs.items():
                cls = self._find_message_class("input", name, spec)
                self._in_cls[name] = cls
                self._in_pubs[name] = self._node.create_publisher(cls, spec.topic, 10)

            # Spin the node in a background thread so messages are handled while we wait.
            self._exec = SingleThreadedExecutor(context=self._ctx)
            self._exec.add_node(self._node)
            self._thread = threading.Thread(target=self._exec.spin, daemon=True)
            self._thread.start()

            # Start the SUT in its own session so we can stop `ros2 run` and the node together.
            self._proc = subprocess.Popen(
                self.sut_cmd, env=dict(os.environ, ROS_DOMAIN_ID=str(self.domain_id)),
                stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                stderr=None if self.sut_stderr else subprocess.DEVNULL,
                start_new_session=True,
            )
            self._wait_until_connected()
            self._ready = True
        except BaseException:
            self.stop_sut()
            raise

    # Loads the message class of an input, or raises a harness fault that says what to check.
    def _find_message_class(self, what, name, spec):
        try:
            return get_message(spec.type)
        except (ValueError, ModuleNotFoundError, AttributeError) as e:
            raise HarnessError(f"{what} {name}: cannot load message type {spec.type} ({e}); "
                               f"is its package built and sourced?") from None

    # Lists what the SUT has not connected yet (an empty list means it is ready).
    def _find_missing_connections(self):
        missing = []
        if self._node.count_publishers(EVENTS_TOPIC) < 1:
            missing.append(f"a publisher on {EVENTS_TOPIC} (is the SUT instrumented with DriverProbe?)")
        if self._exec_pub.get_subscription_count() < 1:
            missing.append(f"a subscriber on {EXECUTE_TOPIC} (does the SUT call probe.run_cycle_on_execute_step?)")
        for name, pub in self._in_pubs.items():
            if pub.get_subscription_count() < 1:
                missing.append(f"a subscriber on {self.inputs[name].topic} of type {self.inputs[name].type}")
        return missing

    # Waits until the SUT's topics are visible, because anything published earlier would be lost.
    def _wait_until_connected(self):
        deadline = time.monotonic() + self.startup_timeout
        while True:
            if self._proc.poll() is not None:
                raise HarnessError(f"SUT exited during start-up (code {self._proc.returncode}); "
                                   f"rerun with --sut-stderr to see why")
            missing = self._find_missing_connections()
            if not missing:
                return
            if time.monotonic() > deadline:
                raise HarnessError("timed out waiting for the SUT; still missing: " + "; ".join(missing))
            time.sleep(0.02)

    # Stops the SUT process and shuts the ROS node down. It is safe to call more than once.
    # With a hold time it first waits, so tools such as rqt_graph can look at the running nodes.
    def stop_sut(self):
        # Wait only after a test that started properly, and only once. Ctrl+C ends the wait early.
        if self.hold > 0 and self._ready:
            self._ready = False
            print(f"holding the SUT for {self.hold:g}s (ROS_DOMAIN_ID={self.domain_id}); Ctrl+C ends the wait", flush=True)
            try:
                time.sleep(self.hold)
            except KeyboardInterrupt:
                pass
        # Ask the SUT to stop politely, then force it if it does not.
        if self._proc is not None:
            for sig in (signal.SIGTERM, signal.SIGKILL):
                try:
                    os.killpg(self._proc.pid, sig)
                except (ProcessLookupError, PermissionError):
                    break
                try:
                    self._proc.wait(timeout=3)
                    break
                except subprocess.TimeoutExpired:
                    pass
            self._proc = None
        # Shut down the ROS side.
        if self._exec is not None:
            self._exec.shutdown()
        if self._thread is not None:
            self._thread.join(timeout=2)
        if self._node is not None:
            self._node.destroy_node()
        if self._ctx is not None and self._ctx.ok():
            self._ctx.shutdown()
        self._ctx = self._node = self._exec = self._thread = None

    # Waits for the next event from the SUT and checks that none were lost or reordered.
    def _take_next_event(self, deadline, describe):
        ev = wait_for_queue_item(self._q, deadline, describe)
        if ev.seq != self._next_seq:
            raise HarnessError(f"lost or reordered event: expected seq {self._next_seq}, got {ev.seq}")
        self._next_seq += 1
        return ev

    # Runs exactly one cycle with the given true inputs, and returns what the SUT read and wrote.
    def run_cycle(self, inputs):
        deadline = time.monotonic() + self.timeout

        # Publish each true input as a message on its topic.
        names = [name for name, _ in inputs]
        for name, values in inputs:
            if name not in self._in_pubs:
                raise HarnessError(f"input register {name!r} is used by the suite but has no entry under "
                                   f"'inputs' in the config file")
            msg = self._in_cls[name]()
            try:
                fill_message_from_values(msg, self.inputs[name].fields, values, self.inputs[name].enums)
            except ConfigError as e:
                raise HarnessError(f"input {name}: {e}") from None
            self._in_pubs[name].publish(msg)

        # Wait for the SUT to confirm it has received every input.
        accepted = []
        while len(accepted) < len(names):
            ev = self._take_next_event(
                deadline, lambda: f"waiting for the SUT to acknowledge inputs {names} (acknowledged: {[n for n, _ in accepted]})")
            if ev.kind != SutEvent.INPUT_ACCEPTED:
                raise HarnessError("SUT produced a non-ack event before the cycle was triggered")
            try:
                accepted.append((ev.register_name, (parse_value(ev.value),) if ev.value else ()))
            except ValueError as e:
                raise HarnessError(f"cannot read the value of {ev.register_name!r} reported by the SUT: {e}") from None
        acked_names = sorted(n for n, _ in accepted)
        if acked_names != sorted(names):
            raise HarnessError(f"acknowledged inputs {acked_names} differ from published {sorted(names)}")

        # The reads of this cycle: an acknowledged input is present with its value, any other input is absent.
        cyc = Cycle()
        received = dict(accepted)
        for name in self.inputs:
            cyc.reads.append((name, (True,) + received[name] if name in received else (False,)))

        # Trigger the cycle by sending the next execute step with this cycle's simulated time.
        # The execute topic is reliable, so it is sent once.
        self._step += 1
        self._exec_pub.publish(ExecuteStep(step=self._step, clock_ns=self._now_ns))

        # Collect the SUT's writes until it says the cycle is done.
        while True:
            ev = self._take_next_event(
                deadline,
                lambda: (f"waiting for the cycle to finish after execute step {self._step} (time {self._now_ns} ns) "
                         f"(seen so far: writes={cyc.writes}); does the SUT run a cycle on /stc/execute "
                         f"and call probe.report_cycle_done()?"))

            # The end of the cycle: move simulated time on by one period (the model's tock) for the next step.
            if ev.kind == SutEvent.CYCLE_DONE:
                self._now_ns += self.period_ns
                return cyc

            # A write: read its value, then store it in the cycle.
            try:
                value = (parse_value(ev.value),) if ev.value else ()
            except ValueError as e:
                raise HarnessError(f"cannot read the value of {ev.register_name!r} reported by the SUT: {e}") from None
            if ev.kind == SutEvent.REGISTER_WRITE:
                cyc.writes.append((ev.register_name, value))
            else:
                raise HarnessError(f"unexpected event kind {ev.kind} during a cycle")
