# STC driver for ROS 2 (Humble)

Runs forbidden traces (`.rtspec`) against any ROS 2 node that has a few observe-only lines added, on simulated time. 

### What each file does
| File | Purpose |
|---|---|
| `README.md` | How to build and run the driver, and how to watch it with `rqt_graph` |
| `configs/random_walk_execute.yaml` | The SUT config: launch command, cycle period, timeout, and the input name to topic mapping |
| `suites/*.rtspec` | The 180 STC forbidden traces |
| `driver_interface/msg/SutEvent.msg` | The one message the SUT sends on `/sut/events`: an input ack, a register write or a cycle-done marker |
| `driver_interface/msg/ExecuteStep.msg` | The message the driver sends on `/stc/execute` to run one cycle: the step number and that cycle's simulated time |
| `driver_interface/CMakeLists.txt`, `package.xml` | Build files for those messages |
| `driver_probe/driver_probe/probe.py` | The helper a SUT node uses to report to the driver (input acks, writes, cycle-done) and to listen for `/stc/execute`. It only observes, never changes the SUT logic |
| `stc_driver/stc_driver/values.py` | Parses and types values: bool, int, float, string, enum, tuple, sequence |
| `stc_driver/stc_driver/observations.py` | The `Cycle` structure, plus comparing and printing reads and writes |
| `stc_driver/stc_driver/harness.py` | Harness errors, waiting on queues, `run_all_tests` and report-row rendering. No ROS |
| `stc_driver/stc_driver/config.py` | Loads and checks the YAML config, and fills input messages from values (`fill_message_from_values`) |
| `stc_driver/stc_driver/ros_transport.py` | The only file that talks to ROS: starts the SUT, picks the domain ID, publishes the inputs and the execute steps, waits for acks, collects events, implements `--hold` |
| `stc_driver/stc_driver/reporting.py` | Prints each verdict, writes `verdicts/<suite>-verdict` and sets the exit code |
| `stc_driver/stc_driver/rtspec.py` | Reads `.rtspec` files and splits each trace into closed cycles, a final cycle and the forbidden event |
| `stc_driver/stc_driver/core.py` | `replay_forbidden_trace`: the STC verdict rules (PASS, FAIL, INCONCLUSIVE, ERROR). No ROS |
| `stc_driver/stc_driver/cli.py` | The `stc_driver` command and its arguments |
| `random_walk/random_walk/random_walk_logic.py` | The RandomWalk behaviour as plain Python, no ROS |
| `random_walk/random_walk/random_walk_node.py` | Wraps that logic as a ROS 2 node, with the probe lines added |
| `mutants/original.py` | An unchanged copy of `random_walk_logic.py`, for putting the original back after a mutant run |
| `mutants/m1_*.py` ... `m6_*.py` | One mutant each, in the same order as the table under "Mutants" |
| `<package>/package.xml`, `setup.py`, `setup.cfg`, `resource/<name>`, `__init__.py` | Boilerplate `colcon` needs to build and register each package |

## Build and run
Make sure to run this from the testing folder
```
source /opt/ros/humble/setup.bash
rm -rf build install log
colcon build
source install/setup.bash
ros2 run stc_driver stc_driver suites/Autonomous_Segregation_RandomWalk.rtspec

```

### Mutants
To check that the tests notice a fault, change one line of random_walk, rebuild (`colcon build`), and run the suite as usual. Change only one line at a time, and restore it afterwards.Each row uses a different mutation operator.

| File in `mutants/` | Mutant | Mutation operator | Line | Change | Expected |
|---|---|---|---|---|---|
| `m1_initial_sign.py` | initial `sign` | inline constants | 23 | `self.sign = 0` to `self.sign = 1` | 8 FAIL, 172 PASS |
| `m2_turn_writes_nothing.py` | Turn writes nothing | void method call removal | 35 | delete `out.append((self.av * self.sign, 0.0))` (replace it with `pass`) | 8 FAIL, 172 PASS |
| `m3_wait_leaves_on_enable_false.py` | Wait leaves on Enable false | negate conditionals | 49 | `if enable:` to `if not enable:` | 16 FAIL, 164 PASS |
| `m4_turning_command.py` | turning command | arithmetic operator replacement (AOR_3) | 35 | `self.av * self.sign` to `self.av + self.sign` | 8 FAIL, 172 PASS |
| `m5_wait_leaves_always.py` | Wait always left | remove conditionals | 49 | `if enable:` to `if True:` | 8 FAIL, 172 PASS |
| `m6_wait_state_test.py` | Wait test inverted | relational operator replacement (ROR5) | 48 | `if self.state == "Wait":` to `if self.state != "Wait":` | 12 FAIL, 168 PASS |


### Watching the nodes and topics with `rqt_graph`
Each test runs in its own ROS domain: `ROS_DOMAIN_ID = 20 + (test number % 50)`. STC trace `t3` is domain 23.
`rqt_graph` only sees nodes in its own domain, so give it the same ID as the test you watch.
The driver and the SUT set the ID themselves; only tools you start by hand (`rqt_graph`, `ros2 topic list`, `ros2 topic echo`) need it.

Terminal 1 (start it first):
```
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=23
rqt_graph
```
Terminal 2 (one test only, so the domain is known; `--hold 30` keeps everything alive for 30 s after the test finishes):
```
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run stc_driver stc_driver suites/Autonomous_Segregation_RandomWalk.rtspec --only 3 --hold 30
```

## Testing your own SUT: three steps

### 1. Add the probe to the SUT node (observe-only, never feeds back into the logic)
Add `<depend>driver_probe</depend>` to its `package.xml`, then:
```python
from driver_probe.probe import DriverProbe

self.probe = DriverProbe(self)                                     # in __init__
self.probe.run_cycle_on_execute_step(self.cycle)                   # in __init__: each step on /stc/execute runs one cycle

def on_input(self, msg):                                           # each input callback
    if msg.data:
        self.probe.report_input_accepted("EnableClusterWatch")     # the driver waits for this before triggering the cycle
                                                                   # (an input with a value: probe.report_input_accepted("Speed", 5))
                                                                   # an input with no such report is read as false

def cycle(self):                                                   # one call = one cycle (no timer)
    now = self.probe.sim_time_ns                                   # this cycle's simulated time, if the SUT needs it
    ...
    self.probe.report_register_write("RWMove", x, y)               # right where each output is published
    self.probe.report_cycle_done()                                 # last line of the cycle
```
`random_walk_node.py` is a complete example: every line mentioning `probe` is the instrumentation.

### 2. Make the SUT follow the cycle contract
- One cycle = one call from `probe.run_cycle_on_execute_step`. The SUT must not run cycles from its own timer.
- Simulated time comes with each step: read it from `probe.sim_time_ns`, not from the node's ROS clock or a ROS timer.
  The driver does not publish `/clock`, so the SUT needs no `use_sim_time`.
- An input is a message on its topic. A true input is published only in the cycles where the trace says true, so "no message this cycle" must mean false (clear the latch each cycle).
- Report each write in the order the SUT produces it. Reads are not reported: the driver works them out from the input acks
  (an acknowledged input is true with its value, any other input is false).
- One `DriverProbe` per SUT. Two probes would give two sequence counters and the driver would report a lost event.

### 3. Write the config (`configs/<sut>.yaml`)
```yaml
sut_cmd: ros2 run random_walk random_walk
cycle_period: 0.1          # simulated seconds that pass between two cycles (added to the time sent with each step)
timeout: 5.0               # wall-clock seconds per cycle (harness faults only, never a verdict)
inputs:                    # one entry per input register in the suite
  EnableClusterWatch:
    topic: /EnableClusterWatch
    type: std_msgs/msg/Bool
    fields: [data]         # optional, default [data]
```
The driver takes each output's value from the probe's `report_register_write` calls, so outputs need no entry in the config.


## What the driver does per cycle
1. publish the inputs that are true (message type and topic from the config)
2. wait for one `accepted` per input
3. publish an `ExecuteStep` on `/stc/execute` with the next step number and this cycle's simulated time, so the SUT runs one cycle at that time
   (a repeated step number is ignored by the probe; the time cannot arrive late because it is in the same message)
4. collect the writes until `report_cycle_done` (the cycle's reads come from the acks in step 2)
5. move simulated time on by `cycle_period` for the next step (the model's tock)

Each trace gets a fresh SUT process and its own `ROS_DOMAIN_ID`. Verdicts: 
- PASS (forbidden event not seen)
- FAIL (seen)
- INCONCLUSIVE (the SUT diverged from the trace before the forbidden point)
- ERROR (harness fault: timeout, lost event, SUT died, bad config)




