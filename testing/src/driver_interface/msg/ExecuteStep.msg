# One execute step published by the STC driver on /stc/execute: run exactly one cycle, at this simulated time.
# The time travels in the same message as the step, so the SUT can never run a cycle with the wrong time.

uint64 step              # 1, 2, 3, ... one per cycle; a step that is not newer than the last one is ignored
int64 clock_ns           # the simulated time of this cycle in nanoseconds (cycle 1 is 0, then + cycle_period each cycle)
