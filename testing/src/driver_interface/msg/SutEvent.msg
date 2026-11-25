# One observation record published by the (instrumented) SUT on /sut/events.

uint8 INPUT_ACCEPTED=0   # a true input was received, with its value & inputs with no such report are read as false
uint8 REGISTER_WRITE=1   # output register written this cycle
uint8 CYCLE_DONE=2       # the SUT finished reporting this cycle (not the model's tock)

uint64 seq               # 0,1,2,... assigned under a lock together with the publish
uint8 kind
string register_name     # e.g. "DisableClusterWatch", "RWMove" (no i_/o_ prefix)
string value             # INPUT_ACCEPTED / REGISTER_WRITE: the value as text, "" = no value. Text grammar:
                         #   true  5  -2.5  "text"  Left  Mode::Left  (| 1, 2 |)  <(0, 0), (1, 0)>
