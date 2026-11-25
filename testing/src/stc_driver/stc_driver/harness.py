# Test-harness plumbing for the STC driver: errors, waiting, report rows
# and the loop that runs every test. The verdict rules live in core.py.
import queue
import time
from dataclasses import dataclass
from typing import List, Optional

from .observations import format_observation

# The four verdicts a test can get.
PASS, FAIL, INCONCLUSIVE, ERROR = "PASS", "FAIL", "INCONCLUSIVE", "ERROR"


# A problem with the test set-up itself (a timeout, a lost message, a bad config), not with the SUT's behaviour.
# A test that raises it gets the verdict ERROR instead of PASS or FAIL.
class HarnessError(RuntimeError):
    pass


# Waits for the next item in queue `q` and returns it. If `deadline` (a time.monotonic() value) passes first, it
# raises a HarnessError whose message ends with describe(), e.g. "timed out waiting for the SUT to finish the cycle".
def wait_for_queue_item(q, deadline, describe):
    try:
        return q.get(timeout=max(0.0, deadline - time.monotonic()))
    except queue.Empty:
        raise HarnessError(f"timed out {describe()}") from None


# One line of a test's report: a cycle (or the final check), the inputs sent in it, what the trace expected,
# what the SUT actually did, and whether they matched ("ok", "MISMATCH", "OCCURRED" or "refused").
@dataclass
class Row:
    label: str
    inputs: Optional[list]
    expected: List[str]
    actual: List[str]
    status: str


# Turns a cycle into readable report lines: one "read ..." line per input, then one "write ..." line per output.
def render_cycle_as_lines(c):
    return [f"read {format_observation(r, read=True)}" for r in c.reads] + [f"write {format_observation(w)}" for w in c.writes]


# Runs every test in order of its id and returns {id: (verdict, details)}.
#   traces            the parsed traces, by id
#   errors            the traces that could not be parsed, by id (each gets ERROR without being run)
#   transport_factory makes a fresh connection to a new SUT for one trace
#   replay            the function that runs one trace and gives its verdict (core.replay_forbidden_trace)
#   only              if given, run just these ids
#   on_result         if given, called with (id, verdict, details) as soon as each test finishes
# Any exception during a test is caught and turned into ERROR, so one broken test does not stop the rest.
def run_all_tests(traces, errors, transport_factory, replay, only=None, on_result=None, tol=1e-6):
    results = {}
    for tid in sorted(set(traces) | set(errors)):
        if only is not None and tid not in only:
            continue
        if tid in errors:
            res = (ERROR, f"cannot parse trace: {errors[tid]}")
        else:
            try:
                res = replay(transport_factory(traces[tid]), traces[tid], tol)
            except HarnessError as e:
                res = (ERROR, str(e))
            except Exception as e:
                res = (ERROR, f"{type(e).__name__}: {e}")
        results[tid] = res
        if on_result:
            on_result(tid, *res)
    return results
