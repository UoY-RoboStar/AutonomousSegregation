from .harness import FAIL, INCONCLUSIVE, PASS, Row, render_cycle_as_lines
from .observations import Cycle, format_observation, observations_equal, observation_lists_equal

from .rtspec import Trace, format_forbidden_event, inputs_that_are_true


# Runs one forbidden trace against a fresh SUT and returns (verdict, report rows).
# A forbidden trace is: some cycles the SUT should go through, then one event it must never do next.
#   PASS          the SUT followed the trace and did not do the forbidden event
#   FAIL          the SUT followed the trace and then did the forbidden event
#   INCONCLUSIVE  the SUT went a different way before reaching the forbidden point, so it could not be tested there
# `tol` is how far apart two numbers can be and still count as equal. The SUT is always stopped at the end.
def replay_forbidden_trace(transport, trace: Trace, tol=1e-6):
    rows = []
    transport.start_sut_and_connect()
    try:
        # Step 1: the complete cycles. Each must match the trace exactly, or the test is INCONCLUSIVE.
        for i, cyc in enumerate(trace.closed):
            ins = inputs_that_are_true(cyc)
            obs = transport.run_cycle(ins)
            ok = observation_lists_equal(cyc.reads, obs.reads, tol, read=True) and observation_lists_equal(cyc.writes, obs.writes, tol)
            rows.append(Row(f"closed cycle {i}", ins, render_cycle_as_lines(cyc), render_cycle_as_lines(obs), "ok" if ok else "MISMATCH"))
            if not ok:
                return INCONCLUSIVE, rows

        # Step 2: the final cycle. The trace gives only its first n writes, so the SUT must start with
        # exactly those. It may write more afterwards, and those extra writes are checked in step 3.
        cyc = trace.open
        ins = inputs_that_are_true(cyc)
        obs = transport.run_cycle(ins)
        n = len(cyc.writes)
        prefix = Cycle(obs.reads, obs.writes[:n])
        ok = (
            observation_lists_equal(cyc.reads, obs.reads, tol, read=True)
            and len(obs.writes) >= n
            and observation_lists_equal(cyc.writes, obs.writes[:n], tol)
        )
        rows.append(Row("final cycle prefix", ins, render_cycle_as_lines(cyc), render_cycle_as_lines(prefix), "ok" if ok else "MISMATCH"))
        if not ok:
            return INCONCLUSIVE, rows

        # Step 3: what the SUT did next. Either it wrote one more output, or it wrote nothing more and the cycle
        # ended (a "tock", time moving on). If that next event is the forbidden one, the test FAILs.
        f = trace.forbidden
        if len(obs.writes) > n:
            nxt = format_observation(obs.writes[n])
            hit = f.kind == "write" and observations_equal(obs.writes[n], (f.name, f.values), tol)
        else:
            nxt = "tock"
            hit = f.kind == "tock"
        rows.append(Row("forbidden event check", None, [f"never {format_forbidden_event(f)}"], [f"next was {nxt}"],
                        "OCCURRED" if hit else "refused"))
        return (FAIL if hit else PASS), rows
    finally:
        transport.stop_sut()
