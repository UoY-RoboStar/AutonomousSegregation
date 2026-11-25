# Command line for the STC driver: stc_driver <suite.rtspec> [--config <sut.yaml>] [options]
# It runs each forbidden trace against the SUT described by the config file, on simulated time, prints one verdict per trace, writes a summary next to the suite (verdicts/<suite>-verdict) and exits with a non-zero code on FAIL or ERROR.
import argparse
import os
import sys

from .config import ConfigError, load_config
from .harness import run_all_tests
from .reporting import DOMAIN_BASE, DOMAIN_SPAN, STARTUP_TIMEOUT, print_report_and_get_exit_code, print_test_result

from .core import replay_forbidden_trace
from .rtspec import load_traces, inputs_that_are_true

# Runs the STC driver: reads the arguments, loads the suite and config, runs every trace and reports.
def run_stc_driver(argv=None):
    ap = argparse.ArgumentParser(prog="stc_driver")
    ap.add_argument("rtspec", help="path to the .rtspec forbidden-trace file")
    ap.add_argument("--config", default="configs/random_walk_execute.yaml",
                    help="YAML file describing the SUT (default: configs/random_walk_execute.yaml)")
    ap.add_argument("--only", default=None, help="comma-separated trace ids to run, e.g. 3,17")
    ap.add_argument("--sut-stderr", action="store_true", help="show the SUT's stderr (debugging)")
    ap.add_argument("--hold", type=float, default=0.0, metavar="SECONDS",
                    help="keep the SUT and driver alive this long after each test so rqt_graph can see them (use with --only)")
    args = ap.parse_args(argv)

    # Imported here because it needs rclpy and driver_interface, which the rest of the driver does not.
    from .ros_transport import RosTransport

    try:
        cfg = load_config(args.config)
    except ConfigError as e:
        print(f"config error: {e}", file=sys.stderr)
        return 2

    start = os.times()
    traces, errors = load_traces(args.rtspec)
    only = {int(x) for x in args.only.split(",")} if args.only else None

    # Warn about inputs the suite uses that the config does not map to a topic.
    unmapped = sorted({n for t in traces.values() for c in (*t.closed, t.open) for n, _ in inputs_that_are_true(c)}
                      - set(cfg.inputs))
    if unmapped:
        print(f"note: the suite sends input(s) {unmapped} that have no mapping under 'inputs' in {args.config}; "
              f"traces using them will be ERROR", file=sys.stderr)

    # Makes a fresh transport (and so a fresh SUT) for each trace.
    def make_fresh_transport(trace):
        return RosTransport(cfg.sut_cmd, DOMAIN_BASE + trace.tid % DOMAIN_SPAN, int(round(cfg.cycle_period * 1e9)),
                            cfg.inputs, timeout=cfg.timeout, startup_timeout=STARTUP_TIMEOUT,
                            sut_stderr=args.sut_stderr,
                            hold=max(0.0, args.hold))

    results = run_all_tests(traces, errors, make_fresh_transport, replay_forbidden_trace, only=only,
                        on_result=lambda tid, v, d: print_test_result(f"t{tid}", v, d))
    return print_report_and_get_exit_code(args.rtspec, results, start, "t")


# The function the stc_driver command runs: calls main and exits with its code.
def run_stc_driver_and_exit():
    sys.exit(run_stc_driver())


if __name__ == "__main__":
    run_stc_driver_and_exit()
