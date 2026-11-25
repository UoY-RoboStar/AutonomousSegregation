# Printing and summarising STC results.
import os
from pathlib import Path

from .harness import ERROR, FAIL, INCONCLUSIVE, PASS

# Each trace gets its own ROS_DOMAIN_ID, counted up from the base and wrapped within the span.
DOMAIN_BASE, DOMAIN_SPAN = 20, 50
STARTUP_TIMEOUT = 15.0


# Prints one result: the verdict, then the details unless the test passed.
def print_test_result(label, verdict, detail):
    print(f"{label}: {verdict}", flush=True)
    if verdict == ERROR:
        print(f"    {detail}")
    elif verdict not in (PASS,):
        for r in detail:
            inp = f" inputs={[i[0] if isinstance(i, tuple) else i for i in r.inputs]}" if r.inputs is not None else ""
            print(f"    {r.label}{inp} [{r.status}]")
            print(f"        expected: {r.expected}")
            print(f"        actual:   {r.actual}")


# Prints the totals, writes the summary file and returns the exit code (1 if anything failed or errored).
def print_report_and_get_exit_code(source, results, start, prefix):
    labels = (PASS, FAIL, INCONCLUSIVE, ERROR)
    counts = {k: 0 for k in labels}
    for verdict, _ in results.values():
        counts[verdict] += 1
    tally = ", ".join(f"{counts[k]} {k.lower()}" for k in labels) + f", {len(results)} total"
    print("\n" + tally)

    # Work out how much CPU time the driver and the SUT used.
    end = os.times()
    driver_cpu = (end.user + end.system) - (start.user + start.system)
    sut_cpu = (end.children_user + end.children_system) - (start.children_user + start.children_system)
    cpu_line = f"CPU execution time: {driver_cpu + sut_cpu:.3f}s (driver {driver_cpu:.3f}s, SUT {sut_cpu:.3f}s)"

    # Write the summary file next to the suite.
    suite = Path(source).resolve()
    out_dir = suite.parent / "verdicts"
    out_dir.mkdir(exist_ok=True)
    out = out_dir / f"{suite.stem}-verdict"
    lines = [tally, ""]
    for label in labels:
        ids = sorted(t for t, (v, _) in results.items() if v == label)
        lines.append(f"{label} ({len(ids)}): {', '.join(f'{prefix}{t}' for t in ids)}")
    out.write_text("\n".join(lines + ["", cpu_line]) + "\n")
    print(f"\nwrote summary to {out}\n{cpu_line}")
    return 1 if counts[FAIL] or counts[ERROR] else 0
