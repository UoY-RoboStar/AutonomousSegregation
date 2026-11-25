# Reads STC .rtspec forbidden traces and splits them into closed cycles, a final cycle and a forbidden event.
# Each trace is split into whole cycles plus the forbidden event, and values are typed (see values.py)
# A bad trace is reported as an error for that trace, not for the whole run.
import re
from dataclasses import dataclass
from pathlib import Path

from .observations import Cycle, format_observation, remove_name_prefix
from .values import parse_value, split_outside_brackets

TRACE_RE = re.compile(r"^t(\d+):\s*(.+)$")

# One raw event from a trace line, before it is sorted into a cycle.
@dataclass(frozen=True)
class Ev:
    # One of: registerRead, registerWrite, tock.
    action: str
    register: str = ""
    # The dot-separated text pieces after the register name.
    args: tuple = ()


# The event the SUT must never do: a write (with its values) or a tock.
@dataclass(frozen=True)
class Forbidden:
    # Either "write" or "tock".
    kind: str
    name: str = ""
    values: tuple = ()


# A whole forbidden trace: complete cycles, then the open cycle that the forbidden event belongs to.
@dataclass
class Trace:
    tid: int
    # Cycles ended by a tock; these must match exactly.
    closed: list
    # The last cycle, where the forbidden event applies.
    open: Cycle
    forbidden: Forbidden


# Splits text on a separator, ignoring separators inside brackets or quotes.
def split_trace_outside_brackets(text, sep):
    return split_outside_brackets(text, sep)


# Turns one piece of text from a trace line into an Ev.
def parse_trace_event(text):
    text = text.strip()
    if text.rsplit("::", 1)[-1] == "tock":
        return Ev("tock")
    parts = split_trace_outside_brackets(text, ".")
    if len(parts) < 2:
        raise ValueError(f"cannot parse event {text!r}")
    action = parts[0].rsplit("::", 1)[-1]
    register = parts[1].rsplit("::", 1)[-1]
    return Ev(action, register, tuple(p.strip() for p in parts[2:]))


# Works out an event's value from its dot-separated pieces:
# no pieces means no value, one piece is that value, and several pieces make a tuple.
# So o_move.2.0 is the tuple (2, 0), and o_cur.2.5 is the tuple (2, 5), never the float 2.5.
# Floats only exist inside brackets, such as (2.5, 1).
def value_from_dotted_parts(parts):
    if not parts:
        return None
    if len(parts) == 1:
        return parse_value(parts[0])
    return tuple(parse_value(p) for p in parts)


# Wraps a value in a tuple, or gives an empty tuple when there is no value.
def _values_from_text(value):
    return () if value is None else (value,)


# Builds a Trace from a list of events: the last event is the forbidden one, and tocks split the rest into cycles.
def split_events_into_cycles(tid, events):
    if not events:
        raise ValueError("empty trace")

    # The last event is the one that must not happen.
    last = events[-1]
    if last.action == "tock":
        forbidden = Forbidden("tock")
    elif last.action == "registerWrite":
        forbidden = Forbidden("write", remove_name_prefix(last.register, "o_"), _values_from_text(value_from_dotted_parts(last.args)))
    else:
        raise ValueError(f"unsupported forbidden event kind {last.action!r}")

    # Everything before it is sorted into cycles; each tock closes the current cycle.
    closed, cur = [], Cycle()
    for ev in events[:-1]:
        if ev.action == "tock":
            closed.append(cur)
            cur = Cycle()
        elif ev.action == "registerRead":
            if not ev.args or ev.args[0] not in ("true", "false"):
                raise ValueError(f"registerRead must start with true or false: {ev.register} {list(ev.args)}")
            cur.reads.append((remove_name_prefix(ev.register, "i_"),
                              (ev.args[0] == "true",) + _values_from_text(value_from_dotted_parts(ev.args[1:]))))
        elif ev.action == "registerWrite":
            cur.writes.append((remove_name_prefix(ev.register, "o_"), _values_from_text(value_from_dotted_parts(ev.args))))
        else:
            raise ValueError(f"unsupported event kind {ev.action!r}")
    return Trace(tid, closed, cur, forbidden)


# Splits the body of a trace line into events.
def parse_trace_events(body):
    return [parse_trace_event(e) for e in split_trace_outside_brackets(body, ",")]


# Reads a whole .rtspec file and returns (traces, errors): {id: Trace} and {id: message} for traces that failed to parse.
def load_traces(path):
    traces, errors = {}, {}
    for line in Path(path).read_text().splitlines():
        m = TRACE_RE.match(line.strip())
        # Header comments, the 'cspm ...' line and the closing brace are not traces.
        if not m:
            continue
        tid, body = int(m.group(1)), m.group(2)
        try:
            traces[tid] = split_events_into_cycles(tid, parse_trace_events(body))
        except ValueError as e:
            errors[tid] = str(e)
    return traces, errors


# Writes the forbidden event as short text for the report.
def format_forbidden_event(f):
    return "tock" if f.kind == "tock" else format_observation((f.name, f.values))


# Lists the inputs the driver must send this cycle: the reads whose available flag is true.
def inputs_that_are_true(cycle):
    return [(n, v) for n, v in cycle.reads if v and v[0] is True]
