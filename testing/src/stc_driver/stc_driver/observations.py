# What the SUT did in a cycle, shared by STC parsing and transport code.
# An observation is (register_name, vals), where vals is a tuple of typed values:
#   a read is (available,) or (available, value), where available is a bool
#   a write is () or (value,)
from dataclasses import dataclass, field

from .values import Ident, format_value, values_equal


# What the SUT reads and writes in one cycle.
@dataclass
class Cycle:
    reads: list = field(default_factory=list)
    writes: list = field(default_factory=list)


# Removes a prefix such as i_ or o_ from a name, if it is there.
def remove_name_prefix(name, prefix):
    return name[len(prefix):] if name.startswith(prefix) else name


# Gives a write a comparable form: enum literals may be represented as part of the event name
# or as the value, and both forms count as the same event.
def _comparable_write(obs):
    name, vals = obs
    if len(vals) == 1 and isinstance(vals[0], Ident):
        return f"{name}.{vals[0].short_name}", ()
    return name, vals


# Checks whether two observations are the same; set read=True when they are reads.
def observations_equal(a, b, tol=1e-6, read=False):
    if read:
        if a[0] != b[0] or a[1][0] != b[1][0]:
            return False
        # If the input is not available its value means nothing, so only the flag is compared.
        if not a[1][0]:
            return True
        return len(a[1]) == len(b[1]) and all(values_equal(x, y, tol) for x, y in zip(a[1][1:], b[1][1:]))
    (na, va), (nb, vb) = _comparable_write(a), _comparable_write(b)
    return na == nb and len(va) == len(vb) and all(values_equal(x, y, tol) for x, y in zip(va, vb))


# Checks whether two lists of observations match, in the same order.
def observation_lists_equal(expected, actual, tol=1e-6, read=False):
    return len(expected) == len(actual) and all(observations_equal(e, a, tol, read) for e, a in zip(expected, actual))


# Writes an observation as short text for the report.
def format_observation(obs, read=False):
    name, vals = obs
    if read:
        return f"{name}({', '.join(format_value(v) for v in vals)})"
    if len(vals) == 1 and isinstance(vals[0], tuple):
        return f"{name}({', '.join(format_value(v) for v in vals[0])})"
    return f"{name}({', '.join(format_value(v) for v in vals)})"
