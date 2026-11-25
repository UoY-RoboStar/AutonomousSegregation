# Typed values of STC (.rtspec) events.
# A value is one of: bool, int, float, str, Ident, tuple or Seq.
#   str is a quoted string, such as "start"
#   Ident is a bare name or enum literal, such as Chemical_Angle_Left or Mode::Active
#   tuple is (0, 1) or (| 0, 1 |), and may be nested, such as ((0, 0), (1, 1))
#   Seq is a sequence, such as <> or <(0, 0), (1, 0)>
# The text format is used for values inside brackets in STC and for values the probe sends to the driver.
# A bare dotted value in STC is handled differently (see value_from_dotted_parts in rtspec.py).
import re
from dataclasses import dataclass


# A bare name such as Left or Mode::Left.
@dataclass(frozen=True)
class Ident:
    text: str

    # The name without its type prefix, so Mode::Left gives Left.
    @property
    def short_name(self):
        return self.text.rsplit("::", 1)[-1]


# A sequence of values, such as <(0, 0), (1, 0)>.
@dataclass(frozen=True)
class Seq:
    items: tuple


INT_RE = re.compile(r"^-?\d+$")
FLOAT_RE = re.compile(r"^(-?\d+\.\d+([eE][-+]?\d+)?|-?\d+[eE][-+]?\d+|nan|-?inf)$")
IDENT_RE = re.compile(r"^[A-Za-z_]\w*(::[A-Za-z_]\w*)*$")


# Splits text on a separator, ignoring separators inside (), <> or double quotes. The '->' arrow does not count as a bracket.
def split_outside_brackets(text, sep):
    parts, depth, cur, quoted = [], 0, [], False
    prev = ""
    for ch in text:
        if ch == '"':
            quoted = not quoted
        elif not quoted:
            if ch in "(<":
                depth += 1
            elif ch == ")" or (ch == ">" and prev != "-"):
                depth -= 1
        if ch == sep and depth == 0 and not quoted:
            parts.append("".join(cur))
            cur = []
        else:
            cur.append(ch)
        prev = ch
    parts.append("".join(cur))
    return parts


# Parses the comma-separated inside of a tuple or sequence into a tuple of values.
def _parse_sequence_items(inner):
    inner = inner.strip()
    return tuple(parse_value(p) for p in split_outside_brackets(inner, ",")) if inner else ()


# Turns text into a typed value (bool, int, float, str, Ident, tuple or Seq).
def parse_value(text):
    s = text.strip()
    if not s:
        raise ValueError("empty value")
    if s == "true":
        return True
    if s == "false":
        return False
    # A quoted string, unless it holds a sequence.
    if len(s) >= 2 and s[0] == s[-1] == '"':
        inner = s[1:-1]
        if inner.startswith("<") and inner.endswith(">"):
            return Seq(_parse_sequence_items(inner[1:-1]))
        return inner
    if INT_RE.match(s):
        return int(s)
    if FLOAT_RE.match(s):
        return float(s)
    if s.startswith("(|") and s.endswith("|)"):
        return _parse_sequence_items(s[2:-2])
    if s.startswith("<") and s.endswith(">"):
        return Seq(_parse_sequence_items(s[1:-1]))
    # Brackets with one item are just that item.
    if s.startswith("(") and s.endswith(")"):
        items = _parse_sequence_items(s[1:-1])
        return items[0] if len(items) == 1 else items
    if IDENT_RE.match(s):
        return Ident(s)
    raise ValueError(f"unsupported value {s!r}")


# True for ints and floats, but not for bools.
def _parse_number(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool)


# Gives the text of a name or string, so the two can be compared.
def _parse_text(v):
    return v.text if isinstance(v, Ident) else v


# Checks whether two values are the same. Numbers may differ by `tol`, and 1 equals 1.0.
def values_equal(a, b, tol=1e-6):
    if _parse_number(a) and _parse_number(b):
        return a == b or abs(a - b) <= tol
    if isinstance(a, bool) or isinstance(b, bool):
        return type(a) is type(b) and a == b
    if isinstance(a, Ident) and isinstance(b, Ident):
        return a.text == b.text or (("::" not in a.text or "::" not in b.text) and a.short_name == b.short_name)
    # A bare name and a string with the same text count as equal.
    if isinstance(a, Ident) or isinstance(b, Ident):
        x, y = _parse_text(a), _parse_text(b)
        return isinstance(x, str) and isinstance(y, str) and (x == y or x == y.rsplit("::", 1)[-1]
                                                              or y == x.rsplit("::", 1)[-1])
    if isinstance(a, Seq) and isinstance(b, Seq):
        return len(a.items) == len(b.items) and all(values_equal(x, y, tol) for x, y in zip(a.items, b.items))
    if isinstance(a, tuple) and isinstance(b, tuple):
        return len(a) == len(b) and all(values_equal(x, y, tol) for x, y in zip(a, b))
    if isinstance(a, str) and isinstance(b, str):
        return a == b
    return False


# Writes a value as short, readable text for reports.
def format_value(v):
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, float):
        return str(int(v)) if v.is_integer() else repr(v)
    if isinstance(v, str):
        return f'"{v}"'
    if isinstance(v, Ident):
        return v.text
    if isinstance(v, Seq):
        return "<" + ", ".join(format_value(x) for x in v.items) + ">"
    if isinstance(v, tuple):
        return "(" + ", ".join(format_value(x) for x in v) + ")"
    return str(v)


# Flattens a value into its leaves, in order: tuples (even nested ones) are opened up, anything else stays as one leaf.
def spread_tuple_into_leaves(v):
    if isinstance(v, tuple):
        return [leaf for x in v for leaf in spread_tuple_into_leaves(x)]
    return [v]
