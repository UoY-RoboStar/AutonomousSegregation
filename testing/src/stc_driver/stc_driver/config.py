# Reads the SUT description file (YAML) that makes the driver generic.
import array
import re
import shlex
from dataclasses import dataclass
from pathlib import Path

import yaml

from .values import Ident, Seq, format_value, spread_tuple_into_leaves


# Raised when the config file is missing, unreadable or wrong.
class ConfigError(ValueError):
    pass


TYPE_RE = re.compile(r"^\w+/msg/\w+$")
TOP_KEYS = {"sut_cmd", "cycle_period", "timeout", "inputs"}
INPUT_KEYS = {"topic", "type", "fields", "enums"}


# How one input register is sent: its topic, message type, field mapping and enum numbers.
@dataclass(frozen=True)
class InputSpec:
    topic: str
    # For example std_msgs/msg/Bool.
    type: str
    # One entry per leaf: a field path ("linear.x" is allowed), None to skip it, or ("path", (item paths...)) for a
    # sequence of tuples.
    fields: tuple
    # Pairs of (literal, number).
    enums: tuple = ()


# Everything the config file says.
@dataclass(frozen=True)
class Config:
    sut_cmd: list
    cycle_period: float
    timeout: float
    # Maps a register name to its InputSpec.
    inputs: dict


# Raises an error if a mapping has keys that are not allowed.
def _reject_unknown_keys(where, got, allowed):
    extra = sorted(set(got) - allowed)
    if extra:
        raise ConfigError(f"{where}: unknown key(s) {extra}; allowed: {sorted(allowed)}")


# Reads the enum numbers of an input entry: a mapping from literal to integer.
def _read_enum_pairs(where, spec):
    enums = spec.get("enums", {})
    if not isinstance(enums, dict) or not all(isinstance(k, str) and isinstance(v, int) and not isinstance(v, bool)
                                              for k, v in enums.items()):
        raise ConfigError(f"{where}.enums: expected a mapping literal -> integer")
    return tuple(enums.items())


# Reads and checks the YAML config file and returns a Config.
def load_config(path):
    try:
        raw = yaml.safe_load(Path(path).read_text())
    except (OSError, yaml.YAMLError) as e:
        raise ConfigError(f"cannot read {path}: {e}") from None
    if not isinstance(raw, dict):
        raise ConfigError(f"{path}: expected a mapping at the top level")
    _reject_unknown_keys(str(path), raw, TOP_KEYS)

    # The command that starts the SUT.
    cmd = raw.get("sut_cmd")
    if not isinstance(cmd, str) or not cmd.strip():
        raise ConfigError("sut_cmd: a command string is required")

    # Reads a number setting that must be positive.
    def read_number(key, default):
        v = raw.get(key, default)
        if isinstance(v, bool) or not isinstance(v, (int, float)) or v <= 0:
            raise ConfigError(f"{key}: must be a positive number, got {v!r}")
        return float(v)

    # Each input register: its topic, message type, field mapping and enum numbers.
    inputs_raw = raw.get("inputs")
    if not isinstance(inputs_raw, dict) or not inputs_raw:
        raise ConfigError("inputs: at least one input register is required")
    inputs = {}
    for name, spec in inputs_raw.items():
        if not isinstance(spec, dict):
            raise ConfigError(f"inputs.{name}: expected topic/type/fields")
        _reject_unknown_keys(f"inputs.{name}", spec, INPUT_KEYS)
        topic, typ = spec.get("topic"), spec.get("type")
        if not isinstance(topic, str) or not topic.startswith("/"):
            raise ConfigError(f"inputs.{name}.topic: absolute topic name required (e.g. /{name})")
        if not isinstance(typ, str) or not TYPE_RE.match(typ):
            raise ConfigError(f"inputs.{name}.type: expected <package>/msg/<Name>, got {typ!r}")
        # The field list: a name, null to skip, or a path with the fields of each item.
        fields = spec.get("fields", ["data"])
        if not isinstance(fields, list):
            raise ConfigError(f"inputs.{name}.fields: expected a list")
        parsed = []
        for f in fields:
            if f is None or isinstance(f, str):
                parsed.append(f)
            elif isinstance(f, dict) and set(f) == {"path", "items"} and isinstance(f["path"], str) \
                    and isinstance(f["items"], list) and all(isinstance(i, str) for i in f["items"]):
                parsed.append((f["path"], tuple(f["items"])))
            else:
                raise ConfigError(f"inputs.{name}.fields: each entry is a field name, null, or "
                                  f"{{path: <field>, items: [<subfield>, ...]}}; got {f!r}")
        inputs[str(name)] = InputSpec(topic, typ, tuple(parsed), _read_enum_pairs(f"inputs.{name}", spec))

    return Config(shlex.split(cmd), read_number("cycle_period", 0.1), read_number("timeout", 5.0), inputs)


# Turns a read, (available, value?), into a flat list: the available flag, then the leaves of the value.
def _list_leaf_paths(values):
    return [values[0]] + (spread_tuple_into_leaves(values[1]) if len(values) > 1 else [])


# Converts a typed value into a Python value of the type the message field already has (`kind`).
def _convert_to_field_type(kind, v, enums, where):
    if kind is bool:
        if isinstance(v, bool):
            return v
    elif kind is int:
        if isinstance(v, Ident) and (v.text in enums or v.short_name in enums):
            return enums[v.text] if v.text in enums else enums[v.short_name]
        if isinstance(v, (int, float)) and not isinstance(v, bool):
            return int(round(v))
        if isinstance(v, bool):
            return int(v)
    elif kind is float:
        if isinstance(v, (int, float)) and not isinstance(v, bool):
            return float(v)
    elif kind is str:
        if isinstance(v, str):
            return v
        if isinstance(v, Ident):
            return v.short_name
    else:
        raise ConfigError(f"field {where!r} has type {kind.__name__}; supported: bool, int, float, string, "
                          f"arrays of those, and (with items) arrays of messages")
    raise ConfigError(f"field {where!r} is a {kind.__name__} but the trace gives {format_value(v)}")


# Finds the object that holds a field (following dots such as "linear.x") and returns it with the field's name.
def _find_message_field(msg, path):
    *parents, last = path.split(".")
    obj = msg
    try:
        for p in parents:
            obj = getattr(obj, p)
        getattr(obj, last)
    except AttributeError:
        raise ConfigError(f"message has no field {path!r}") from None
    return obj, last


# Finds the message class of the items in an array field, such as geometry_msgs/Point.
def _find_array_element_class(obj, last):
    # Only needed for arrays of messages, so it is imported here.
    from rosidl_runtime_py.utilities import get_message
    ftype = obj.get_fields_and_field_types()[last]
    m = re.search(r"(\w+)/(?:msg/)?(\w+)", ftype)
    if not m:
        raise ConfigError(f"cannot find the element type of field {last!r} ({ftype})")
    return get_message(f"{m.group(1)}/msg/{m.group(2)}")


# Fills an array field from a sequence value: either an array of messages (when `items` is given) or an array of plain values.
def _set_sequence_field(obj, last, path, v, items, enums):
    if not isinstance(v, Seq):
        raise ConfigError(f"field {path!r} is an array but the trace gives {format_value(v)}")
    cur = getattr(obj, last)
    # An array of messages: make one message per element and fill its fields.
    if items is not None:
        cls = _find_array_element_class(obj, last)
        out = []
        for element in v.items:
            leaves = spread_tuple_into_leaves(element)
            if len(leaves) != len(items):
                raise ConfigError(f"field {path!r}: item {format_value(element)} has {len(leaves)} value(s) "
                                  f"but items lists {len(items)} field(s)")
            item = cls()
            for sub, leaf in zip(items, leaves):
                o, l = _find_message_field(item, sub)
                setattr(o, l, _convert_to_field_type(type(getattr(o, l)), leaf, enums, f"{path}[].{sub}"))
            out.append(item)
        setattr(obj, last, out)
        return
    # An array of plain values: work out the element type, then convert each one.
    if isinstance(cur, array.array):
        kind = float if cur.typecode in "fd" else int
    else:
        kind = type(v.items[0]) if v.items else str
        kind = {bool: bool, int: int, float: float, str: str, Ident: str}.get(kind)
        if kind is None:
            raise ConfigError(f"field {path!r}: give a sequence of tuples an 'items' list")
    setattr(obj, last, [_convert_to_field_type(kind, x, enums, path) for x in v.items])


# Sets a message's fields from a typed read, (available,) or (available, value).
# The leaves are matched to `fields` in order, and each is converted to the type the field already has.
def fill_message_from_values(msg, fields, values, enums=()):
    enums = dict(enums)
    leaves = _list_leaf_paths(values)
    if len(fields) != len(leaves):
        raise ConfigError(f"the trace gives {len(leaves)} value(s) {[format_value(x) for x in leaves]} but 'fields' "
                          f"has {len(fields)} entries {list(fields)}")
    for spec, v in zip(fields, leaves):
        if spec is None:
            continue
        path, items = (spec, None) if isinstance(spec, str) else spec
        obj, last = _find_message_field(msg, path)
        if isinstance(v, Seq):
            _set_sequence_field(obj, last, path, v, items, enums)
        else:
            setattr(obj, last, _convert_to_field_type(type(getattr(obj, last)), v, enums, path))
