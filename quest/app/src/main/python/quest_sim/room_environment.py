"""Immutable bounded room boxes in the simulator's Z-up world, in metres."""
from dataclasses import dataclass
import json
import math
import struct

MAX_COLLIDERS = 64
KINDS = ("floor", "wall", "table")
# Match the native .005F boundary exactly, including its JSON round trip.
MIN_HALF_EXTENT = struct.unpack("<f", struct.pack("<f", .005))[0]


@dataclass(frozen=True)
class RoomCollider:
    kind: str
    pose: tuple[float, ...]
    half_extents: tuple[float, ...]


@dataclass(frozen=True)
class RoomEnvironment:
    revision: int = 0
    enabled: bool = False
    colliders: tuple[RoomCollider, ...] = ()

    def counts(self):
        return {kind: sum(c.kind == kind for c in self.colliders) for kind in KINDS}

    def metadata(self):
        return dict(enabled=self.enabled, revision=self.revision,
                    collider_count=len(self.colliders), counts=self.counts())


def _vector(value, size, name):
    if (not isinstance(value, list) or len(value) != size
            or any(type(v) not in (int, float) for v in value)):
        raise ValueError(f"{name} must contain {size} finite numbers")
    try:
        result = tuple(float(v) for v in value)
    except OverflowError as error:
        raise ValueError(f"{name} contains an out-of-range number") from error
    if not all(math.isfinite(v) for v in result):
        raise ValueError(f"{name} must contain {size} finite numbers")
    return result


def _fields(value, expected, name):
    if not isinstance(value, dict) or set(value) != set(expected):
        raise ValueError(f"{name} fields must be {', '.join(expected)}")


def _unique_object(pairs):
    result = {}
    for name, value in pairs:
        if name in result:
            raise ValueError(f"duplicate JSON key: {name}")
        result[name] = value
    return result


def parse_environment(raw):
    """Validate the complete batch before a live scene can be touched."""
    incoming = json.loads(raw, object_pairs_hook=_unique_object)
    _fields(incoming, ("version", "revision", "enabled", "colliders"), "environment")
    if type(incoming["version"]) is not int or incoming["version"] != 1:
        raise ValueError("environment version must be 1")
    revision, enabled = incoming["revision"], incoming["enabled"]
    if type(revision) is not int or not 0 <= revision < 2**64:
        raise ValueError("environment revision must be a uint64 integer")
    if type(enabled) is not bool:
        raise ValueError("environment enabled must be boolean")
    if enabled and revision == 0:
        raise ValueError("enabled environment requires a positive revision")
    records = incoming["colliders"]
    if not isinstance(records, list) or len(records) > MAX_COLLIDERS:
        raise ValueError(f"environment colliders must be a list of at most {MAX_COLLIDERS} boxes")
    if not enabled and records:
        raise ValueError("disabled environment must have no colliders")
    colliders = []
    for record in records:
        _fields(record, ("kind", "pose", "half_extents"), "collider")
        kind = record["kind"]
        if kind not in KINDS:
            raise ValueError("collider kind must be floor, wall or table")
        pose = _vector(record["pose"], 7, "collider pose")
        if any(abs(v) > 100. for v in pose[:3]):
            raise ValueError("collider position must be within +/-100 metres")
        if abs(math.hypot(*pose[3:]) - 1.) > 1e-3:
            raise ValueError("collider quaternion must be unit length within 1e-3")
        half_extents = _vector(record["half_extents"], 3, "collider half_extents")
        if any(not MIN_HALF_EXTENT <= v <= 20. for v in half_extents):
            raise ValueError("collider half extents must be in [float32(.005), 20] metres")
        colliders.append(RoomCollider(kind, pose, half_extents))
    result = RoomEnvironment(revision, enabled, tuple(colliders))
    if enabled and not result.counts()["floor"]:
        raise ValueError("enabled environment requires a floor collider")
    return result
