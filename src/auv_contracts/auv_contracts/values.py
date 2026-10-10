"""Immutable values shared across policy boundaries; no middleware types."""
from dataclasses import dataclass
from types import MappingProxyType
from typing import Mapping, Any

def _freeze(value):
    if isinstance(value, dict):
        return MappingProxyType({key: _freeze(item) for key, item in value.items()})
    if isinstance(value, (list, tuple)):
        return tuple(_freeze(item) for item in value)
    return value

@dataclass(frozen=True)
class Configuration:
    values: Mapping[str, Any]
    def __post_init__(self):
        object.__setattr__(self, 'values', _freeze(dict(self.values)))
    def __getattr__(self, name):
        try:
            return self.values[name]
        except KeyError as error:
            raise AttributeError(name) from error

@dataclass(frozen=True)
class Vector3:
    x: float
    y: float
    z: float

@dataclass(frozen=True)
class Quaternion:
    x: float
    y: float
    z: float
    w: float

@dataclass(frozen=True)
class Stamp:
    sec: int
    nanosec: int

@dataclass(frozen=True)
class PoseSample:
    position: Vector3
    orientation: Quaternion
    stamp: Stamp

@dataclass(frozen=True)
class Trajectory:
    points: tuple[tuple[float, float, float, float], ...]
    def __post_init__(self):
        object.__setattr__(self, 'points', tuple(tuple(p) for p in self.points))

@dataclass(frozen=True)
class ArrayEvent:
    data: tuple[float, ...]
    def __post_init__(self):
        object.__setattr__(self, 'data', tuple(self.data))

@dataclass(frozen=True)
class Flag:
    data: bool

@dataclass(frozen=True)
class CameraCalibration:
    focal_length_px: float = 256.0
    baseline_meters: float = 0.5

@dataclass(frozen=True)
class TrialResult:
    coverage: float
    rmse: float
    distance: float
