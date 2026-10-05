from dataclasses import dataclass
from types import MappingProxyType
from typing import Protocol

@dataclass(frozen=True)
class ReportSnapshot:
    trajectory_log: tuple
    diagnostic_log: tuple
    total_distance: float
    metrics: object
    show_reference_objects: bool
    def __post_init__(self):
        object.__setattr__(self, 'trajectory_log', tuple(tuple(v) for v in self.trajectory_log))
        object.__setattr__(self, 'diagnostic_log', tuple(MappingProxyType(dict(v)) for v in self.diagnostic_log))
        object.__setattr__(self, 'metrics', MappingProxyType(dict(self.metrics)))

class ReportWriter(Protocol):
    def write(self, snapshot: ReportSnapshot, total_time: float) -> None: ...

class EvaluationGeometry(Protocol):
    def prepare(self): ...
    def map_distances(self): ...
    def ground_truth_distances(self): ...
    def visualize(self, distances): ...

@dataclass(frozen=True)
class MapEvaluationConfig:
    octomap_file: str
    ground_truth_file: str
    obj_center_x: float = -13.0
    obj_center_y: float = -81.9
    surface_z: float = -32.0
