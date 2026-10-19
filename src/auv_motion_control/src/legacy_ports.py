"""Optional ports for the unbuilt, legacy 2D planner."""

from typing import Protocol


class ExplorationMemory(Protocol):

    def add_waypoint(self, point) -> None: ...
    def is_scanned(self, point) -> bool: ...


class Distance(Protocol):

    def __call__(self, first, second) -> float: ...
