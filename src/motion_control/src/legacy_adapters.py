"""
Explicit integration with optional repository-level legacy helpers.

Those helpers must be importable through the caller's normal Python environment.
This dormant adapter is not installed by the C++ package and is not launched.
"""


class RepositoryExplorationMemory:

    def __init__(self):
        from Tools.explore2D import addWaypoint, isItScanned

        self._add = addWaypoint
        self._scanned = isItScanned

    def add_waypoint(self, point):
        return self._add(point)

    def is_scanned(self, point):
        return self._scanned(point)


def repository_distance(first, second):
    from Tools.mapping2D import diff

    return diff(first, second)
