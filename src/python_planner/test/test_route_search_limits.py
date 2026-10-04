"""Regression checks for the restored route-planning API."""

import ast
import inspect
from pathlib import Path

import numpy as np

from python_planner import route_planning as routes


def test_runtime_callers_match_restored_route_planning_signatures():
    package = Path(__file__).parents[1] / 'python_planner'
    checked = 0
    for source in package.glob('*.py'):
        tree = ast.parse(source.read_text())
        imports = {alias.asname or alias.name: alias.name
                   for node in tree.body if isinstance(node, ast.ImportFrom)
                   and node.module == 'route_planning' for alias in node.names}
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Name):
                continue
            name = imports.get(node.func.id)
            if name is None:
                continue
            assert all(keyword.arg is not None for keyword in node.keywords)
            signature = inspect.signature(getattr(routes, name))
            try:
                signature.bind(*[object() for _ in node.args],
                               **{keyword.arg: object() for keyword in node.keywords})
            except TypeError as error:
                raise AssertionError(f'{source.name}:{node.lineno}: {name}: {error}') from error
            checked += 1
    assert checked >= 10




def test_astar_preserves_detours_around_obstacles():
    start = np.array([.5, .5, .5])
    goal = np.array([3.5, .5, .5])
    occupied = {(1, 0, 0), (2, 0, 0)}
    path = routes.astar_3d(start, goal, occupied, 1.0)
    assert len(path) == 6  # Three horizontal and two detour steps.
    assert routes.path_is_collision_free(np.asarray(path), occupied, 1.0)



def test_resampling_handles_duplicate_and_stationary_points():
    stationary = np.tile([1., 2., 3., .5], (5, 1))
    result = routes.resample_trajectory_by_distance(
        stationary, .5, max_points=1)
    np.testing.assert_allclose(result, stationary[:1])
