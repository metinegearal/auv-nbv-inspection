"""Verify that reduced planning work preserves geometry and scoring quality."""
from collections import deque
from types import SimpleNamespace

import numpy as np
import pytest
from python_planner.route_planning import astar_3d, path_is_collision_free
from python_planner.use_cases.nbv import (
    calculate_kinematic_cost, calculate_kinematic_costs, calculate_pca_complexity,
    detect_frontiers, evaluate_viewpoint, evaluate_viewpoints,
)
from scipy.spatial import KDTree


def test_sparse_pca_uses_the_full_cloud_neighborhood_without_scoring_unused_points():
    rng = np.random.default_rng(51)
    points = rng.normal(size=(300, 3))
    cloud = SimpleNamespace(points=points)
    selected = np.array([3, 23, 102, 278])
    queries = []
    tree = KDTree(points)

    def query(values, **kwargs):
        queries.extend(values)
        return tree.query(values, **kwargs)

    expected = calculate_pca_complexity(cloud, k=20)
    result = calculate_pca_complexity(
        cloud, k=20, indices=selected, tree=SimpleNamespace(query=query))
    np.testing.assert_allclose(result[selected], expected[selected], rtol=0., atol=1e-14)
    np.testing.assert_array_equal(np.asarray(queries), points[selected])
    assert np.count_nonzero(result) == len(selected)
    assert not calculate_pca_complexity(cloud, indices=[]).any()


def test_reused_tree_and_sparse_radius_rows_preserve_frontiers():
    points = np.array([(x, y, 0.) for x in range(9) for y in range(9)], dtype=float) * .25
    points = np.vstack((points, np.arange(400, 700).reshape(-1, 3)))
    cloud = SimpleNamespace(points=points, normals=np.tile([0., 0., 1.], (len(points), 1)))
    result = detect_frontiers(cloud, tree=KDTree(points))
    expected = [index for index, (x, y, z) in enumerate(points[:81])
                if x in (0., 2.) or y in (0., 2.)]
    assert result == expected


def test_focused_frontiers_preserve_global_indices_across_batches_and_sparse_rows():
    points = np.array([(x, y, 0.) for x in range(45) for y in range(45)]) * .25
    points = np.vstack((points, np.arange(6000, 6300).reshape(-1, 3)))
    cloud = SimpleNamespace(points=points, normals=np.tile([0., 0., 1.], (len(points), 1)))
    selected = np.random.default_rng(18).permutation(len(points))[:1500]
    full = set(detect_frontiers(cloud))
    queries = []
    tree = KDTree(points)

    def query(values, **kwargs):
        queries.extend(values)
        return tree.query(values, **kwargs)

    result = detect_frontiers(
        cloud, focus_indices=selected, tree=SimpleNamespace(query=query))
    assert result == [int(index) for index in selected if index in full]
    np.testing.assert_array_equal(queries, points[selected])


def test_focus_window_does_not_create_boundaries_by_cropping_neighbors():
    points = np.array([(x, y, 0.) for x in range(9) for y in range(9)]) * .25
    normals = np.tile([0., 0., 1.], (len(points), 1))
    cloud = SimpleNamespace(points=points, normals=normals)
    selected = np.array([x * 9 + y for x in range(3, 6) for y in range(3, 6)])
    assert detect_frontiers(cloud, focus_indices=selected) == []
    cropped = SimpleNamespace(points=points[selected], normals=normals[selected])
    assert len(detect_frontiers(cropped)) == 8
    # An empty focus must not issue any neighborhood queries.
    assert detect_frontiers(cloud, focus_indices=[], tree=SimpleNamespace()) == []


@pytest.mark.parametrize('selected', [[-1], [81], [[0, 1]]])
def test_focused_frontiers_reject_invalid_global_indices(selected):
    cloud = SimpleNamespace(points=np.zeros((81, 3)), normals=np.ones((81, 3)))
    with pytest.raises(ValueError, match='focus indices'):
        detect_frontiers(cloud, focus_indices=selected)


@pytest.mark.parametrize('point_count', [7, 2048])
def test_batched_gain_matches_scalar_formula_including_coincident_points(point_count):
    rng = np.random.default_rng(9)
    points = rng.normal(size=(point_count, 3))
    normals = rng.normal(size=points.shape)
    normals /= np.linalg.norm(normals, axis=1, keepdims=True)
    complexities = rng.uniform(0., .3, point_count)
    positions = rng.normal(size=(70, 3))
    positions[0] = points[0]
    expected = [evaluate_viewpoint(position, points, normals, complexities, complexity_weight=30.)
                for position in positions]
    result = evaluate_viewpoints(positions, points, normals, complexities, complexity_weight=30.)
    np.testing.assert_allclose(result, expected, rtol=1e-13, atol=1e-13)
    assert evaluate_viewpoints([], points, normals, complexities).shape == (0,)


def test_batched_cost_matches_scalar_turning_and_collision_formula():
    rng = np.random.default_rng(11)
    positions = rng.normal(size=(90, 3))
    auv, target = np.array([.1, .2, .3]), np.array([.7, -.8, .5])
    positions[0], positions[1] = auv, target
    heading = np.array([1., 0., 0.])
    tree = KDTree(rng.normal(size=(100, 3)))
    options = {'distance_weight': 1.2, 'rotation_weight': .3, 'collision_weight': 2.5,
               'collision_distance_threshold': .8, 'collision_distance_offset': .2}
    expected = [calculate_kinematic_cost(auv, heading, position, target, tree, **options)
                for position in positions]
    result = calculate_kinematic_costs(auv, heading, positions, target, tree, **options)
    np.testing.assert_allclose(result, expected, rtol=1e-13, atol=1e-13)


@pytest.mark.parametrize('seed', range(15))
def test_astar_matches_exhaustive_shortest_path_with_obstacles(seed):
    rng = np.random.default_rng(seed)
    shape = (6, 6, 3)
    start, goal = (0, 0, 0), (5, 5, 2)
    cells = list(np.ndindex(shape))
    occupied = {cell for cell in cells if rng.random() < .25 and cell not in (start, goal)}
    # Close the test volume so unreachable outcomes can be compared with BFS.
    occupied.update((x, y, z) for x in range(-1, 7) for y in range(-1, 7)
                    for z in range(-1, 4) if x in (-1, 6) or y in (-1, 6) or z in (-1, 3))
    directions = ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1))
    queue, visited, distance = deque([(start, 0)]), {start}, None
    while queue:
        point, steps = queue.popleft()
        if point == goal:
            distance = steps
            break
        for offset in directions:
            neighbor = tuple(point[axis] + offset[axis] for axis in range(3))
            if neighbor not in visited and neighbor not in occupied:
                visited.add(neighbor)
                queue.append((neighbor, steps + 1))
    path = astar_3d(np.asarray(start) + .5, np.asarray(goal) + .5, occupied, 1., max_steps=500)
    if distance is None:
        assert path == []
    else:
        assert len(path) - 1 == distance
        assert path_is_collision_free(np.asarray(path), occupied, 1.)
        np.testing.assert_array_equal(np.abs(np.diff(path, axis=0)).sum(axis=1), 1.)


def test_goal_directed_ties_avoid_expanding_an_open_equal_cost_volume():
    start, goal = np.array([.5, .5, .5]), np.array([100.5, 100.5, 30.5])
    # The optimal path needs 230 edges; allow exactly its 231 node expansions.
    path = astar_3d(start, goal, set(), 1., max_steps=231)
    assert len(path) == 231
    assert astar_3d(start, goal, set(), 1., max_steps=230) == []
    np.testing.assert_array_equal(path[-1], goal)
