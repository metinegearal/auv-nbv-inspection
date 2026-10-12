"""Compare batched NBV math with the scalar formulas, without Open3D or ROS."""

import ast
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial import KDTree


source = Path(__file__).parents[1] / 'python_planner' / 'use_cases/nbv.py'
names = {'calculate_pca_complexity', 'detect_frontiers', 'cluster_frontiers',
         'evaluate_viewpoint'}
tree = ast.parse(source.read_text())
helpers = ast.Module(body=[node for node in tree.body
                          if isinstance(node, ast.FunctionDef) and node.name in names],
                     type_ignores=[])
math_functions = {'np': np, 'KDTree': KDTree}
exec(compile(helpers, str(source), 'exec'), math_functions)


@pytest.mark.parametrize('weight,expected', [(0., 3.), (5., 6.), (15., 12.)])
def test_viewpoint_gain_uses_configured_complexity_weight(weight, expected):
    gain = math_functions['evaluate_viewpoint'](
        np.array([0., 0., 1.]), np.zeros((3, 3)),
        np.tile([0., 0., 1.], (3, 1)), np.array([.1, .2, .3]),
        complexity_weight=weight)
    assert gain == pytest.approx(expected)


@pytest.mark.parametrize('count', [40, 121, 1050])
def test_pca_matches_original_covariance_formula(count):
    points = np.random.default_rng(12).normal(size=(count, 3))
    expected = []
    neighbor_tree = KDTree(points)
    for point in points:
        neighbors = points[neighbor_tree.query(point, k=20)[1]]
        centered = neighbors - neighbors.mean(axis=0)
        values = np.linalg.eigvalsh(np.cov(centered.T))
        expected.append(values[0] / values.sum())
    result = math_functions['calculate_pca_complexity'](SimpleNamespace(points=points))
    np.testing.assert_allclose(result, expected, atol=1e-13)


@pytest.mark.parametrize('count', [40, 121, 1050])
def test_frontiers_match_original_angular_gap_formula(count):
    rng = np.random.default_rng(16)
    points = rng.normal(size=(count, 3))
    normals = rng.normal(size=(count, 3))
    normals /= np.linalg.norm(normals, axis=1, keepdims=True)
    normals[:2] = [[0, 0, 1], [0, 0, -1]]
    neighbor_tree = KDTree(points)
    expected = []
    for i, (point, normal) in enumerate(zip(points, normals)):
        u = np.cross(normal, [0, 0, 1])
        if np.linalg.norm(u) < 1e-4:
            u = np.cross(normal, [0, 1, 0])
        u /= np.linalg.norm(u)
        v = np.cross(normal, u)
        neighbors = points[neighbor_tree.query(point, k=40)[1]]
        angles = sorted(np.arctan2(np.dot(other - point, v),
                                  np.dot(other - point, u))
                        for other in neighbors if np.linalg.norm(other - point) >= 1e-5)
        if angles and np.max(np.diff(angles + [angles[0] + 2 * np.pi])) > np.radians(130):
            expected.append(i)
    assert math_functions['detect_frontiers'](SimpleNamespace(
        points=points, normals=normals), radius=None) == expected


@pytest.mark.parametrize('count', [0, 1, 2, 39])
def test_frontiers_handle_small_and_duplicate_clouds(count):
    pcd = SimpleNamespace(points=np.zeros((count, 3)),
                          normals=np.tile([0, 0, 1], (count, 1)))
    assert math_functions['detect_frontiers'](pcd) == []


def test_greedy_clustering_keeps_order_and_discards_small_clusters():
    points = np.array([[x + dx, 0, 0] for x in [0, 3] for dx in [0, .1, .2]]
                      + [[9, 0, 0]], dtype=float)
    normals = np.tile([0, 0, 1], (len(points), 1))
    result = math_functions['cluster_frontiers'](
        points, normals, list(range(len(points))), min_samples=3)
    assert [cluster['id'] for cluster in result] == [0, 1]
    assert [cluster['global_indices'].tolist() for cluster in result] == [[0, 1, 2], [3, 4, 5]]
    np.testing.assert_allclose(result[0]['centroid'], [.1, 0, 0])


def test_viewpoint_gain_matches_original_and_skips_coincident_point():
    rng = np.random.default_rng(21)
    position = np.array([1, 2, 3])
    points = rng.normal(size=(50, 3))
    points[0] = position
    normals = rng.normal(size=(50, 3))
    complexities = rng.uniform(0, .3, 50)
    expected = 0.0
    for point, normal, complexity in zip(points, normals, complexities):
        ray = point - position
        if np.linalg.norm(ray) < 1e-4:
            continue
        expected += np.clip(np.dot(-ray / np.linalg.norm(ray), normal), 0, 1) * (1 + 15 * complexity)
    actual = math_functions['evaluate_viewpoint'](position, points, normals, complexities)
    np.testing.assert_allclose(actual, expected)
