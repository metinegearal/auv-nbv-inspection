"""Geometric regressions for missing boundaries, independent of ROS."""
from types import SimpleNamespace

import numpy as np
import pytest
from auv_planner_reference.domain.nbv_parameters import DEFAULT_NBV_PARAMETERS
from auv_planner_reference.use_cases.nbv import cluster_frontiers, detect_frontiers
from scipy.spatial import KDTree
from scipy.spatial.transform import Rotation


def sheet_with_hole():
    points = np.array([(x, y, 0.) for x in range(-8, 9) for y in range(-8, 9)
                       if not (abs(x) <= 1 and abs(y) <= 1)], dtype=float) * .25
    hole_edges = np.flatnonzero(
        ((abs(points[:, 0]) == .5) & (abs(points[:, 1]) <= .25))
        | ((abs(points[:, 1]) == .5) & (abs(points[:, 0]) <= .25)))
    return points, hole_edges


def cloud(points, normals=None):
    return SimpleNamespace(points=np.asarray(points, dtype=float),
                           normals=(np.tile([0., 0., 1.], (len(points), 1))
                                    if normals is None else normals))


def test_local_support_preserves_hole_edges_hidden_by_global_knn():
    points, hole_edges = sheet_with_hole()
    detected = set(detect_frontiers(cloud(points)))
    assert len(hole_edges) == 12
    assert set(hole_edges) <= detected
    assert not set(hole_edges).intersection(detect_frontiers(cloud(points), radius=None))
    # Solid interior samples must not turn into frontiers as a side effect.
    interior = np.flatnonzero((abs(points[:, 0]) == 1.) & (abs(points[:, 1]) <= 1.))
    assert not set(interior).intersection(detected)


def test_frontiers_are_rotation_translation_and_normal_scale_invariant():
    points, _ = sheet_with_hole()
    expected = detect_frontiers(cloud(points), radius=.51)
    rotation = Rotation.from_euler('xyz', [31., -48., 27.], degrees=True)
    moved = rotation.apply(points) + [300., 800., -300.]
    normals = rotation.apply(cloud(points).normals) * 7.
    assert detect_frontiers(cloud(moved, normals), radius=.51) == expected


def test_normal_direction_neighbors_cannot_fill_a_tangent_gap():
    # At index 0 the left half-plane is empty; the point directly above it has
    # no tangent angle and must not be inserted as atan2(0, 0) == 0.
    points = [[0., 0., 0.], [.2, 0., 0.], [0., .2, 0.], [0., -.2, 0.], [0., 0., .1]]
    assert 0 in detect_frontiers(cloud(points))


def test_isolated_samples_invalid_normals_and_duplicates_are_not_frontiers():
    points = [[0., 0., 0.], [.2, 0., 0.], [0., .2, 0.], [0., -.2, 0.], [4., 4., 0.]]
    normals = cloud(points).normals
    normals[0] = 0.
    normals[1] = np.nan
    detected = detect_frontiers(cloud(points, normals))
    assert not {0, 1, 4}.intersection(detected)
    assert detect_frontiers(cloud(np.zeros((10, 3)))) == []


def test_clustering_does_not_lose_valid_neighbors_to_an_underfilled_seed():
    points = np.array([[x, 0., 0.] for x in [0., .9, 1.1, 1.2, 1.3, 1.4]])
    normals = cloud(points).normals
    baseline = cluster_frontiers(points, normals, list(range(6)))
    assert len(baseline) == 1
    assert set(baseline[0]['global_indices']) == set(range(6))
    for seed in range(5):
        order = np.random.default_rng(seed).permutation(6)
        result = cluster_frontiers(points[order], normals[order], np.arange(6))
        np.testing.assert_allclose(result[0]['centroid'], baseline[0]['centroid'])
        assert len(result[0]['points']) == 6


def test_cluster_normal_validation_and_angle_constraint():
    points = np.array([[x, 0., 0.] for x in [0., .1, .2, .3, .4, .5]])
    normals = np.array([[0., 0., 3.]] * 3 + [[0., 0., -5.]] * 3)
    result = cluster_frontiers(points, normals, list(range(6)), min_samples=3)
    assert sorted(len(cluster['points']) for cluster in result) == [3, 3]
    normals[:] = 0.
    assert cluster_frontiers(points, normals, list(range(6)), min_samples=3) == []


@pytest.mark.parametrize('options', [
    {'radius': 0.}, {'radius': np.nan}, {'angle_threshold_deg': 360.},
    {'k': 3, 'min_neighbors': 3},
])
def test_invalid_frontier_configuration_has_a_clear_error(options):
    with pytest.raises(ValueError):
        detect_frontiers(cloud(np.zeros((5, 3))), **options)


def preprocess(points):
    pytest.importorskip('open3d')
    from auv_planner_reference.adapters.point_cloud import Open3DPreprocessor
    return Open3DPreprocessor().prepare(
        np.asarray(points), np.array([0., 0., 5.]), DEFAULT_NBV_PARAMETERS,
        lambda stage, operation: operation())


def test_preprocessing_preserves_real_corners_and_removes_isolated_noise():
    points = np.array([(x, y, 0.) for x in range(15) for y in range(15)], dtype=float) * .25
    boundary = ((points[:, 0] == 0.) | (points[:, 0] == 3.5)
                | (points[:, 1] == 0.) | (points[:, 1] == 3.5))
    # Verify the production Open3D adapter, not a mock statistical filter.
    cleaned = preprocess(points)
    distance, _ = KDTree(cleaned.points).query(points[boundary])
    np.testing.assert_allclose(distance, 0., atol=1e-10)
    cleaned = preprocess(np.vstack((points, [100., 100., 100.], [np.nan, 0., 0.])))
    assert len(cleaned.points) == len(points)
    assert np.isfinite(cleaned.normals).all()


def test_small_supported_patch_is_not_discarded_by_the_old_40_point_cutoff():
    points = np.array([(x, y, 0.) for x in range(5) for y in range(5)], dtype=float) * .25
    cleaned = preprocess(points)
    assert cleaned is not None and len(cleaned.points) == 25
    frontiers = detect_frontiers(cleaned)
    assert len(frontiers) == 16
    clusters = cluster_frontiers(cleaned.points, cleaned.normals, frontiers)
    assert sum(len(cluster['points']) for cluster in clusters) == 16


def test_full_preprocessing_keeps_hole_boundary_detectable():
    points, hole_edges = sheet_with_hole()
    cleaned = preprocess(points)
    distances, indices = KDTree(cleaned.points).query(points[hole_edges])
    np.testing.assert_allclose(distances, 0., atol=1e-10)
    assert set(indices) <= set(detect_frontiers(cleaned))
