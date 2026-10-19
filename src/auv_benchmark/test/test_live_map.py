"""Incremental display updates must preserve distant geometry and remove stale data."""

import os
import struct

import numpy as np
import pytest

from auv_benchmark.adapters import live_map
from auv_benchmark.adapters.live_map import ChunkedMap, SnapshotUpdates, trajectory_points


def rows(*points):
    return np.array([(*point, 200, 210, 220) for point in points], dtype=float).reshape(-1, 6)


def test_local_addition_does_not_rebuild_distant_chunk():
    model = ChunkedMap(0.1, 4.0)
    model.update(rows((1, 1, 1), (30, 30, 30)))
    distant = model.chunks[(7, 7, 7)]
    changes = model.update(rows((1, 1, 1), (1.2, 1, 1), (30, 30, 30)))
    assert set(changes) == {(0, 0, 0)}
    assert model.chunks[(7, 7, 7)] is distant


def test_reordering_and_duplicates_do_not_trigger_upload():
    model = ChunkedMap()
    points = rows((-1, -1, -1), (0, 0, 0), (5, 5, 5))
    model.update(points)
    assert model.update(np.vstack([points[::-1], points[:1]])) == {}


def test_deletions_color_changes_and_mission_reset():
    model = ChunkedMap()
    points = rows((-1, -1, -1), (8, 0, 0))
    model.update(points)
    points[0, 3:] = [1, 2, 3]
    changes = model.update(points[:1])
    assert changes[(2, 0, 0)] is None
    np.testing.assert_array_equal(changes[(-1, -1, -1)][:, 3:], [[1, 2, 3]])
    assert model.update(np.empty((0, 6))) == {(-1, -1, -1): None}
    assert model.chunks == {}


def test_voxel_sampling_bounds_density_and_discards_nonfinite_points():
    model = ChunkedMap(0.2, 4)
    model.update(rows((0.01, 0, 0), (0.05, 0, 0), (np.nan, 0, 0)))
    assert sum(map(len, model.chunks.values())) == 1


def test_point_moves_within_display_voxel_and_deletion_replaces_chunk():
    model = ChunkedMap(.2, 4)
    model.update(rows((.01, 0, 0), (1, 1, 1)))
    changed = model.update(rows((.15, 0, 0), (1, 1, 1)))
    np.testing.assert_allclose(changed[(0, 0, 0)][:, :3], [[.15, 0, 0], [1, 1, 1]])
    changed = model.update(rows((.15, 0, 0)))
    np.testing.assert_allclose(changed[(0, 0, 0)][:, :3], [[.15, 0, 0]])


def test_downsampling_uses_measured_centroid_and_mean_color():
    model = ChunkedMap(.2, 4)
    points = rows((.01, 0, 0), (.05, 0, 0))
    points[1, 3:] = [100, 110, 120]
    changed = model.update(points)
    np.testing.assert_allclose(changed[(0, 0, 0)], [[.03, 0, 0, 150, 160, 170]])
    assert model.update(points[::-1]) == {}


def test_outlier_filter_removes_old_noise_and_restores_newly_supported_surface():
    pytest.importorskip('open3d')
    model = ChunkedMap(.05, 4, filter_outliers=True)
    noise = rows((12, 0, 0))
    # Preserve a new, small cloud rather than making it disappear immediately.
    model.update(noise)
    dense = rows(*[(x * .1, y * .1, 0) for x in range(6) for y in range(6)])
    changed = model.update(np.vstack([dense, noise]))
    assert changed[(3, 0, 0)] is None
    assert (3, 0, 0) not in model.chunks
    supported = dense.copy()
    supported[:, 0] += 12
    changed = model.update(np.vstack([dense, supported]))
    assert len(changed[(3, 0, 0)]) > 15
    assert model.update(np.vstack([supported[::-1], dense[::-1]])) == {}
    # Disabling display filtering exposes the original isolated occupancy.
    raw = ChunkedMap(.05, 4, filter_outliers=False)
    assert (3, 0, 0) in raw.update(np.vstack([dense, noise]))


def test_outlier_filter_uses_neighbors_across_chunk_boundaries():
    pytest.importorskip('open3d')
    points = rows(*[(3.7 + x * .1, .1 + y * .1, 0) for x in range(7) for y in range(7)])
    global_model = ChunkedMap(.05, 100, filter_outliers=True)
    chunked_model = ChunkedMap(.05, 4, filter_outliers=True)
    global_model.update(points)
    chunked_model.update(points)
    expected = np.vstack(list(global_model.chunks.values()))
    actual = np.vstack(list(chunked_model.chunks.values()))
    assert {0, 1} == {key[0] for key in chunked_model.chunks}
    np.testing.assert_allclose(actual, expected)


@pytest.mark.parametrize('neighbors,ratio', [(1, 1.5), (2.5, 1.5), (True, 1.5),
                                            (15, 0), (15, np.nan)])
def test_bad_outlier_filter_parameters_rejected(neighbors, ratio):
    with pytest.raises(ValueError):
        ChunkedMap(outlier_neighbors=neighbors, outlier_std_ratio=ratio)


@pytest.mark.parametrize('voxel,chunk', [(0, 4), (-1, 4), (1, .5), (np.nan, 4)])
def test_bad_display_geometry_rejected(voxel, chunk):
    with pytest.raises(ValueError):
        ChunkedMap(voxel, chunk)


def snapshot_bytes(masks, log_odds=1.):
    header = f'# Octomap OcTree file\nid ColorOcTree\nsize {len(masks)}\nres 0.2\ndata\n'
    return header.encode() + b''.join(struct.pack('<fBBBB', log_odds, 20, 30, 40, mask)
                                    for mask in masks)


def test_snapshot_validation_rejects_partial_or_inconsistent_tree():
    good = snapshot_bytes([1, 0])
    live_map.validate_snapshot(good)
    live_map.validate_snapshot(snapshot_bytes([]))
    for bad in (good[:-1], good + b'extra', snapshot_bytes([1]), snapshot_bytes([0, 0])):
        with pytest.raises(ValueError):
            live_map.validate_snapshot(bad)


def test_unchanged_snapshot_skips_decode_and_failed_snapshot_keeps_map(tmp_path, monkeypatch):
    path = tmp_path / 'map.bt'
    path.write_bytes(b'first')
    state = SnapshotUpdates(path, .2, 4)
    calls = []

    def read(_):
        calls.append(True)
        return live_map.signature(path), rows((1, 1, 1))

    monkeypatch.setattr(live_map, 'read_snapshot', read)
    assert state.poll()
    assert state.poll() == {}
    assert len(calls) == 1
    old = state.map.chunks[(0, 0, 0)]
    path.write_bytes(b'partial second snapshot')
    monkeypatch.setattr(live_map, 'read_snapshot', lambda _: None)
    assert state.poll() == {}
    assert state.map.chunks[(0, 0, 0)] is old
    path.unlink()
    assert state.poll() == {}
    assert state.map.chunks[(0, 0, 0)] is old


def test_native_reader_uses_occupied_leaf_centers(tmp_path):
    pytest.importorskip('pyoctomap')
    path = tmp_path / 'map.bt'
    # A single occupied leaf at depth 16, with no free-space expansion.
    path.write_bytes(snapshot_bytes([1] * 16 + [0]))
    os.utime(path, (1, 1))
    stamp, points = live_map.read_snapshot(path)
    assert stamp == live_map.signature(path)
    assert points.shape == (1, 6)
    np.testing.assert_allclose(points[0, :3], [-6553.5] * 3, atol=.001)
    np.testing.assert_array_equal(points[0, 3:], [20, 30, 40])


def test_native_occupied_free_empty_and_reoccupied_snapshots_remove_stale_chunks(tmp_path):
    pytest.importorskip('pyoctomap')
    path = tmp_path / 'map.bt'
    state = SnapshotUpdates(path, .2, 4)
    occupied = snapshot_bytes([1] * 16 + [0])
    for stamp, data in enumerate((occupied, snapshot_bytes([1] * 16 + [0], -2.),
                                  snapshot_bytes([]), occupied), start=1):
        path.write_bytes(data)
        os.utime(path, (stamp, stamp))
        changes = state.poll()
        if stamp in (1, 4):
            assert len(state.map.chunks) == 1
            assert sum(len(points) for points in changes.values()) == 1
        elif stamp == 2:
            assert len(changes) == 1
            assert next(iter(changes.values())) is None
            assert state.map.chunks == {}
        else:
            assert changes == {}
            assert state.map.chunks == {}


def test_trajectory_replacement_and_clear():
    np.testing.assert_array_equal(trajectory_points([1, 2, 3, 0, 4, 5, 6, 1]),
                                  [[1, 2, 3], [4, 5, 6]])
    assert trajectory_points([]).shape == (0, 3)
    for values in ([1, 2, 3], [1, 2, np.nan, 0], [1, 2, 3, np.inf]):
        with pytest.raises(ValueError):
            trajectory_points(values)
