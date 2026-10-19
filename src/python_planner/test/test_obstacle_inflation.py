"""Optimized inflation must preserve every collision voxel of the scalar rule."""

import numpy as np
import pytest

from python_planner import route_planning as routes


def scalar_inflation(occupied, radius, resolution):
    cells = int(np.ceil(radius / resolution))
    return {(x + dx, y + dy, z + dz)
            for x, y, z in occupied
            for dx in range(-cells, cells + 1)
            for dy in range(-cells, cells + 1)
            for dz in range(-cells, cells + 1)
            if dx * dx + dy * dy + dz * dz <= cells * cells}


@pytest.mark.parametrize('radius', [0., .19, .2, .21, .9])
@pytest.mark.parametrize('seed', [0, 12, 41])
def test_dense_inflation_exactly_matches_scalar_voxels(radius, seed):
    rng = np.random.default_rng(seed)
    occupied = set(map(tuple, rng.integers(-8, 9, size=(40, 3))))
    assert routes.inflate_occupied(occupied, radius, .2) == scalar_inflation(occupied, radius, .2)


def test_boundary_padding_includes_sphere_extremes_and_excludes_diagonals():
    inflated = routes.inflate_occupied({(-5, 10, -100)}, .9, .2)
    assert len(inflated) == 515
    assert (-10, 10, -100) in inflated
    assert (-5, 15, -100) in inflated
    assert (-5, 10, -105) in inflated
    assert (0, 15, -100) not in inflated


def test_sparse_fallback_matches_dense_result(monkeypatch):
    occupied = {(-3, 1, 4), (4, -5, 0)}
    expected = routes.inflate_occupied(occupied, .9, .2)
    monkeypatch.setattr(routes, '_MAX_INFLATION_GRID_CELLS', 1)
    assert routes.inflate_occupied(occupied, .9, .2) == expected


def test_distant_sparse_voxels_do_not_request_a_large_dense_grid(monkeypatch):
    occupied = {(0, 0, 0), (1_000_000, -1_000_000, 500_000)}
    def unexpected_dense_grid(*args, **kwargs):
        raise AssertionError('Sparse map must not allocate a dense grid')
    monkeypatch.setattr(routes.np, 'zeros', unexpected_dense_grid)
    assert routes.inflate_occupied(occupied, .2, .2) == scalar_inflation(occupied, .2, .2)


def test_empty_map_inflates_to_empty_set():
    assert routes.inflate_occupied(set(), .9, .2) == set()


@pytest.mark.parametrize('radius,resolution', [(-1., .2), (.9, 0.), (float('nan'), .2),
                                             (.9, float('inf'))])
def test_invalid_inflation_settings_are_rejected(radius, resolution):
    with pytest.raises(ValueError):
        routes.inflate_occupied({(0, 0, 0)}, radius, resolution)
