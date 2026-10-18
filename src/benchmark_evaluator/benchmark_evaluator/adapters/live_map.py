"""Read stable planner snapshots and identify only changed display chunks."""

from pathlib import Path
import tempfile
import time

import numpy as np


def signature(path):
    stat = Path(path).stat()
    return stat.st_ino, stat.st_size, stat.st_mtime_ns


def validate_snapshot(data):
    """Reject partially written full OctoMaps before calling native bindings.

    Both planners use tree.write(), even though their output is named .bt.
    Full ColorOcTree records contain a float, RGB bytes, and a child mask.
    """
    header, separator, payload = data.partition(b'\ndata\n')
    if not separator or not header.startswith(b'# Octomap OcTree file\n'):
        raise ValueError('Expected a full OctoMap written by the planner')
    fields = dict(line.split(maxsplit=1) for line in header.splitlines()
                  if line and not line.startswith(b'#'))
    record_size = {b'ColorOcTree': 8, b'OcTree': 5}.get(fields.get(b'id'))
    count = int(fields.get(b'size', b'-1'))
    resolution = float(fields.get(b'res', b'0'))
    if (record_size is None or count < 0 or not np.isfinite(resolution)
            or resolution <= 0 or len(payload) != count * record_size):
        raise ValueError('Incomplete or unsupported OctoMap snapshot')
    # Check topology as well as length: never pass a truncated tree to C++.
    pending = 1 if count else 0
    for mask in payload[record_size - 1::record_size]:
        if pending == 0:
            raise ValueError('Invalid OctoMap child structure')
        pending += int(mask).bit_count() - 1
    if pending:
        raise ValueError('Incomplete OctoMap child structure')


def read_snapshot(path, settle_s=0.2):
    """Read occupied leaf centers only; never expand free-space voxels."""
    path = Path(path)
    before = signature(path)
    if time.time_ns() - before[2] < settle_s * 1e9:
        return None
    data = path.read_bytes()
    if signature(path) != before:
        return None
    validate_snapshot(data)
    import pyoctomap

    # Decode an immutable copy so an in-place planner write cannot race C++.
    with tempfile.NamedTemporaryFile(suffix='.ot') as snapshot:
        snapshot.write(data)
        snapshot.flush()
        tree = pyoctomap.ColorOcTree(0.1).read(snapshot.name)
    if tree is None:
        raise ValueError('Could not decode OctoMap')
    rows = []
    for leaf in tree.begin_leafs():
        if tree.isNodeOccupied(leaf):
            color = leaf.getColor() if hasattr(leaf, 'getColor') else (190, 200, 210)
            rows.append((*leaf.getCoordinate(), *color))
    return before, np.asarray(rows, dtype=np.float64).reshape(-1, 6)


class ChunkedMap:
    """Keep stable geometry for unchanged regions, including distant regions."""

    def __init__(self, voxel_size=0.2, chunk_size=4.0):
        if not (np.isfinite(voxel_size) and np.isfinite(chunk_size)
                and 0 < voxel_size <= chunk_size):
            raise ValueError('Require 0 < voxel_size_m <= chunk_size_m')
        self.voxel_size = voxel_size
        self.chunk_size = chunk_size
        self.chunks = {}

    def update(self, rows):
        rows = np.asarray(rows, dtype=np.float64).reshape(-1, 6)
        rows = rows[np.isfinite(rows).all(axis=1)]
        if len(rows):
            # Canonical order makes input reordering a no-op. One point per
            # display voxel bounds density without changing the planner map.
            rows = rows[np.lexsort(tuple(rows[:, i] for i in range(5, -1, -1)))]
            cells = np.floor(rows[:, :3] / self.voxel_size).astype(np.int64)
            cells, indices = np.unique(cells, axis=0, return_index=True)
            rows = rows[indices].copy()
            rows[:, :3] = (cells + 0.5) * self.voxel_size
            keys = np.floor(rows[:, :3] / self.chunk_size).astype(np.int64)
            order = np.lexsort((keys[:, 2], keys[:, 1], keys[:, 0]))
            keys, rows = keys[order], rows[order]
            boundaries = np.r_[0, np.flatnonzero(np.any(np.diff(keys, axis=0), axis=1)) + 1,
                               len(rows)]
            current = {tuple(keys[a]): rows[a:b] for a, b in zip(boundaries[:-1], boundaries[1:])}
        else:
            current = {}
        changes = {key: None for key in self.chunks.keys() - current.keys()}
        for key, points in current.items():
            old = self.chunks.get(key)
            if old is None or not np.array_equal(old, points):
                changes[key] = points
            else:
                current[key] = old
        self.chunks = current
        return changes


class SnapshotUpdates:
    """Worker-owned state; unchanged files require only a stat call."""

    def __init__(self, path, voxel_size, chunk_size):
        self.path = Path(path)
        self.last_signature = None
        self.map = ChunkedMap(voxel_size, chunk_size)

    def poll(self):
        try:
            if signature(self.path) == self.last_signature:
                return {}
            snapshot = read_snapshot(self.path)
        except FileNotFoundError:
            return {}
        if snapshot is None:
            return {}
        stamp, rows = snapshot
        changes = self.map.update(rows)
        self.last_signature = stamp
        return changes


def trajectory_points(values):
    """Decode the planner's [x,y,z,yaw] wire format, including queue clears."""
    values = np.asarray(values, dtype=np.float64)
    if values.ndim != 1 or len(values) % 4 or not np.isfinite(values).all():
        raise ValueError('Trajectory must contain finite [x,y,z,yaw] rows')
    return values.reshape(-1, 4)[:, :3].copy()
