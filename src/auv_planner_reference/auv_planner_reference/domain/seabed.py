"""Automatic horizontal floor detection with incremental map-voxel statistics."""

from collections import Counter, defaultdict
import numpy as np


class SeabedEstimate:
    """Find the lowest well-supported, broad horizontal layer in the map.

    Cache height-band sums/counts and XY support. Only changed map voxels update
    the statistics; unchanged maps return immediately. Repeated observations
    cannot inflate the count, and voxels becoming free are removed. No absolute
    world-Z cutoff or fallback floor is used. Without enough evidence, mean_z
    stays None and gain is not penalized.
    """

    def __init__(self, resolution=0.2):
        if not np.isfinite(resolution) or resolution <= 0:
            raise ValueError('seabed map resolution must be finite and positive')
        self.resolution = float(resolution)
        self.mean_z = None
        self.sample_count = 0
        self._voxels = {}
        self._counts = Counter()
        self._sums = defaultdict(float)
        self._xy = defaultdict(Counter)

    def _adjust(self, key, z, sign):
        band = key[2]
        self._counts[band] += sign
        self._sums[band] += sign * z
        xy = key[:2]
        self._xy[band][xy] += sign
        if self._xy[band][xy] <= 0:
            del self._xy[band][xy]
        if self._counts[band] == 0:
            del self._counts[band]
            del self._sums[band]
            del self._xy[band]

    def update(self, points):
        """Update from the current occupied-map snapshot; return changed voxel count."""
        points = np.asarray(points, dtype=float).reshape(-1, 3)
        points = points[np.isfinite(points).all(axis=1)]
        keys = np.floor(points / self.resolution).astype(np.int64)
        current = dict(zip(map(tuple, keys.tolist()), points[:, 2].tolist()))
        changed = 0
        for key, z in self._voxels.items():
            if current.get(key) != z:
                self._adjust(key, z, -1)
                changed += 1
        for key, z in current.items():
            if self._voxels.get(key) != z:
                self._adjust(key, z, 1)
                changed += 1
        self._voxels = current
        if changed == 0:
            return 0

        self.mean_z, self.sample_count = None, 0
        minimum_support = max(24, int(np.ceil(len(current) * 0.03)))
        # Height-density peaks reject isolated low noise. Adjacent layers cover
        # voxel quantization and small surface roughness around a flat floor.
        for band in sorted(self._counts):
            count = self._counts[band]
            if count < self._counts.get(band - 1, 0) or count < self._counts.get(band + 1, 0):
                continue
            neighbors = [b for b in (band - 1, band, band + 1) if b in self._counts]
            support = sum(self._counts[b] for b in neighbors)
            if support < minimum_support:
                continue
            xy = set().union(*(self._xy[b] for b in neighbors))
            positions = np.asarray(list(xy), dtype=np.int64)
            spans = positions.max(axis=0) - positions.min(axis=0) + 1
            # A floor must span both horizontal axes and occupy an area, rather
            # than a narrow pipe or the line made by slicing a vertical wall.
            if np.any(spans * self.resolution < 1.0):
                continue
            coverage = len(xy) / (int(spans[0]) * int(spans[1]))
            if coverage < 0.15:
                continue
            self.mean_z = sum(self._sums[b] for b in neighbors) / support
            self.sample_count = support
            break
        return changed
