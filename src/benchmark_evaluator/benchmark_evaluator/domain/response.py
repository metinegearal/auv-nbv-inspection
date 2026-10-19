"""Measured position-step response, independent of the controller model."""
from collections import deque
import math

import numpy as np


class ResponseMonitor:
    """Track a fixed target; settling is provisional over observed samples."""

    def __init__(self, start, target, *, band_fraction=0.02, dwell_s=1.0,
                 max_samples=3000):
        self.start = self._position(start)
        self.target = self._position(target)
        if not math.isfinite(band_fraction) or not 0 < band_fraction < 1:
            raise ValueError('band_fraction must be between zero and one')
        if not math.isfinite(dwell_s) or dwell_s < 0:
            raise ValueError('dwell_s must be finite and nonnegative')
        if max_samples < 2:
            raise ValueError('max_samples must be at least two')
        self.delta = self.target - self.start
        self.initial_distance = float(np.linalg.norm(self.delta))
        self.valid = self.initial_distance > 1e-6
        self.band = band_fraction * self.initial_distance
        self.dwell_s = dwell_s
        self.samples = deque(maxlen=max_samples)
        self.sample_count = 0
        self.crossings = {0.1: None, 1 - math.exp(-1): None, 0.9: None}
        self.settling_since = None
        self.peak_progress, self.peak_time = 1.0, None
        self.iae = self.ise = 0.0
        self.update(0.0, self.start)

    @staticmethod
    def _position(position):
        value = np.asarray(position, dtype=float)
        if value.shape != (3,) or not np.isfinite(value).all():
            raise ValueError('position must contain three finite values')
        return value.copy()

    def update(self, elapsed, position):
        position = self._position(position)
        if not math.isfinite(elapsed) or elapsed < 0:
            raise ValueError('elapsed must be finite and nonnegative')
        error = self.target - position
        distance = float(np.linalg.norm(error))
        progress = (float(1 - error @ self.delta / self.initial_distance**2)
                    if self.valid else 1.0)
        if self.samples:
            previous_time, previous_distance, previous_progress, *_ = self.samples[-1]
            dt = elapsed - previous_time
            if dt < 0:
                raise ValueError('response time must be monotonic')
            self.iae += dt * (previous_distance + distance) / 2
            self.ise += dt * (previous_distance**2 + distance**2) / 2
            if self.valid:
                for threshold in self.crossings:
                    if (self.crossings[threshold] is None
                            and previous_progress < threshold <= progress):
                        fraction = (threshold - previous_progress) / (progress - previous_progress)
                        self.crossings[threshold] = previous_time + fraction * dt
        if self.valid and distance <= self.band:
            if self.settling_since is None:
                self.settling_since = float(elapsed)
        else:
            self.settling_since = None
        if self.valid and progress > self.peak_progress:
            self.peak_progress, self.peak_time = progress, float(elapsed)
        self.sample_count += 1
        self.samples.append((float(elapsed), distance, progress, *map(float, error)))

    def metrics(self):
        t10, t90 = self.crossings[0.1], self.crossings[0.9]
        in_band = (0.0 if self.settling_since is None
                   else self.samples[-1][0] - self.settling_since)
        return {
            'valid_step': self.valid,
            'initial_distance_m': self.initial_distance,
            'band_m': self.band,
            'duration_s': self.samples[-1][0],
            'rise_time_s': None if t10 is None or t90 is None else t90 - t10,
            't63_s': self.crossings[1 - math.exp(-1)],
            'settling_time_s': self.settling_since,
            'in_band_for_s': in_band,
            'settling_dwell_met': self.settling_since is not None and in_band >= self.dwell_s,
            'overshoot_pct': 100 * (self.peak_progress - 1) if self.valid else None,
            'peak_time_s': self.peak_time,
            'final_error_m': self.samples[-1][1],
            'iae_m_s': self.iae,
            'ise_m2_s': self.ise,
            'sample_count': self.sample_count,
            'retained_samples': len(self.samples),
        }

    def summary(self):
        if not self.valid:
            return 'No position step; response times N/A'
        values = self.metrics()

        def seconds(value):
            return 'pending' if value is None else f'{value:.2f}s'

        return (
            f'tr(10-90%)={seconds(values["rise_time_s"])}; '
            f't63={seconds(values["t63_s"])}; '
            f'ts(provisional)={seconds(values["settling_time_s"])} '
            f'[in band {values["in_band_for_s"]:.1f}s]; '
            f'overshoot={values["overshoot_pct"]:.2f}%; '
            f'tp={seconds(values["peak_time_s"])}; '
            f'error={values["final_error_m"]:.3f}m; '
            f'IAE={values["iae_m_s"]:.3f}m*s; ISE={values["ise_m2_s"]:.3f}m^2*s')
