"""Observe actual controller targets and every received odometry sample."""
import math

from ..domain.response import ResponseMonitor


class MotionResponseEvaluation:
    """Use monotonic callback receipt time for unstamped waypoint commands."""

    def __init__(self, clock, diagnostics, *, band_fraction=0.02, dwell_s=1.0,
                 target_tolerance_m=1e-5, max_samples=3000, log_period_s=1.0):
        if not math.isfinite(target_tolerance_m) or target_tolerance_m < 0:
            raise ValueError('target_tolerance_m must be finite and nonnegative')
        if not math.isfinite(log_period_s) or log_period_s <= 0:
            raise ValueError('log_period_s must be finite and positive')
        self.options = {'band_fraction': band_fraction, 'dwell_s': dwell_s,
                        'max_samples': max_samples}
        # Validate options even when no target has arrived yet.
        ResponseMonitor((0., 0., 0.), (0., 0., 0.), **self.options)
        self.clock, self.diagnostics = clock, diagnostics
        self.tolerance = target_tolerance_m
        self.log_period = log_period_s
        self.position = self.target = self.monitor = None
        self.started_at = self.last_log_at = None
        self.episode_id = 0
        self.records, self.samples = [], []
        self.closed = False

    def waypoint_callback(self, msg):
        if self.closed or len(msg.data) < 3:
            return
        try:
            target = ResponseMonitor._position(msg.data[:3])
        except ValueError:
            self.diagnostics.warning('Ignoring nonfinite motion response target')
            return
        if self.target is not None and math.dist(target, self.target) <= self.tolerance:
            return
        self._finish_episode('target_changed')
        self.target = target
        if self.position is not None:
            self._start_episode()

    def _start_episode(self):
        self.episode_id += 1
        self.monitor = ResponseMonitor(self.position, self.target, **self.options)
        self.started_at = self.clock.perf_counter()
        self.last_log_at = self.started_at
        self.diagnostics.info(f'Motion response #{self.episode_id}: {self.monitor.summary()}')

    def odom_callback(self, msg):
        if self.closed:
            return
        try:
            position = ResponseMonitor._position((msg.position.x, msg.position.y, msg.position.z))
        except ValueError:
            self.diagnostics.warning('Ignoring nonfinite motion response odometry')
            return
        self.position = position
        if self.target is None:
            return
        if self.monitor is None:
            self._start_episode()
            return
        now = self.clock.perf_counter()
        self.monitor.update(now - self.started_at, position)
        if now - self.last_log_at >= self.log_period:
            self.last_log_at = now
            self.diagnostics.info(f'Motion response #{self.episode_id}: {self.monitor.summary()}')

    def _finish_episode(self, reason):
        if self.monitor is None:
            return
        record = dict(episode_id=self.episode_id, end_reason=reason,
                      **self.monitor.metrics())
        for index, axis in enumerate('xyz'):
            record[f'start_{axis}_m'] = float(self.monitor.start[index])
            record[f'target_{axis}_m'] = float(self.monitor.target[index])
        self.records.append(record)
        self.samples.extend((self.episode_id, *sample) for sample in self.monitor.samples)
        self.diagnostics.info(
            f'Motion response #{self.episode_id} ({reason}): {self.monitor.summary()}')
        self.monitor = None

    def close(self, reason='shutdown'):
        if not self.closed:
            self._finish_episode(reason)
            self.closed = True
