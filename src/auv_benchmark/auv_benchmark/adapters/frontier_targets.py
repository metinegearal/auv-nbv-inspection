"""Bounded, incremental observation of the planner's active frontier target."""

import json
import os
from pathlib import Path

import numpy as np


class FrontierTargets:
    """Read existing diagnostics, without running frontier detection or NBV math.

    On attachment, search backwards for the latest lifecycle event, one bounded
    block per poll. Once caught up, read appended bytes only. Keep incomplete
    lines for the next poll and detect rotation/truncation/in-place rewrites.
    """

    def __init__(self, path, read_bytes=131072):
        self.path = Path(path).expanduser()
        self.read_bytes = read_bytes
        self.identity = None
        self.offset = 0
        self.anchor = b''
        self.pending = b''
        self.back_position = 0
        self.back_suffix = b''
        self.searching = False
        self.back_first = False
        self.points = np.empty((0, 3))

    @staticmethod
    def event_target(line):
        try:
            event = json.loads(line)
            if not isinstance(event, dict):
                return None
            stage = event.get('stage')
            if stage in ('trajectory_cancelled', 'inspection_target_observed', 'mission_completion'):
                return np.empty((0, 3))
            if stage != 'inspection_attempt_started':
                return None
            point = np.asarray(event['details']['target'], dtype=float)
            if point.shape == (3,) and np.isfinite(point).all():
                return point.reshape(1, 3)
        except (ValueError, KeyError, TypeError, OverflowError):
            pass
        return None

    def _latest(self, lines):
        for line in reversed(lines):
            points = self.event_target(line)
            if points is not None:
                self.points = points
                self.searching = False
                self.back_suffix = b''
                return True
        return False

    def poll(self):
        try:
            stream = self.path.open('rb')
        except FileNotFoundError:
            self.identity = None
            self.points = np.empty((0, 3))
            return self.points
        with stream:
            stat = os.fstat(stream.fileno())
            identity = stat.st_dev, stat.st_ino
            stream.seek(max(0, self.offset - len(self.anchor)))
            anchor = stream.read(len(self.anchor))
            reset = (identity != self.identity or stat.st_size < self.offset or anchor != self.anchor)
            if reset:
                self.points = np.empty((0, 3))
                self.identity = identity
                # Start with a backwards search, rather than replaying a huge
                # history or treating a partial final record as an active goal.
                self.offset = self.back_position = stat.st_size
                self.anchor = self.pending = self.back_suffix = b''
                self.searching = True
                self.back_first = True
            stream.seek(self.offset)
            appended = stream.read(0 if reset else self.read_bytes)
            self.offset = stream.tell()
            if appended:
                lines = (self.pending + appended).split(b'\n')
                self.pending = lines.pop()
                self._latest(lines)
            stream.seek(max(0, self.offset - 256))
            self.anchor = stream.read(min(256, self.offset))
            if self.searching:
                end = self.back_position
                start = max(0, end - self.read_bytes)
                stream.seek(start)
                data = stream.read(end - start) + self.back_suffix
                lines = data.split(b'\n')
                if self.back_first:
                    # The trailing record may still be in flight. Preserve it
                    # for forward appends; only newline-terminated rows count.
                    trailing = lines.pop() if lines else b''
                    self.pending = trailing + self.pending
                    self.back_first = False
                self.back_suffix = lines.pop(0) if start and lines else b''
                self.back_position = start
                self._latest(lines)
                if not start:
                    self.searching = False
                    self.back_suffix = b''
            # Bound memory even if a malformed diagnostic omits its newline.
            if len(self.pending) > self.read_bytes:
                self.pending = b''
            if len(self.back_suffix) > self.read_bytes:
                self.back_suffix = b''
        return self.points
