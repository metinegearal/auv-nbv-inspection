"""Record mission-long planner exclusions as map-coordinate spheres."""

import argparse
import json
import math
import os
from pathlib import Path
import tempfile

BAN_SPHERES_FILENAME = 'permanent_ban_spheres.json'


def _center(value):
    if not isinstance(value, (list, tuple)):
        raise ValueError('Ban sphere center must be a coordinate array')
    result = [float(v) for v in value]
    if len(result) != 3 or not all(math.isfinite(v) for v in result):
        raise ValueError('Ban sphere center must contain three finite coordinates')
    return result


def target_is_banned(target, spheres):
    return any(math.dist(target, sphere['center']) <= sphere['radius_m'] + 1e-9
               for sphere in spheres)


def load_ban_spheres(path):
    """Load a benchmark snapshot; a missing snapshot means no saved spheres."""
    path = Path(path)
    if not path.exists():
        return []
    data = json.loads(path.read_text())
    if not isinstance(data, dict) or data.get('version') != 1 or data.get('coordinate_frame') != 'map':
        raise ValueError(f'Unsupported ban sphere snapshot: {path}')
    spheres = []
    for value in data['spheres']:
        sphere = dict(value, center=_center(value['center']), radius_m=float(value['radius_m']))
        if not math.isfinite(sphere['radius_m']) or sphere['radius_m'] <= 0:
            raise ValueError('Ban sphere radius must be finite and positive')
        spheres.append(sphere)
    return spheres


class PermanentBanRecorder:
    """Incrementally consume complete planner-filter records and save snapshots."""

    def __init__(self, results_dir, source=None):
        self.directory = Path(results_dir).expanduser()
        self.source = Path(source).expanduser() if source else self.directory / 'planner_nbv_filters.jsonl'
        self.output = self.directory / BAN_SPHERES_FILENAME
        self._identity = None
        self._offset = 0
        self._pending = b''
        self._anchor = b''
        self._spheres = {}
        self._attempt_centers = []

    @property
    def spheres(self):
        return list(self._spheres.values())

    def _consume(self, event):
        if not isinstance(event, dict):
            return False
        stage, details = event.get('stage'), event.get('details', {})
        if not isinstance(details, dict):
            return False
        if stage in ('inspection_attempt_started', 'inspection_plan_failed'):
            center = _center(details.get('region_center', details['target']))
            radius = float(details.get('matching_radius_m', .5))
            if not math.isfinite(radius) or radius <= 0:
                raise ValueError('Retry matching radius must be finite and positive')
            if not any(math.dist(center, old['center']) < old['radius_m'] for old in self._attempt_centers):
                self._attempt_centers.append(dict(center=center, radius_m=radius))
            return False
        if stage == 'blacklist_addition' and details.get('expires') is False:
            center = _center(details['target'])
            radius = float(details.get('radius', .5))
            reason = details.get('reason', 'permanent_blacklist')
        elif stage == 'inspection_target_suppressed' and details.get('reason') == 'attempt_budget_exhausted':
            target = _center(details['target'])
            # Match the planner's first spatial attempt record, rather than
            # moving the saved sphere with jittering candidate centroids.
            if 'exclusion_center' in details:
                center = _center(details['exclusion_center'])
                radius = float(details['radius'])
            else:
                region = next((c for c in self._attempt_centers
                               if math.dist(c['center'], target) < c['radius_m']), None)
                if region is None:
                    return False  # An incomplete log cannot establish the original center.
                center, radius = region['center'], region['radius_m']
            reason = 'attempt_budget_exhausted'
        else:
            return False
        if not math.isfinite(radius) or radius <= 0:
            raise ValueError('Ban sphere radius must be finite and positive')
        key = tuple(center)
        previous = self._spheres.get(key)
        if previous is not None and previous['radius_m'] >= radius:
            return False
        self._spheres[key] = dict(center=center, radius_m=radius, reason=reason,
                                 iteration=event.get('iteration'), frame_id=event.get('frame_id'))
        return True

    def save_snapshot(self, path=None):
        path = Path(path) if path else self.output
        path.parent.mkdir(parents=True, exist_ok=True)
        data = dict(version=1, coordinate_frame='map', scope='mission',
                    source_log=str(self.source.resolve()), spheres=self.spheres)
        # Readers always see a complete JSON snapshot, including while ROS runs.
        temporary = None
        try:
            with tempfile.NamedTemporaryFile('w', dir=path.parent, prefix=f'.{path.name}.',
                                              delete=False) as stream:
                temporary = Path(stream.name)
                json.dump(data, stream, indent=2, allow_nan=False)
                stream.write('\n')
            temporary.replace(path)
        finally:
            if temporary is not None:
                temporary.unlink(missing_ok=True)
        return path

    def poll(self):
        try:
            stream = self.source.open('rb')
        except FileNotFoundError:
            return False
        with stream:
            stat = os.fstat(stream.fileno())
            identity = (stat.st_dev, stat.st_ino)
            stream.seek(max(0, self._offset - len(self._anchor)))
            anchor = stream.read(len(self._anchor))
            reset = (identity != self._identity or stat.st_size < self._offset or anchor != self._anchor)
            if reset:
                self._offset = 0
                self._pending = b''
                self._spheres.clear()
                self._attempt_centers.clear()
            self._identity = identity
            stream.seek(self._offset)
            chunk = stream.read()
            self._offset = stream.tell()
            stream.seek(max(0, self._offset - 256))
            self._anchor = stream.read(min(256, self._offset))
        lines = (self._pending + chunk).split(b'\n')
        self._pending = lines.pop()
        changed = reset
        for line in lines:
            try:
                changed = self._consume(json.loads(line)) or changed
            except (ValueError, KeyError, TypeError, OverflowError):
                continue  # One malformed diagnostic must not stop benchmark recording.
        if changed:
            self.save_snapshot()
        return changed


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--results-dir', type=Path, default=Path('results'))
    parser.add_argument('--filters', type=Path, help='Override the planner filter JSONL source')
    args = parser.parse_args()
    recorder = PermanentBanRecorder(args.results_dir, args.filters)
    if not recorder.source.exists():
        parser.error(f'Planner filter log does not exist: {recorder.source}')
    recorder.poll()
    print(f'Saved {len(recorder.spheres)} permanent ban spheres to {recorder.output}')


if __name__ == '__main__':
    main()
