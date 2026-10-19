"""Replay the on-disk Python implementation. Never patch its behavior."""
import json
import pathlib
import resource
import sys
import time
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
from scipy.spatial import KDTree

ROOT = pathlib.Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT / p) for p in ('python_planner', 'auv_contracts', 'telemetry_adapters')]
from python_planner import route_planning as route
from python_planner.use_cases import nbv as math
from python_planner.use_cases import trajectory_optimization as motion
from python_planner.use_cases.mission import InspectionMission, vehicle_viewpoint
from python_planner.domain.configuration import load_configuration
from python_planner.domain.seabed import SeabedEstimate
from python_planner.domain.nbv_parameters import DEFAULT_NBV_PARAMETERS
from python_planner.adapters.point_cloud import Open3DPreprocessor, CloudSnapshot
from python_planner.adapters.optimization import ScipySolver
from python_planner.adapters.octomap import OctomapAdapter
from auv_contracts.values import PoseSample, Vector3, Quaternion, Stamp, Flag


def array(value, width=3):
    return np.asarray(value, dtype=float).reshape(-1, width)


def clean(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {k: clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(v) for v in value]
    return value


def stamp(value):
    seconds = int(value)
    return Stamp(seconds, round((value - seconds) * 1e9))


def depth_frame(j):
    values = [np.nan if x is None else x for x in j['values']]
    shape = (j['rows'], j['cols'])
    if j.get('channels', 1) != 1:
        shape += (j['channels'],)
    return np.asarray(values, dtype=np.uint16 if j.get('uint16') else np.float64).reshape(shape)


def mission(j):
    events, published, completed, inserts, writes, states, evaluation = [], [], [], [], [], [], []
    now = [0.]
    move = [None]
    config = load_configuration(lambda k, default: j.get('config', {}).get(k, default))
    clock = time if j.get('realtime') else SimpleNamespace(perf_counter=lambda: now[0])
    map_points = [np.empty((0, 3))]
    real_map = OctomapAdapter(config.resolution) if j.get('real_map') else None
    class Map:
        def insertPointCloud(self, points, sensor_origin, max_range):
            inserts.append(dict(points=points.copy(), origin=sensor_origin.copy(), max_range=max_range))
            if real_map is not None:
                real_map.insertPointCloud(points, sensor_origin=sensor_origin, max_range=max_range)
        def extractPointCloud(self):
            return real_map.extractPointCloud() if real_map is not None else (map_points[0],)
        def getNumLeafNodes(self):
            return real_map.getNumLeafNodes() if real_map is not None else len(map_points[0])
        def write(self, path):
            writes.append(path)
            if real_map is not None and j.get('write_map'):
                real_map.write(path)
    class Output:
        def encode(self, trajectory, offset):
            wire = trajectory.copy()
            wire[:, :3] += offset
            wire = wire.astype(np.float32)
            executed = wire.astype(np.float64)
            executed[:, :3] -= offset
            return wire, executed
        def publish(self, values):
            published.append(list(values))
    class Telemetry:
        def record_stage(self, stage, duration_ms, iteration=None, frame_id=None,
                         source_stamp_s=None, details=''):
            events.append(dict(type='stage', stage=stage, duration_ms=duration_ms,
                               iteration=iteration, frame_id=frame_id,
                               source_stamp_s=source_stamp_s, details=details))
            if stage == 'route_search' and move[0] is not None:
                m.position = np.array(move[0], dtype=float)
                move[0] = None
        def record_filter(self, stage, input_count, output_count, details, iteration=None, frame_id=None):
            events.append(clean(dict(type='filter', stage=stage, input_count=input_count,
                                     output_count=output_count, details=details,
                                     iteration=iteration, frame_id=frame_id)))
        def record_depth_pose(self, sequence, depth_time_s, position, roll, pitch, yaw):
            events.append(clean(dict(type='depth_pose', sequence=sequence, depth_time_s=depth_time_s,
                                     position=position.copy(), roll=roll, pitch=pitch, yaw=yaw)))
        def record_iteration(self, iteration, elapsed_ms, vehicle_position, candidates,
                             trajectory, published, route_blocked, selected_targets):
            events.append(clean(dict(type='iteration', iteration=iteration, elapsed_ms=elapsed_ms,
                                     vehicle_position=vehicle_position.copy(), candidates=candidates,
                                     trajectory=trajectory.copy(), published=published,
                                     route_blocked=route_blocked, selected_targets=selected_targets)))
    evaluation_sink = SimpleNamespace(write=evaluation.append, flush=lambda: None, close=lambda: None)
    m = InspectionMission(config, Map(), Telemetry(), evaluation_sink, Mock(), clock,
                          Output(), SimpleNamespace(publish=completed.append),
                          lambda *args, **kwargs: math.calculate_nbv_waypoints(*args, **kwargs,
                              preprocessor=Open3DPreprocessor(), diagnostics=Mock(), clock=clock),
                          lambda *args, **kwargs: motion.optimize_trajectory(*args, **kwargs, solver=ScipySolver()),
                          'replay')
    fields = ('position yaw roll pitch latched_position latched_yaw latched_roll latched_pitch '
              'depth_sequence processed_depth_sequence iteration failed_plan_count finished_mapping '
              'started_mapping force_replan active_trajectory selected_targets visited_targets '
              'blacklisted_targets observation_history _plan_failure_reason _nbv_empty_reason '
              '_active_route_failure_reason').split()
    np.random.seed(j.get('seed', 0))
    for e in j['events']:
        now[0] = e.get('time', now[0])
        result = None
        kind = e['type']
        if kind == 'odom':
            m.odom_callback(PoseSample(Vector3(*e['position']), Quaternion(*e.get('orientation', [0, 0, 0, 1])), stamp(e['stamp'])))
        elif kind == 'mag': m.mag_callback(Vector3(*e['value']))
        elif kind == 'rgb': m.rgb_callback(None)
        elif kind == 'depth': m.depth_callback(depth_frame(e['frame']), stamp(e['stamp']))
        elif kind == 'project': result = m._depth_points(depth_frame(e['frame']))
        elif kind == 'map': map_points[0] = array(e['points'])
        elif kind == 'request': m.request_nbv_callback(Flag(e.get('value', True)))
        elif kind == 'tick': m.timer_callback()
        elif kind == 'cancel': m._cancel_active_trajectory()
        elif kind == 'plan':
            move[0] = e.get('move_during_plan')
            result = m._plan_and_publish(candidate_array(e['candidates']), set(map(tuple, e.get('occupied', []))))
        elif kind == 'publish': result = m._publish_trajectory(array(e['trajectory'], 4), set(map(tuple, e.get('occupied', []))))
        elif kind == 'active_safe': result = m._active_trajectory_is_safe(set(map(tuple, e.get('occupied', []))))
        elif kind == 'blacklist': result = m._blacklist_failed_target()
        elif kind == 'expire': m._expire_target_blacklist()
        elif kind == 'candidates':
            values = candidate_array(e['values'])
            m.nbv_calculator = lambda *a, **kw: values
        else: raise ValueError(kind)
        states.append(clean(dict(result=result, state={k: getattr(m, k, None) for k in fields})))
    return clean(dict(states=states, events=events, published=published, completed=completed,
                      inserts=inserts, writes=writes, evaluation=evaluation))


def candidate_array(values):
    return [dict(cluster_id=c.get('cluster_id', -1), position=np.array(c['position'], dtype=float),
                 target=np.array(c['target'], dtype=float), utility=c.get('utility', 0.),
                 raw_gain=c.get('raw_gain', 0.), raw_cost=c.get('raw_cost', 0.)) for c in values]


def run(j):
    op = j['op']
    r = j.get('resolution', .2)
    occupied = set(map(tuple, j.get('occupied', [])))
    if op == 'defaults':
        params = {}
        load_configuration(lambda k, d: params.setdefault(k, d))
        return params
    if op == 'astar':
        expansions = [0]
        def profile(frame, event, arg):
            if event == 'return' and frame.f_code is route.astar_3d.__code__:
                expansions[0] = frame.f_locals.get('expansions', 0)
        if not j.get('benchmark'): sys.setprofile(profile)
        try:
            path, reason = route.astar_3d(np.array(j['start']), np.array(j['goal']), occupied, r, j.get('max_steps', 50000))
        finally: sys.setprofile(None)
        return dict(path=clean(path), reason=reason, expansions=expansions[0])
    if op == 'inflate': return sorted(route.inflate_occupied(occupied, j['radius'], r))
    if op == 'collision': return route.path_is_collision_free(array(j['path']), occupied, r, j.get('factor', .5))
    if op == 'route':
        cs = candidate_array(j['candidates'])
        path, targets, reason = route.plan_multi_waypoint_route(np.array(j['start']),
            [(c['position'], c['target']) for c in cs], occupied, r, j.get('max_steps', 50000),
            np.array(j.get('velocity', [0., 0., 0.])), j.get('reversal', 2.), [c['utility'] for c in cs])
        return dict(path=path, targets=targets, reason=reason)
    if op == 'smooth': return route.generate_smooth_trajectory(array(j['path']), array(j['targets']), j.get('count', 200), j.get('smoothing', 2.), j.get('spacing'))
    if op == 'resample': return route.resample_trajectory_by_distance(array(j['trajectory'], 4), j['spacing'], j.get('cap'))
    if op == 'reconnect': return route.reconnect_trajectory(array(j['trajectory'], 4), np.array(j['position']), j['yaw'], occupied, r, j.get('factor', .5))
    if op == 'vehicle_viewpoint': return vehicle_viewpoint(np.array(j['camera']), np.array(j['target']), np.array(j.get('offset', [.35, .3875, -.5])))
    if op == 'seabed':
        s = SeabedEstimate(r)
        out = []
        for snapshot in j['snapshots']:
            changed = s.update(array(snapshot))
            out.append(dict(changed=changed, mean_z=s.mean_z, sample_count=s.sample_count))
        return out
    if op == 'pca': return math.calculate_pca_complexity(CloudSnapshot(array(j['points']), None), j.get('k', 20))
    if op == 'frontiers': return math.detect_frontiers(CloudSnapshot(array(j['points']), array(j['normals'])), j.get('k', 40), j.get('angle', 130.), j.get('radius', .5), j.get('min_neighbors', 3))
    if op == 'clusters': return math.cluster_frontiers(array(j['points']), array(j['normals']), j['indices'], j.get('radius', 1.), j.get('angle', 45.), j.get('minimum', 5))
    if op == 'axis': return math.select_upward_view_axis(np.array(j['target']), np.array(j['position']), KDTree(array(j['points'])), j.get('standoff', 1.5), j.get('steps', 36))
    if op == 'cone': return array([c['position'] for c in math.generate_cone_viewpoints(np.array(j['target'], dtype=float), np.array(j['normal'], dtype=float), j.get('standoff', 1.5), j.get('polar', [15., 30.]), j.get('steps', 8))])
    if op == 'prepare':
        cloud = Open3DPreprocessor().prepare(array(j['points']), np.array(j['position']), {**DEFAULT_NBV_PARAMETERS, **j.get('parameters', {})}, lambda stage, f: f())
        return None if cloud is None else dict(points=cloud.points, normals=cloud.normals)
    if op == 'nbv': return math.calculate_nbv_waypoints(array(j['points']), np.array(j['position']), np.array(j['heading']),
        [(np.array(p), y) for p, y in j.get('history', [])], j.get('parameters'),
        preprocessor=Open3DPreprocessor(), diagnostics=Mock(), clock=time)
    if op == 'velocity': return motion.estimate_velocity([(t, np.array(p)) for t, p in j['history']], j.get('window', 1.))
    if op == 'continuity': return motion.continuity_cost(array(j['trajectory'], 4), np.array(j['velocity']), array(j.get('history', [])), **j.get('weights', {}))
    if op == 'optimize': return motion.optimize_trajectory(array(j['trajectory'], 4), np.array(j['position']), j['yaw'], np.array(j['velocity']), occupied, r,
        array(j.get('history', [])), **j.get('weights', {}), max_deviation=j.get('deviation', .5), max_iterations=j.get('iterations', 40), collision_spacing_factor=j.get('factor', .5), solver=ScipySolver())
    if op == 'octomap':
        import pyoctomap
        tree = pyoctomap.ColorOcTree(j['load']) if 'load' in j else pyoctomap.ColorOcTree(r)
        out = []
        for scan in j.get('scans', []):
            tree.insertPointCloud(array(scan['points']), sensor_origin=np.array(scan['origin'], dtype=float), max_range=scan.get('max_range', -1.))
            out.append(dict(points=tree.extractPointCloud()[0], leaves=tree.getNumLeafNodes()))
        if 'load' in j: out.append(dict(points=tree.extractPointCloud()[0], leaves=tree.getNumLeafNodes()))
        return out
    if op == 'mission': return mission(j)
    raise ValueError(op)


if __name__ == '__main__':
    for line in sys.stdin:
        try:
            j = json.loads(line)
            times = []
            for _ in range(j.get('repeat', 1)):
                started = time.perf_counter()
                result = run(j)
                times.append((time.perf_counter() - started) * 1000)
            status = dict(line.split(':', 1) for line in pathlib.Path('/proc/self/status').read_text().splitlines())
            print(json.dumps(dict(result=clean(result), latency_ms=times,
                                  max_rss_kib=int(status['VmHWM'].split()[0]),
                                  current_rss_kib=int(status['VmRSS'].split()[0]))), flush=True)
        except Exception as exc:
            print(json.dumps(dict(error=str(exc))), flush=True)
