"""Freeze available mission outputs; these are not raw sensor recordings."""
import csv
import hashlib
import json
import pathlib
import shutil

package = pathlib.Path(__file__).resolve().parents[1]
results = package.parents[1] / 'results'
dest = package / 'test/migration/recorded'
dest.mkdir(exist_ok=True)
shutil.copyfile(results / 'final_map_ros2.bt', dest / 'map.bt')
records = [json.loads(line) for line in (results / 'planner_nbv_iterations.jsonl').read_text().splitlines() if line]
selected = [r for r in records if r.get('candidates')][-3:]
fixtures = []
for record in selected:
    candidates = record['candidates']
    fixtures.append(dict(name=f'recorded_candidates_{record["iteration"]}', input=dict(op='mission',
        events=[dict(type='odom', stamp=1., position=record['vehicle_position']),
                dict(type='plan', candidates=candidates)])))
with (results / 'planner_future_trajectories.csv').open() as file:
    rows = list(csv.DictReader(file))
groups = {}
for row in rows:
    groups.setdefault(row['iteration'], []).append([float(row[k]) for k in ('x_m', 'y_m', 'z_m', 'yaw_rad')])
for iteration, route in list(groups.items())[-3:]:
    fixtures.append(dict(name=f'recorded_route_{iteration}', input=dict(op='resample', trajectory=route, spacing=.5, cap=300)))
with (results / 'planner_depth_poses.csv').open() as file:
    poses = list(csv.DictReader(file))
events = [dict(type='odom', stamp=float(p['depth_time_s']), position=[float(p[k]) for k in ('x_m', 'y_m', 'z_m')]) for p in poses[-50:]]
fixtures.append(dict(name='recorded_pose_history', input=dict(op='mission', events=events)))
(dest / 'inputs.json').write_text(json.dumps(fixtures, indent=2) + '\n')
(dest / 'provenance.json').write_text(json.dumps(dict(
    sources={str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in
             [results / 'final_map_ros2.bt', results / 'planner_nbv_iterations.jsonl',
              results / 'planner_future_trajectories.csv', results / 'planner_depth_poses.csv']},
    limitation='Repository contains map and planner output logs, but no raw depth frames or complete ROS mission bag. Pose replay reconstructs identity quaternion because the source log has Euler angles only; these fixtures are not a complete mission replay.'), indent=2) + '\n')
