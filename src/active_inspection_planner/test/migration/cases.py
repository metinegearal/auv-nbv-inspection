"""Deterministic fixtures shared by differential tests and performance runs."""
import numpy as np


def candidate(position, target=None, utility=1., cluster_id=0):
    return dict(position=list(position), target=list(target if target is not None else np.asarray(position) - [0, 0, 1]), utility=utility, cluster_id=cluster_id)


def cases():
    yield 'defaults', dict(op='defaults')
    rng = np.random.RandomState(482)
    for seed in range(20):
        obstacles = rng.randint(-4, 5, (90, 3)).tolist()
        yield f'astar_obstacles_{seed}', dict(op='astar', start=[-.77, -.72, -.61], goal=[.79, .65, .83], occupied=obstacles, resolution=.2, max_steps=1000)
    for name, occupied, limit in [('start_occupied', [[0, 0, 0]], 100), ('goal_occupied', [[2, 0, 0]], 100),
                                   ('exhaustion', [], 2), ('zero_limit', [], 0),
                                   ('no_connection', [[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]], 100)]:
        yield name, dict(op='astar', start=[.1, .1, .1], goal=[2.1, .1, .1], occupied=occupied, resolution=1., max_steps=limit)
    for radius in [0., .2, .21, .6, 1.]:
        yield f'inflate_{radius}', dict(op='inflate', occupied=[[0, 0, 0], [-2, 3, 1]], radius=radius, resolution=.2)
    yield 'inflate_empty', dict(op='inflate', occupied=[], radius=.6)
    for name, p in [('empty', []), ('point', [[.1, .1, .1]]), ('gap', [[.1, .1, .1], [2.1, .1, .1]]), ('detour', [[.1, .1, .1], [.1, 2.1, .1], [2.1, 2.1, .1]])]:
        yield f'collision_{name}', dict(op='collision', path=p, occupied=[[1, 0, 0]], resolution=1.)
    cs = [candidate([1.1, .1, .1], utility=.2), candidate([3.1, .1, .1], utility=.8), candidate([-.9, .1, .1], utility=.8)]
    for velocity in ([0., 0., 0.], [1., 0., 0.]):
        for obstacles in ([], [[3, 0, 0]], [[0, 0, 0]]):
            yield f'route_{velocity}_{obstacles}', dict(op='route', start=[.1, .1, .1], candidates=cs, velocity=velocity, occupied=obstacles, resolution=1.)
    yield 'route_empty', dict(op='route', start=[0, 0, 0], candidates=[])
    for n in [0, 1, 2, 3, 4, 8, 30]:
        p = np.cumsum(rng.normal(size=(n, 3)), axis=0)
        for smoothing in [0., .1, 2., 50.]:
            yield f'spline_{n}_{smoothing}', dict(op='smooth', path=p.tolist(), targets=(p + [2., -.5, -1.]).tolist(), smoothing=smoothing, count=35, spacing=.2)
    p = [[0., 0., 0.], [1., 0., 0.], [1., 0., 0.], [1., 1., 0.], [2., 1., 0.]]
    yield 'spline_duplicates_yaw_wrap', dict(op='smooth', path=p, targets=[[-1., -.001, 0.], [0., .001, 0.], [0., .001, 0.], [0., 1.001, 0.], [1., .999, 0.]])
    t = [[0., 0., 0., 3.1], [1., 0., 0., -3.1], [1., 0., 0., 2.], [1., 1., 0., -2.8], [2., 1., 0., 2.9]]
    for spacing in [0., .1, .3, 10.]:
        for cap in [2, 8, 100]:
            yield f'resample_{spacing}_{cap}', dict(op='resample', trajectory=t, spacing=spacing, cap=cap)
    for pos in [[.7, .1, 0.], [3., 1., 0.], [1., 1., 0.]]:
        for obstacles in [[], [[0, 0, 0]]]:
            yield f'reconnect_{pos}_{obstacles}', dict(op='reconnect', trajectory=t, position=pos, yaw=3.13, occupied=obstacles)
    yield 'reconnect_empty', dict(op='reconnect', trajectory=[], position=[0, 0, 0], yaw=0.)
    for n in range(8):
        yield f'camera_offset_{n}', dict(op='vehicle_viewpoint', camera=rng.normal(size=3).tolist(), target=rng.normal(size=3).tolist())
    floor = [[x * .2 + .1, y * .2 + .1, -3.91] for x in range(12) for y in range(10)]
    changed = floor.copy(); changed[0] = [floor[0][0], floor[0][1], -3.9]
    yield 'seabed_changes_free_absent', dict(op='seabed', snapshots=[[], floor, floor, changed, floor[10:], [], [[0, 0, -100]]])
    for shape in ['random', 'plane', 'corner', 'line', 'empty']:
        if shape == 'random': p = rng.uniform(-2, 2, (200, 3))
        elif shape == 'plane': p = np.array([[x * .15, y * .15, 0.] for x in range(15) for y in range(15)])
        elif shape == 'corner': p = np.array([[x * .15, y * .15, 0.] for x in range(15) for y in range(15)] + [[x * .15, 0., z * .15] for x in range(15) for z in range(15)])
        elif shape == 'line': p = np.array([[x * .15, 0., 0.] for x in range(30)])
        else: p = np.empty((0, 3))
        normals = np.tile([0., 0., 1.], (len(p), 1))
        yield f'pca_{shape}', dict(op='pca', points=p.tolist())
        yield f'frontiers_{shape}', dict(op='frontiers', points=p.tolist(), normals=normals.tolist())
        yield f'clusters_{shape}', dict(op='clusters', points=p.tolist(), normals=normals.tolist(), indices=list(range(len(p))))
        yield f'preprocessing_{shape}', dict(op='prepare', points=p.tolist(), position=[0, 0, 2.])
        for history in ([], [[[0, 0, 1.], 0.]]):
            yield f'nbv_{shape}_{len(history)}', dict(op='nbv', points=p.tolist(), position=[.3, .2, 2.], heading=[1., 0., 0.], history=history)
    for n in range(3):
        yield f'axis_{n}', dict(op='axis', points=rng.uniform(-2, 2, (80, 3)).tolist(), target=[0., 0., 0.], position=[2., 1., 0.])
    for normal in ([0., 0., 1.], [1., 1., 1.]):
        yield f'cone_{normal}', dict(op='cone', target=[.1, .2, .3], normal=normal)
    for count in [0, 1, 3, 15]:
        history = [[i * .1, [i * .03, i * .1, i * -.02]] for i in range(count)]
        yield f'velocity_{count}', dict(op='velocity', history=history)
    for n in [2, 4, 10]:
        t = np.c_[np.arange(n) * .3, np.sin(np.arange(n)), np.zeros(n), np.linspace(3., 4., n)]
        common = dict(trajectory=t.tolist(), velocity=[.3, .1, 0.], history=[[-1., 0., 0.]], position=t[0, :3].tolist(), yaw=2.9, iterations=6)
        yield f'continuity_{n}', dict(op='continuity', **common)
        yield f'optimizer_{n}', dict(op='optimize', **common)
    yield 'optimizer_unsafe', dict(op='optimize', trajectory=[[0., 0., 0., 0.], [1., 0., 0., 0.]], position=[0., 0., 0.], yaw=0., velocity=[1., 0., 0.], occupied=[[0, 0, 0]])
    for n, iterations in [(4, 40), (10, 40), (40, 3)]:
        t = np.c_[np.arange(n) * .3, .1*np.sin(np.arange(n)), np.zeros(n), np.linspace(3., 4., n)]
        yield f'optimizer_controls_{n}_{iterations}', dict(op='optimize', trajectory=t.tolist(), position=t[0, :3].tolist(), yaw=2.9, velocity=[.3, .1, 0.], iterations=iterations)
    scans = [dict(points=rng.uniform([1., -.5, -.5], [2., .5, .5], (100, 3)).tolist(), origin=[0., 0., 0.], max_range=3.)]
    scans += [dict(points=[[3., 0., 0.]], origin=[0., 0., 0.], max_range=4.) for _ in range(5)]
    yield 'octomap_rays', dict(op='octomap', scans=scans)
    yield from mission_cases()
    yield from recorded_cases()


def recorded_cases():
    import json
    import pathlib
    directory = pathlib.Path(__file__).parent / 'recorded'
    for case in json.loads((directory / 'inputs.json').read_text()):
        yield case['name'], case['input']
    yield 'recorded_map_extraction', dict(op='octomap', load=str(directory / 'map.bt'))
    import pyoctomap
    tree = pyoctomap.ColorOcTree(str(directory / 'map.bt'))
    points = tree.extractPointCloud()[0].tolist()
    yield 'recorded_map_nbvs', dict(op='nbv', points=points,
        position=[362.3865051269531, 808.332763671875, -295.830322265625],
        heading=[1., 0., 0.])


def mission_cases():
    initial = [dict(type='mag', value=[0., 1., 0.]), dict(type='odom', position=[.1, .1, .1], stamp=1.), dict(type='rgb')]
    frame = dict(rows=4, cols=5, values=[.5, 1., 2., 3., None] * 4)
    yield 'depth_timestamps_origin_guard', dict(op='mission', config=dict(depth_stride=1), events=[dict(type='odom', position=[0., 0., 2.], stamp=0.), *initial,
        dict(type='depth', frame=frame, stamp=1.01), dict(type='project', frame=frame),
        dict(type='odom', position=[.5, .3, .2], stamp=1.2, orientation=[.1, .2, .3, .9]),
        dict(type='depth', frame=frame, stamp=9.), dict(type='project', frame=frame),
        dict(type='odom', position=[.7, .3, .2], stamp=1.2), dict(type='odom', position=[.8, .3, .2], stamp=.1)])
    for uint16 in [True, False]:
        f = dict(rows=2, cols=3, uint16=uint16, values=[500, 501, 2000, 3000, 2999, 0] if uint16 else [.5, .501, 2., 3., 2.999, 0])
        yield f'depth_units_{uint16}', dict(op='mission', config=dict(depth_stride=1, depth_scale=.7), events=[*initial, dict(type='depth', stamp=1., frame=f), dict(type='project', frame=f)])
    for goal in [[.11, .11, .1], [.5, .1, .1], [3., .1, .1]]:
        yield f'mission_short_route_{goal}', dict(op='mission', config=dict(voxel_resolution=.2), events=[*initial, dict(type='plan', candidates=[candidate(goal)])])
    yield 'stall_cancel', dict(op='mission', events=[*initial, dict(type='publish', trajectory=[[.1, .1, .1, 0.], [2., .1, .1, 0.]]), dict(type='active_safe', time=0.), dict(type='active_safe', time=11.), dict(type='cancel')])
    yield 'new_obstacle_cancel', dict(op='mission', events=[*initial, dict(type='publish', trajectory=[[.1, .1, .1, 0.], [2., .1, .1, 0.]]), dict(type='active_safe', occupied=[[10, 1, 1]]), dict(type='cancel')])
    for move in [[.2, .2, .1], [20., 20., .1]]:
        yield f'move_during_plan_{move}', dict(op='mission', events=[*initial, dict(type='plan', candidates=[candidate([3., .1, .1])], move_during_plan=move)])
    yield 'blacklist_expiry', dict(op='mission', config=dict(astar_max_steps=1), events=[*initial, dict(type='plan', candidates=[candidate([5., 1., .1])]), dict(type='blacklist', time=2.), dict(type='expire', time=31.), dict(type='expire', time=33.)])
    yield 'completion_three_passes', dict(op='mission', config=dict(planning_period=1), events=[*initial, dict(type='candidates', values=[]), *sum(([dict(type='depth', frame=frame, stamp=1.), dict(type='tick', time=float(i))] for i in range(4)), [])])
    yield 'depth_cap_randomstate', dict(op='mission', config=dict(depth_stride=1, collision_radius=0., collision_tracking_margin=0.), seed=872,
        events=[*initial, dict(type='candidates', values=[]), dict(type='depth', stamp=1., frame=dict(rows=70, cols=70, values=[1.] * 4900)), dict(type='tick')])
    # An obstructed smoothed corner must retain the route's individual A* corners.
    yield 'unsafe_spline_corner_fallback', dict(op='mission', config=dict(voxel_resolution=1., trajectory_points=2, waypoint_spacing=3., spline_smoothing=100.), events=[*initial,
        dict(type='plan', candidates=[candidate([4.5, .5, .5], [4.5, .5, -1.])], occupied=[[2, 0, 0], [2, 1, 0], [2, -1, 0]])])
    yield 'depth_octomap_mission', dict(op='mission', real_map=True, config=dict(
        depth_stride=1, voxel_resolution=.2, collision_radius=.1, collision_tracking_margin=.1),
        events=[*initial, dict(type='depth', stamp=1., frame=dict(rows=30, cols=40, values=[2.] * 1200)), dict(type='tick')])
    yield 'wire_rounding_collision', dict(op='mission', config=dict(voxel_resolution=.1), events=[*initial,
        dict(type='publish', trajectory=[[1.099999999, .21, .21, 0.], [1.099999999, .25, .21, 0.]], occupied=[[11, 2, 2]])])
    yield 'movement_reconnection_rejected', dict(op='mission', config=dict(voxel_resolution=1.), events=[*initial,
        dict(type='plan', candidates=[candidate([4.5, .5, .5], [4.5, .5, -1.])], occupied=[[2, 1, 0]], move_during_plan=[2.5, 1.5, .5])])
