"""Read-only bag analysis; outputs are written into controller_bag_report."""
from pathlib import Path
import csv, json
import numpy as np
import rosbag
from scipy.spatial import cKDTree
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parent
OUT = ROOT / 'controller_bag_report'
OUT.mkdir(exist_ok=True)
results = []
fig, axes = plt.subplots(2, 2, figsize=(12, 10))
for name, ax in zip(['stanley', 'MPC', 'model', 'PP'], axes.flat):
    poses, odoms, latency, collisions, logs, controls = [], [], [], [], [], []
    route = None
    with rosbag.Bag(str(ROOT / (name + '.bag'))) as bag:
        info = bag.get_type_and_topic_info().topics
        duration = bag.get_end_time() - bag.get_start_time()
        for topic, msg, stamp in bag.read_messages(topics=[
            '/global_path', '/morai/ego_vehicle_status', '/ctrl_cmd', '/localization/kinematic_state',
            '/multimodal_learning/inference_latency_ms', '/morai/collision_data', '/rosout']):
            t = stamp.to_sec()
            if topic == '/global_path' and route is None:
                route = np.array([(p.pose.position.x, p.pose.position.y) for p in msg.poses])
            elif topic == '/morai/ego_vehicle_status':
                poses.append((t, msg.position.x, msg.position.y, msg.velocity.x, msg.ctrl_mode))
            elif topic == '/localization/kinematic_state':
                odoms.append((t,msg.pose.pose.position.x,msg.pose.pose.position.y,msg.twist.twist.linear.x*3.6,-1))
            elif topic == '/ctrl_cmd':
                controls.append((t, msg.accel, msg.brake, msg.steer, msg.ctrl_mode))
            elif topic.endswith('inference_latency_ms'):
                latency.append(float(msg.data))
            elif topic == '/morai/collision_data':
                active = [(o.objType, o.obj_id) for o in msg.collision_object
                          if (o.objType, o.obj_id) not in [(0, 0), (-1, 0)]]
                if active: collisions.append((t, active))
            elif topic == '/rosout' and any(k in msg.msg for k in ['checkpoint loaded', 'control:', 'stale', 'safe stop', 'failed', 'latched']):
                logs.append((t, msg.name, msg.msg))
    pose_source = 'ego status' if poses else 'localization (GT absent)'
    p = np.asarray(poses if poses else odoms)
    route_source = 'bag /global_path'
    if route is None:
        reference = np.genfromtxt(str(ROOT/'global_path/global_path.csv'),delimiter=',',names=True)
        route = np.column_stack([reference['x'],reference['y']])
        route_source = 'workspace global_path.csv (bag route missing; provisional)'
    if len(route) < 2 or not len(p):
        raise RuntimeError('Missing reference route or ego pose: ' + name)
    delta = np.diff(route, axis=0)
    length = np.linalg.norm(delta, axis=1)
    s = np.r_[0., np.cumsum(length)]
    tree = cKDTree(route)
    _, idx = tree.query(p[:, 1:3])
    # Project onto both adjacent segments, rather than point spacing error.
    seg = np.stack([np.maximum(idx-1,0), np.minimum(idx,len(route)-2)], axis=1)
    vectors = p[:,None,1:3] - route[seg]
    fraction = np.clip(np.sum(vectors*delta[seg],axis=2)/np.maximum(length[seg]**2,1e-12),0,1)
    projection = route[seg] + fraction[:,:,None]*delta[seg]
    distances = np.linalg.norm(p[:,None,1:3]-projection,axis=2)
    best = distances.argmin(axis=1)
    rows = np.arange(len(p))
    cte = distances[rows,best]
    progress = s[seg[rows,best]] + fraction[rows,best]*length[seg[rows,best]]
    jumps = np.linalg.norm(np.diff(p[:,1:3],axis=0),axis=1)
    dt = np.diff(p[:,0])
    teleports = int(np.sum(jumps > np.maximum(10., 60.*dt)))
    bursts = sum(i == 0 or t-collisions[i-1][0] > 1.0 for i,(t,_) in enumerate(collisions))
    r = dict(controller=name, pose_source=pose_source, route_source=route_source, duration_s=duration, route_length_m=s[-1],
             start_progress_m=progress[0], furthest_progress_m=float(progress.max()),
             route_completion_nearest_projection_pct=float(100*progress.max()/s[-1]),
             start_xy=p[0,1:3].tolist(), end_xy=p[-1,1:3].tolist(),
             mean_cte_m=float(cte.mean()), max_cte_m=float(cte.max()),
             p95_cte_m=float(np.percentile(cte,95)),
             mean_abs_speed_kph=float(np.abs(p[:,3]).mean()), max_abs_speed_kph=float(np.abs(p[:,3]).max()),
             collision_nonzero_messages=len(collisions), collision_bursts_1s=bursts,
             collision_ids=sorted(set(str(o) for _,obs in collisions for o in obs)),
             teleport_candidates=teleports, manual_mode_samples=int(np.sum(p[:,4] != 2)) if poses else None,
             mean_inference_ms=float(np.mean(latency)) if latency else None,
             p95_inference_ms=float(np.percentile(latency,95)) if latency else None,
             missing_event_topics=[k for k in ['object','traffic','scenario','recovery']
                                   if not any(k in t.lower() for t in info)])
    results.append(r)
    with (OUT/(name+'_trajectory.csv')).open('w') as f:
        w=csv.writer(f);w.writerow(['timestamp','ego_x','ego_y','speed_kph','ctrl_mode','route_progress_m','cte_m'])
        w.writerows(np.column_stack([p, progress, cte]))
    with (OUT/(name+'_control.csv')).open('w') as f:
        w=csv.writer(f);w.writerow(['timestamp','accel','brake','steer','ctrl_mode']);w.writerows(controls)
    (OUT/(name+'_evidence.json')).write_text(json.dumps(dict(topics=list(info),collisions=collisions,logs=logs),ensure_ascii=False,indent=2))
    ax.plot(route[:,0],route[:,1],color='gray',lw=1,label='Reference route')
    ax.plot(p[:,1],p[:,2],lw=1,label=name)
    ax.scatter(*p[0,1:3],color='green',s=25);ax.scatter(*p[-1,1:3],color='red',s=25)
    ax.set_aspect('equal');ax.legend();ax.set_title(name);ax.set_xlabel('x [m]');ax.set_ylabel('y [m]')
    print(json.dumps(r),flush=True)
(OUT/'metrics.json').write_text(json.dumps(results,indent=2))
with (OUT/'metrics.csv').open('w') as f:
    w=csv.DictWriter(f,fieldnames=list(results[0]));w.writeheader();w.writerows(results)
fig.tight_layout();fig.savefig(str(OUT/'trajectories.png'),dpi=180)
