"""Investigate false LiDAR obstacles and disconnected A* space in MuJoCo.

Diagnostic only: controller, EKF, physics, mapping, and safety remain unchanged.
MuJoCo true pose / body orientation is used ONLY for diagnosis, never for control.

Usage (from repository root):
    MUJOCO_GL=osmesa MPLBACKEND=Agg python scripts/diagnose_v11_lidar_tilt.py
"""
from __future__ import annotations

import json
import math
from collections import Counter, deque
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from turtlebot_integrated.config import Config
from turtlebot_integrated.experiment import run_trial
from turtlebot_integrated.mujoco_backend import MuJoCoBackend
from turtlebot_integrated.navigation import Navigator

OUT = Path('outputs/diagnostics/v11_lidar_tilt')
OUT.mkdir(parents=True, exist_ok=True)
CFG = 'configs/mujoco_v10_goal_region_probe.yaml'
# These are MuJoCo *simulation-time* checkpoints, not wall-clock waits.
SNAPSHOT_TIMES = (0.0, 8.0, 19.68, 29.0)
records = []
map_snapshots = []
planner_statuses = Counter()
first_route_failure = None


def floating(value, precision=5):
    return round(float(value), precision)


def inspect_rays(sim, packet):
    """Compare measured ranges to floor-intersection geometry of tilted rays."""
    r_body = np.asarray(sim.data.xmat[sim.body_id]).reshape(3, 3)
    # Rotation matrix: world -> body axes; ZYX Euler roll/pitch for diagnostics.
    roll = math.atan2(r_body[2, 1], r_body[2, 2])
    pitch = math.atan2(-r_body[2, 0], math.hypot(r_body[2, 1], r_body[2, 2]))
    front = np.where(np.abs(np.asarray(packet.scan.angles)) < 0.40)[0]
    ray_infos = []
    for i in front:
        # Rangefinder sensors point along the local +Z axis of each site.
        site_id = int(sim.model.sensor_objid[sim.ray_sids[int(i)]])
        xyz = np.asarray(sim.data.site_xpos[site_id], dtype=float)
        axis = np.asarray(sim.data.site_xmat[site_id], dtype=float).reshape(3, 3)[:, 2]
        dz = float(axis[2])
        floor_distance = float(xyz[2] / -dz) if dz < -1e-5 and xyz[2] > 0 else None
        measured = float(packet.scan.ranges[int(i)])
        # Noise ~1.5 cm, so +/-5 cm is a useful *candidate* floor-hit test.
        matches_floor = (floor_distance is not None and
                         abs(measured - floor_distance) <= 0.05)
        ray_infos.append(dict(angle=floating(packet.scan.angles[int(i)], 3),
                              measured_m=floating(measured, 4),
                              origin_height_m=floating(xyz[2], 4),
                              vertical_direction_z=floating(dz, 4),
                              predicted_floor_range_m=(floating(floor_distance, 4)
                                                       if floor_distance is not None else None),
                              likely_floor_hit=bool(matches_floor)))
    nearest = min(ray_infos, key=lambda x: x['measured_m'])
    short = [r for r in ray_infos if r['measured_m'] < 0.445]
    return dict(time_s=floating(sim.time, 3),
                true_body_xyz_m=[floating(v, 4) for v in sim.data.xpos[sim.body_id]],
                true_roll_deg=floating(math.degrees(roll), 3),
                true_pitch_deg=floating(math.degrees(pitch), 3),
                nearest_front_range_m=nearest['measured_m'],
                nearest_front_ray=nearest,
                short_front_ray_count=len(short),
                likely_floor_hit_count=sum(r['likely_floor_hit'] for r in short),
                front_ray_count=len(ray_infos),
                rays=ray_infos)


def reachable_region_stats(nav, pose):
    occupied = nav.map.obstacles()
    s = nav.map.xy_to_cell(float(pose[0]), float(pose[1]))
    goal = nav.c.waypoints[nav.idx]
    g = nav.map.xy_to_cell(*goal)
    if not nav.map.inbounds(s):
        return dict(reason='start_outside_map')
    mask = occupied.copy()
    # Reproduce A*'s 3x3 local exception for the robot's start position.
    for yy in range(max(0, s[1]-1), min(mask.shape[0], s[1]+2)):
        for xx in range(max(0, s[0]-1), min(mask.shape[1], s[0]+2)):
            mask[yy, xx] = False
    q = deque([s]); seen = {s}
    while q:
        x, y = q.popleft()
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if not (dx or dy):
                    continue
                cell = (x+dx, y+dy)
                if cell in seen or not nav.map.inbounds(cell) or mask[cell[1], cell[0]]:
                    continue
                if dx and dy and (mask[y, cell[0]] or mask[cell[1], x]):
                    continue
                seen.add(cell); q.append(cell)
    return dict(start_cell=list(s), goal_cell=list(g),
                goal_occupied=bool(occupied[g[1], g[0]]) if nav.map.inbounds(g) else None,
                reachable_cells=len(seen), free_cells=int(np.count_nonzero(~mask)),
                fraction_grid_occupied=floating(np.mean(occupied), 4),
                goal_connected=bool(g in seen))


def render_map(nav, pose, time_s, file):
    mask = nav.map.obstacles()
    fig, ax = plt.subplots(figsize=(7, 6))
    xmin, xmax, ymin, ymax = nav.map.bounds
    ax.imshow(mask.astype(float), origin='lower', extent=[xmin, xmax, ymin, ymax],
              vmin=0, vmax=1, cmap='Greys', interpolation='nearest', alpha=0.82)
    # These obstacle outlines are from the known simulated scene FOR EVALUATION ONLY.
    # They are not supplied to the navigator or used in A*.
    from turtlebot_integrated.scenarios import get_scenario
    from matplotlib.patches import Rectangle
    for x1,x2,y1,y2 in get_scenario('nominal').obstacles:
        ax.add_patch(Rectangle((x1,y1),x2-x1,y2-y1,fill=False,
                               edgecolor='tab:red', linewidth=2, label='True obstacle (offline)' ))
    ax.scatter([pose[0]],[pose[1]],s=70,c='tab:blue',label='EKF robot position',zorder=10)
    w=nav.c.waypoints[nav.idx]
    ax.scatter([w[0]],[w[1]],marker='x',s=100,c='tab:orange',
               label=f'Waypoint {nav.idx}',zorder=10)
    ax.set_xlim(xmin,xmax);ax.set_ylim(ymin,ymax)
    ax.set_aspect('equal');ax.set_xlabel('x (m)');ax.set_ylabel('y (m)')
    ax.set_title(f'LiDAR-built inflated map at t={time_s:.2f}s')
    handles, labels = ax.get_legend_handles_labels()
    uniq = dict(zip(labels,handles));ax.legend(uniq.values(),uniq.keys(),fontsize=8)
    fig.tight_layout();fig.savefig(file,dpi=140);plt.close(fig)


def main():
    global first_route_failure
    cfg = Config.load(CFG)
    cfg.max_duration = 30.0
    old_observe = MuJoCoBackend.observe
    old_tick = Navigator.tick
    next_snap = 0

    def observe_probe(sim):
        pkt = old_observe(sim)
        records.append(inspect_rays(sim, pkt))
        return pkt

    def tick_probe(nav, t, pose, packet):
        nonlocal next_snap
        global first_route_failure
        result = old_tick(nav, t, pose, packet)
        planner_statuses[nav.last_planner_status] += 1
        if nav.last_planner_status == 'no_astar_route' and first_route_failure is None:
            first_route_failure = dict(t=floating(t,3), estimated_pose=[floating(x) for x in pose],
                                       **reachable_region_stats(nav, pose))
            render_map(nav, pose, t, OUT/'map_first_no_route.png')
        if next_snap < len(SNAPSHOT_TIMES) and t >= SNAPSHOT_TIMES[next_snap]:
            file=OUT/f'map_{next_snap:02d}_{int(t*100):05d}.png'
            render_map(nav, pose, t, file)
            map_snapshots.append(dict(t=floating(t,3), file=str(file),
                                      status=nav.last_planner_status,
                                      **reachable_region_stats(nav, pose)))
            next_snap+=1
        return result

    MuJoCoBackend.observe = observe_probe
    Navigator.tick = tick_probe
    try:
        summary=run_trial(cfg,'mujoco','nominal',7,OUT/'trial',capture_frames=False)
    finally:
        MuJoCoBackend.observe = old_observe
        Navigator.tick = old_tick

    short=[x for x in records if x['nearest_front_range_m'] < 0.445]
    possible_floor=[x for x in short if x['likely_floor_hit_count'] > 0]
    pitches=np.array([x['true_pitch_deg'] for x in records],float)
    rolls=np.array([x['true_roll_deg'] for x in records],float)
    report=dict(title='MuJoCo LiDAR and map connectivity diagnostic',
                run=summary,
                warning='True orientation and geometry are for offline diagnosis ONLY, never navigation.',
                scan_steps=len(records),
                front_near_obstacle_steps=len(short),
                suspected_floor_intersection_steps=len(possible_floor),
                pitch_abs_degrees_p50_p95=[floating(x,3) for x in np.percentile(np.abs(pitches),[50,95])],
                roll_abs_degrees_p50_p95=[floating(x,3) for x in np.percentile(np.abs(rolls),[50,95])],
                first_no_route=first_route_failure,
                planner_status_counts=dict(planner_statuses),
                map_snapshots=map_snapshots,
                short_range_samples=short[:3]+short[-3:],
                all_step_summaries=[{k:v for k,v in record.items() if k!='rays'} for record in records])
    (OUT/'lidar_tilt_report.json').write_text(json.dumps(report,indent=2)+'\n')
    print('=== v0.11 LIDAR / OCCUPANCY DIAGNOSTIC ===')
    print('Python test target: no changes to planner, controller, EKF, or safety.')
    print('Scans:',len(records),'near-front scans (<0.445m):',len(short))
    print('Near-front scans with at least one likely floor hit:',len(possible_floor))
    print('Absolute pitch, 50th/95th percentile (degrees):',report['pitch_abs_degrees_p50_p95'])
    print('Absolute roll, 50th/95th percentile (degrees):',report['roll_abs_degrees_p50_p95'])
    print('First no-route:',json.dumps(first_route_failure))
    print('Planner states:',dict(planner_statuses))
    print('Simulation success:',summary['success'])
    print('Report:',OUT/'lidar_tilt_report.json')
    print('Maps:',*[item['file'] for item in map_snapshots],sep='\n  ')
    if first_route_failure:
        print('Map at first no-route:',OUT/'map_first_no_route.png')

if __name__ == '__main__':
    main()
