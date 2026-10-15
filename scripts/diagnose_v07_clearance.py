#!/usr/bin/env python3
"""Offline audit of v0.7 MuJoCo navigation safety interventions.

Reads a saved trajectory and compares its LiDAR-derived forward clearance to
obstacle geometry at the simulator pose (EVALUATION ONLY). No simulator truth
is passed to the robot's estimator, controller, or planner. Does not rerun MuJoCo.

Usage:
  python scripts/diagnose_v07_clearance.py
  python scripts/diagnose_v07_clearance.py --run outputs/nominal_v07_steering
"""
from __future__ import annotations

import argparse
import json
import math
from collections import Counter
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Rectangle

from turtlebot_integrated.config import Config
from turtlebot_integrated.scenarios import get_scenario


def ray_box_range(x: float, y: float, angle: float, box: tuple[float, float, float, float]) -> float:
    """Distance from a 2D point to the first intersection of a positive ray and AABB."""
    xlo, xhi, ylo, yhi = box
    dx, dy = math.cos(angle), math.sin(angle)
    near, far = -math.inf, math.inf
    for o, d, lo, hi in ((x, dx, xlo, xhi), (y, dy, ylo, yhi)):
        if abs(d) < 1e-12:
            if o < lo or o > hi:
                return math.inf
            continue
        a, b = (lo-o)/d, (hi-o)/d
        near = max(near, min(a, b))
        far = min(far, max(a, b))
    if far < max(near, 0.0):
        return math.inf
    return max(0.0, near) if near >= 0 else 0.0


def approximate_front_range(pose, scenario, cfg, return_source=False):
    """Expected front clearance from scene geometry. Diagnostic approximation only."""
    x, y, theta = map(float, pose)
    xmin, xmax, ymin, ymax = scenario.bounds
    # MuJoCo uses 0.12m-thick walls centred on world bounds.
    walls = [(xmin-.12, xmin, ymin-.1, ymax+.1),
             (xmax, xmax+.12, ymin-.1, ymax+.1),
             (xmin, xmax, ymin-.12, ymin),
             (xmin, xmax, ymax, ymax+.12)]
    boxes = [('wall', b) for b in walls] + [('obstacle', b) for b in scenario.obstacles]
    angles = np.linspace(-math.pi, math.pi, cfg.lidar_beams, endpoint=False)
    best = (math.inf, 'none')
    for a in angles[np.abs(angles) < .4]:
        for name, rect in boxes:
            d = ray_box_range(x, y, theta + float(a), rect)
            if d < best[0]:
                best = (d, name)
    value = min(float(cfg.lidar_max_range), best[0])
    return (value, best[1]) if return_source else value


def segment_intersects_inflated_box(start, end, rect, margin):
    x1, y1 = start
    x2, y2 = end
    xlo, xhi, ylo, yhi = rect
    ext = (xlo-margin, xhi+margin, ylo-margin, yhi+margin)
    span = math.hypot(x2-x1, y2-y1)
    count = max(2, math.ceil(span/.035))
    for t in np.linspace(0., 1., count):
        x, y = (1-t)*x1+t*x2, (1-t)*y1+t*y2
        if ext[0] <= x <= ext[1] and ext[2] <= y <= ext[3]:
            return True
    return False


def longest_period(rows, test, number=4):
    runs=[]
    start=None
    for i,r in enumerate(rows):
        if test(r) and start is None:
            start=i
        if start is not None and (not test(r) or i==len(rows)-1):
            end = i if i == len(rows)-1 and test(r) else i-1
            t0=float(rows[start]['t']); t1=float(rows[end]['t'])
            runs.append(dict(start_s=round(t0,2), end_s=round(t1,2),
                             duration_s=round(t1-t0+float(rows[1]['t'])-float(rows[0]['t']),2) if len(rows)>1 else 0,
                             waypoint_index=rows[start].get('waypoint_index')))
            start=None
    return sorted(runs,key=lambda r:r['duration_s'],reverse=True)[:number]


def q(arr):
    if not arr:return None
    return [round(float(a),3) for a in np.quantile(np.array(arr),[0.1,0.5,0.9])]


def analyze(run: Path, output: Path) -> dict:
    trajectory=run/'trajectory.json'
    if not trajectory.is_file():
        raise FileNotFoundError(f'Missing trajectory: {trajectory}')
    rows=json.loads(trajectory.read_text())
    if not rows:
        raise ValueError('Trajectory is empty')
    cfg=Config.load(None)
    snapshot=run/'config_snapshot.json'
    if snapshot.is_file():
        config_data=json.loads(snapshot.read_text())
        for key,val in config_data.items():
            if hasattr(cfg,key):
                setattr(cfg,key,val)
    scenario_name=json.loads((run/'summary.json').read_text()).get('scenario','nominal')
    scenario=get_scenario(scenario_name,int(cfg.seed))
    clearance=[]; geometric=[]; discrepancy=[]; closest_kind=[]; intersection=[]
    margin=float(cfg.robot_radius)+.07
    for i,r in enumerate(rows):
        p=np.array(rows[i-1]['truth'] if i>0 else cfg.start_pose,dtype=float)
        expected,kind=approximate_front_range(p,scenario,cfg,True)
        sensed=r.get('front_clearance_m')
        geometric.append(expected)
        closest_kind.append(kind)
        clearance.append(None if sensed is None else float(sensed))
        discrepancy.append(None if sensed is None else float(sensed)-expected)
        s=r.get('estimate'); g=r.get('target')
        intersection.append(bool(s is not None and g is not None and any(
            segment_intersects_inflated_box(s[:2],g[:2],rect,margin)
            for rect in scenario.obstacles)))
    counts=Counter(str(r.get('nav_stop_reason','missing')) for r in rows)
    waypoints=sorted({r.get('waypoint_index') for r in rows if r.get('waypoint_index') is not None})
    per_wp=[]
    for w in waypoints:
        indices=[i for i,r in enumerate(rows) if r.get('waypoint_index')==w]
        per_wp.append({
            'index':w,'steps':len(indices),'fraction':round(len(indices)/len(rows),3),
            'stop_reasons':dict(Counter(rows[i].get('nav_stop_reason') for i in indices)),
            'measured_clearance_10_50_90_m':q([clearance[i] for i in indices if clearance[i] is not None]),
            'geometric_clearance_10_50_90_m':q([geometric[i] for i in indices]),
            'target_lines_near_obstacles_fraction':round(sum(intersection[i] for i in indices)/len(indices),3)
        })
    valid=[i for i,v in enumerate(discrepancy) if v is not None and math.isfinite(v)]
    disagreement=[i for i in valid if discrepancy[i]<-.15]
    n=len(rows)
    report={
        'run':str(run), 'scenario':scenario_name,'steps':n,
        'brake_threshold_m':round(cfg.robot_radius+.13,3),
        'slow_threshold_m':round(cfg.robot_radius+.32,3),
        'stop_reasons':dict(counts),
        'stop_reason_fractions':{k:round(v/n,3) for k,v in counts.items()},
        'measured_front_clearance_10_50_90_m':q([v for v in clearance if v is not None]),
        'geometry_expected_front_clearance_10_50_90_m':q(geometric),
        'clearance_difference_measured_minus_geometry_10_50_90_m':q([discrepancy[i] for i in valid]),
        'fraction_measured_clearance_more_than_15cm_shorter_than_geometry':round(len(disagreement)/max(1,len(valid)),3),
        'target_lines_near_obstacles_fraction':round(sum(intersection)/n,3),
        'per_waypoint':per_wp,
        'longest_safety_brakes':longest_period(rows,lambda r:r.get('nav_stop_reason')=='safety_brake'),
        'notable_disagreement_samples':[
            {'t_s':round(float(rows[i]['t']),2),'pose':rows[i-1]['truth'] if i>0 else cfg.start_pose,
             'measured_m':round(clearance[i],3),'expected_geometry_m':round(geometric[i],3),
             'difference_m':round(discrepancy[i],3)}
            for i in sorted(disagreement,key=lambda i:discrepancy[i])[:6]],
        'analysis_limitations':[
            'Scene geometry is used offline ONLY for evaluation and never for control.',
            'Expected front ranges are simplified 2D ray intersections; scan was measured before the logged post-step pose.',
            'Noisy LiDAR and self-occlusion can cause differences; this does not prove a specific fault.',
            'Direct line to lookahead target is not the entire planned A* path.',
            'A safety slowdown is not a safety stop; preserve emergency braking.'
        ]
    }
    output.mkdir(parents=True,exist_ok=True)
    (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    fig,ax=plt.subplots(figsize=(9,7))
    for k,rect in enumerate(scenario.obstacles):
        xlo,xhi,ylo,yhi=rect
        ax.add_patch(Rectangle((xlo,ylo),xhi-xlo,yhi-ylo,edgecolor='brown',facecolor='#bf9a80',alpha=.5,label='Physical obstacle' if k==0 else None))
        ax.add_patch(Rectangle((xlo-margin,ylo-margin),xhi-xlo+2*margin,yhi-ylo+2*margin,
                               fill=False,edgecolor='darkorange',linestyle='--',linewidth=1.2,
                               label='Robot clearance buffer' if k==0 else None))
    actual=np.array([r['truth'][:2] for r in rows],float)
    estimate=np.array([r['estimate'][:2] for r in rows],float)
    ax.plot(actual[:,0],actual[:,1],label='Actual MuJoCo motion (evaluation)',color='steelblue')
    ax.plot(estimate[:,0],estimate[:,1],label='EKF estimated path',color='purple',alpha=.7)
    stop_idx=np.array([i for i,r in enumerate(rows) if r.get('nav_stop_reason')=='safety_brake'],dtype=int)
    if len(stop_idx):
        ax.scatter(actual[stop_idx,0],actual[stop_idx,1],s=8,color='red',label='LiDAR brake location',zorder=5)
    for k,wp in enumerate(cfg.waypoints):
        ax.scatter(*wp,marker='x',s=75,color='black')
        ax.annotate(f'WP{k}',wp,xytext=(5,5),textcoords='offset points')
    ax.scatter(*cfg.dock_pose[:2],s=90,marker='*',color='green',label='Dock')
    ax.set(xlabel='X (m)',ylabel='Y (m)',title='v0.7 MuJoCo safety and waypoint audit')
    ax.set_aspect('equal');ax.set_xlim(-2.4,2.4);ax.set_ylim(-2.,2.)
    ax.grid(alpha=.2);ax.legend(fontsize=8,loc='upper left');fig.tight_layout()
    fig.savefig(output/'trajectory_safety_audit.png',dpi=150)
    plt.close(fig)
    print('=== MUJOCO v0.7 SAFETY AND PLANNER AUDIT ===')
    for k in ('steps','brake_threshold_m','slow_threshold_m','stop_reason_fractions',
              'measured_front_clearance_10_50_90_m','geometry_expected_front_clearance_10_50_90_m',
              'clearance_difference_measured_minus_geometry_10_50_90_m',
              'fraction_measured_clearance_more_than_15cm_shorter_than_geometry',
              'target_lines_near_obstacles_fraction'):
        print(f'{k}: {report[k]}')
    print('=== PER WAYPOINT ===')
    for row in report['per_waypoint']:
        print(row)
    print('=== LONGEST SAFETY BRAKES ===')
    for row in report['longest_safety_brakes']:
        print(row)
    print('=== STRONGEST RANGE DISAGREEMENTS (if any) ===')
    for row in report['notable_disagreement_samples']:
        print(row)
    print(f'Saved report: {output/"report.json"}')
    print(f'Saved plot:   {output/"trajectory_safety_audit.png"}')
    return report


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--run',type=Path,default=Path('outputs/nominal_v07_steering'))
    parser.add_argument('--out',type=Path,default=Path('outputs/diagnostics/v07_safety_audit'))
    args=parser.parse_args()
    analyze(args.run,args.out)


if __name__=='__main__':
    main()
