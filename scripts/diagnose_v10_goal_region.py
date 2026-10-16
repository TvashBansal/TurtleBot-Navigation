"""Diagnose MuJoCo v0.10 goal-region fallback without modifying controller or planner.

Upload this file to scripts/ in TurtleBot-Navigation, then execute:

    MUJOCO_GL=osmesa MPLBACKEND=Agg python scripts/diagnose_v10_goal_region.py

Report is experimental evidence only. Occupancy is an estimate from LiDAR,
not ground-truth obstacles, and the experiment never feeds truth to control.
"""
from __future__ import annotations

import argparse
from collections import Counter, deque
from dataclasses import asdict
import json
import math
from pathlib import Path

import numpy as np

from turtlebot_integrated.config import Config
from turtlebot_integrated.experiment import run_trial
from turtlebot_integrated.mapping import OccupancyMap


def inspect_map(om: OccupancyMap, start, goal, radius):
    """Inspect exactly the occupancy and corner-cutting rules used in A*."""
    s = om.xy_to_cell(*start)
    g = om.xy_to_cell(*goal)
    if not om.inbounds(s) or not om.inbounds(g):
        return dict(reason='out_of_bounds', goal_cell_blocked=None,
                    candidates_total=0, candidates_reachable=0)
    occupied = om.obstacles()
    goal_blocked = bool(occupied[g[1], g[0]])
    # Match the local start exception made by the planner.
    for yy in range(max(0, s[1] - 1), min(om.height, s[1] + 2)):
        for xx in range(max(0, s[0] - 1), min(om.width, s[0] + 2)):
            occupied[yy, xx] = False
    capture_radius = max(0, float(radius) - 0.015)
    extent = int(math.ceil(radius / om.res)) + 2
    candidates = set()
    for yy in range(max(0, g[1]-extent), min(om.height, g[1]+extent+1)):
        for xx in range(max(0, g[0]-extent), min(om.width, g[0]+extent+1)):
            if not occupied[yy, xx] and math.dist(om.cell_to_xy((xx,yy)), goal) <= capture_radius:
                candidates.add((xx,yy))
    q=deque([s]); seen={s}
    while q:
        x,y=q.popleft()
        for dx in (-1,0,1):
            for dy in (-1,0,1):
                if not (dx or dy): continue
                n=(x+dx,y+dy)
                if n in seen or not om.inbounds(n) or occupied[n[1], n[0]]: continue
                if dx and dy and (occupied[y,x+dx] or occupied[y+dy,x]): continue
                seen.add(n);q.append(n)
    reachable=candidates & seen
    if not candidates: cause='no_free_cells_in_goal_region'
    elif not reachable: cause='free_goal_region_disconnected'
    else: cause='reachable_region_exists'
    closest = min((math.dist(om.cell_to_xy(p),goal) for p in reachable), default=None)
    return dict(reason=cause, goal_cell_blocked=goal_blocked,
                candidates_total=len(candidates), candidates_reachable=len(reachable),
                occupied_cells=int(occupied.sum()), total_cells=int(occupied.size),
                start_cell=list(s), goal_cell=list(g),
                closest_reachable_distance_m=(round(closest,3) if closest is not None else None))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--backend', choices=['mujoco', 'kinematic'], default='mujoco')
    p.add_argument('--config', default='configs/mujoco_v10_goal_region_probe.yaml')
    p.add_argument('--scenario', default='nominal')
    p.add_argument('--seed', type=int, default=7)
    p.add_argument('--out', default='outputs/diagnostics/v10_goal_region_probe')
    a=p.parse_args()
    cfg=Config.load(a.config)
    if not cfg.nav_intermediate_goal_region_fallback:
        raise SystemExit('This config does not enable the goal-region fallback; choose v0.10 config.')
    out=Path(a.out);out.mkdir(parents=True,exist_ok=True)
    stats=Counter();samples={};records=[]
    original_plan=OccupancyMap.plan
    original_region=OccupancyMap.plan_to_goal_region

    def plan_probe(om, start, goal):
        result=original_plan(om,start,goal)
        stats['exact_astar_found' if result else 'exact_astar_failed']+=1
        return result

    def region_probe(om, start, goal, radius):
        inspection=inspect_map(om,start,goal,radius)
        result=original_region(om,start,goal,radius)
        category=inspection['reason']
        stats['region_fallback_calls']+=1
        stats['region_'+category]+=1
        stats['region_success' if result else 'region_failure']+=1
        k=('region_success_' if result else 'region_failure_')+category
        if k not in samples:
            samples[k]=dict(start_xy=list(start), nominal_waypoint=list(goal),
                            radius_m=radius, **inspection,
                            selected_endpoint=list(result[-1]) if result else None)
        if len(records)<5000:
            records.append(dict(category=category, success=bool(result),
                                start_xy=list(start), goal_xy=list(goal),
                                candidate_count=inspection['candidates_total'],
                                reachable_count=inspection['candidates_reachable'],
                                chosen_endpoint=list(result[-1]) if result else None))
        return result

    OccupancyMap.plan=plan_probe
    OccupancyMap.plan_to_goal_region=region_probe
    try:
        result=run_trial(cfg,a.backend,a.scenario,a.seed,out,capture_frames=False)
    finally:
        OccupancyMap.plan=original_plan
        OccupancyMap.plan_to_goal_region=original_region

    logs=json.loads((out/'trajectory.json').read_text())
    statuses=Counter(x.get('planner_status','unavailable') for x in logs)
    by_waypoint={str(i):dict(Counter(x.get('planner_status','unavailable') for x in logs if x.get('waypoint_index')==i)) for i in range(len(cfg.waypoints))}
    report=dict(description='Observed LiDAR occupancy-map and A* outcomes; no ground truth used for planning.',
                backend=a.backend,config=a.config,seed=a.seed, astar_calls=dict(stats),
                planner_states=dict(statuses), planner_states_by_waypoint=by_waypoint,
                representative_cases=samples, region_calls=records,
                run_summary=result)
    (out/'goal_region_diagnosis.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print('=== V0.10 A* GOAL-REGION DIAGNOSIS ===')
    print('A* and fallback calls:',dict(stats))
    print('Planner states:',dict(statuses))
    print('Planner states by waypoint:',by_waypoint)
    for key,sample in samples.items():
        print('CASE:',key,json.dumps(sample))
    print('Run success:',result['success'])
    print('Run goal position error (m):',result['position_error_m'])
    print('Full report:',out/'goal_region_diagnosis.json')
    print('NOTE: LiDAR occupancy is not ground-truth geometry; diagnosis does not bypass safety.')


if __name__=='__main__': main()
