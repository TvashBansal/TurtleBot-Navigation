"""Diagnose *why* LiDAR occupancy-map A* fails during a short MuJoCo run.

This is instrumentation only. Navigator, EKF, LQR, and safety actions are
unchanged. Simulator ground truth is not used in mapping or planning.

Example:
    python scripts/diagnose_astar_v09.py
    python scripts/diagnose_astar_v09.py --backend kinematic
"""
from __future__ import annotations

import argparse
from collections import Counter, deque
import json
from pathlib import Path

import numpy as np

from turtlebot_integrated.config import Config
from turtlebot_integrated.experiment import run_trial
from turtlebot_integrated.mapping import OccupancyMap


def classify_no_route(occupancy_map: OccupancyMap, start, goal):
    """Explain an empty A* result using only the same LiDAR occupancy grid."""
    s = occupancy_map.xy_to_cell(*start)
    g = occupancy_map.xy_to_cell(*goal)
    if not occupancy_map.inbounds(s):
        return "start_out_of_bounds", None
    if not occupancy_map.inbounds(g):
        return "goal_out_of_bounds", None
    mask = occupancy_map.obstacles()
    # Match the start-cell carve-out used by OccupancyMap.plan.
    for y in range(max(0, s[1] - 1), min(mask.shape[0], s[1] + 2)):
        for x in range(max(0, s[0] - 1), min(mask.shape[1], s[0] + 2)):
            mask[y, x] = False

    if mask[g[1], g[0]]:
        return "goal_in_inflated_obstacle", mask

    # A* forbids diagonal corner cutting; match this rule exactly.
    queue = deque([s])
    seen = {s}
    while queue:
        x, y = queue.popleft()
        if (x, y) == g:
            return "route_exists_but_astar_empty", mask
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if not (dx or dy):
                    continue
                n = (x + dx, y + dy)
                if n in seen or not occupancy_map.inbounds(n) or mask[n[1], n[0]]:
                    continue
                if dx and dy and (mask[y, x + dx] or mask[y + dy, x]):
                    continue
                seen.add(n)
                queue.append(n)
    return "no_connected_free_path", mask


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend', choices=['mujoco', 'kinematic'], default='mujoco')
    parser.add_argument('--scenario', default='nominal')
    parser.add_argument('--seed', type=int, default=7)
    parser.add_argument('--config', default='configs/mujoco_v09_short_probe.yaml')
    parser.add_argument('--out', default='outputs/diagnostics/v09_astar_probe')
    args = parser.parse_args()
    cfg = Config.load(args.config)
    output = Path(args.out)
    output.mkdir(parents=True, exist_ok=True)

    counts = Counter()
    samples = {}
    last_status = None
    transition_counts = Counter()
    original_plan = OccupancyMap.plan

    def probe(self, start, goal):
        nonlocal last_status
        result = original_plan(self, start, goal)
        if result:
            status = 'astar_path_found'
            mask = None
        else:
            status, mask = classify_no_route(self, start, goal)
        counts[status] += 1
        if last_status != status:
            transition_counts[f'{last_status or "initial"} -> {status}'] += 1
            last_status = status
        if status not in samples:
            record = {
                'start_est_xy': [round(float(x), 4) for x in start],
                'goal_xy': [round(float(x), 4) for x in goal],
                'map_resolution_m': self.res,
                'occupied_cells_raw': int(np.count_nonzero(self.log_odds > 1.6)),
                'path_nodes': len(result),
                'failure_reason': status,
            }
            if mask is not None:
                record['occupied_cells_inflated'] = int(mask.sum())
                record['total_map_cells'] = int(mask.size)
                record['goal_cell'] = list(self.xy_to_cell(*goal))
                # Save the first occupancy mask of each kind; this is only diagnostic.
                record['mask_npy'] = f'{status}_first_mask.npy'
                np.save(output / record['mask_npy'], mask)
            samples[status] = record
        return result

    OccupancyMap.plan = probe
    try:
        summary = run_trial(cfg, args.backend, args.scenario, args.seed, output, capture_frames=False)
    finally:
        OccupancyMap.plan = original_plan

    trajectory = json.loads((output / 'trajectory.json').read_text())
    by_planner_step = Counter(r.get('planner_status') for r in trajectory)
    by_safety_step = Counter(r.get('nav_stop_reason') for r in trajectory)
    no_route_steps = [r for r in trajectory if r.get('planner_status') == 'no_astar_route']
    report = {
        'backend': args.backend,
        'scenario': args.scenario,
        'seed': args.seed,
        'notes': 'Diagnostic observations use estimated state and LiDAR occupancy grid; controls unchanged.',
        'astar_call_counts': dict(counts),
        'astar_call_transitions': dict(transition_counts),
        'first_samples_by_cause': samples,
        'planner_state_step_counts': dict(by_planner_step),
        'command_cause_step_counts': dict(by_safety_step),
        'first_no_route_step_time_s': (round(no_route_steps[0]['t'], 3) if no_route_steps else None),
        'last_no_route_step_time_s': (round(no_route_steps[-1]['t'], 3) if no_route_steps else None),
        'first_no_route_step': ({
            'estimated_pose': no_route_steps[0]['estimate'],
            'waypoint_index': no_route_steps[0].get('waypoint_index'),
            'front_clearance_m': no_route_steps[0].get('front_clearance_m'),
        } if no_route_steps else None),
        'run_summary': summary,
    }
    (output / 'astar_diagnosis.json').write_text(json.dumps(report, indent=2) + '\n')
    print('\n=== v0.9 A* ROOT-CAUSE DIAGNOSIS ===')
    print('A* call outcomes:', dict(counts))
    print('Transitions:', dict(transition_counts))
    print('Per-step planner states:', dict(by_planner_step))
    print('Per-step control/safety causes:', dict(by_safety_step))
    print('First no-route time:', report['first_no_route_step_time_s'])
    print('First no-route step:', report['first_no_route_step'])
    print('Success:', summary['success'])
    print('Saved:', output / 'astar_diagnosis.json')
    print('Do not treat diagnostic occupancy masks as ground-truth obstacles.')


if __name__ == '__main__':
    main()
