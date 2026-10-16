"""Assess actual MuJoCo robustness without conflating scoring and strict docking.

Usage: python scripts/review_robustness_v16.py outputs/robustness_v16_smoke
Writes audit_v16.json/csv; never modifies the original trial summaries.
"""
import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path
from statistics import mean

SCENARIOS = ('nominal', 'noise', 'blocked', 'disturbance', 'camera_loss')
FIELDS = ('scenario', 'seed', 'success', 'strict_dock_position_pass',
          'strict_dock_pose_pass', 'collision', 'duration_s', 'position_error_m',
          'heading_error_rad', 'accepted_camera_updates',
          'planner_no_route_fraction', 'bypassed_intermediate_waypoints',
          'disturbance_applied_steps')


def audit_suite(folder: Path):
    aggregate = json.loads((folder/'aggregate.json').read_text(encoding='utf-8'))
    config = json.loads((folder/'config_snapshot.json').read_text(encoding='utf-8'))
    if aggregate['backend'] != 'mujoco':
        raise ValueError('MuJoCo physics runs required for this report')
    rows = aggregate['scenario_results']
    if aggregate['count'] != len(rows):
        raise ValueError('Aggregate count does not match its trial results')
    tolerance = float(config['dock_tolerance'])
    grouped = defaultdict(list)
    failures = []
    enriched = []
    for r in rows:
        scenario = r['scenario']
        if scenario not in SCENARIOS:
            raise ValueError(f'Unknown scenario {scenario}')
        seed = int(r['seed'])
        path = folder/scenario/f'seed_{seed}'/'summary.json'
        if not path.is_file() or json.loads(path.read_text(encoding='utf-8')) != r:
            raise ValueError(f'Missing or mismatched MuJoCo trial summary {scenario} seed {seed}')
        steps = r.get('disturbance_applied_steps')
        if steps is None:
            failures.append(f'{scenario} seed {seed}: missing disturbance telemetry')
        elif scenario == 'disturbance' and steps <= 0:
            failures.append(f'{scenario} seed {seed}: physical disturbance not applied')
        elif scenario != 'disturbance' and steps != 0:
            failures.append(f'{scenario} seed {seed}: unexpected disturbance')
        strict_pos = r['position_error_m'] < tolerance and not r['collision']
        strict_pose = strict_pos and r['heading_error_rad'] < 0.18
        if ('strict_dock_position_pass' in r and r['strict_dock_position_pass'] != strict_pos
                and abs(r['position_error_m'] - tolerance) > 0.0001):
            failures.append(f'{scenario} seed {seed}: strict position flag inconsistent')
        if ('strict_dock_pose_pass' in r and r['strict_dock_pose_pass'] != strict_pose
                and abs(r['position_error_m'] - tolerance) > 0.0001
                and abs(r['heading_error_rad'] - 0.18) > 0.0001):
            failures.append(f'{scenario} seed {seed}: strict pose flag inconsistent')
        rr = dict(r)
        # Prefer unrounded per-trial flags where available; the JSON position
        # error was rounded to four decimal places for historical summaries.
        rr['strict_dock_position_pass'] = r.get('strict_dock_position_pass', strict_pos)
        rr['strict_dock_pose_pass'] = r.get('strict_dock_pose_pass', strict_pose)
        rr['bypassed_intermediate_waypoints'] = r.get('bypassed_intermediate_waypoints', [])
        enriched.append(rr)
        grouped[scenario].append(rr)
    seed_sets = {name: sorted(r['seed'] for r in grouped[name]) for name in SCENARIOS}
    if any(not seed_sets[name] for name in SCENARIOS):
        failures.append('At least one scenario has no trials')
    if len({tuple(s) for s in seed_sets.values()}) != 1:
        failures.append('Unequal random seed sets across scenarios')
    per_scenario = []
    for name in SCENARIOS:
        group = grouped[name]
        if not group:
            continue
        per_scenario.append(dict(scenario=name, trials=len(group),
            reported_successes=sum(bool(r['success']) for r in group),
            strict_position_passes=sum(r['strict_dock_position_pass'] for r in group),
            strict_pose_passes=sum(r['strict_dock_pose_pass'] for r in group),
            collisions=sum(bool(r['collision']) for r in group),
            bypass_trials=sum(bool(r['bypassed_intermediate_waypoints']) for r in group),
            mean_goal_error_m=round(mean(r['position_error_m'] for r in group), 4),
            mean_no_route_fraction=round(mean(r['planner_no_route_fraction'] for r in group), 3),
            mean_camera_updates=round(mean(r['accepted_camera_updates'] for r in group), 2),
            disturbance_confirmed=sum((r.get('disturbance_applied_steps') or 0) > 0 for r in group)))
    result = dict(backend='mujoco', count=len(rows), dock_tolerance_m=tolerance,
                  strict_heading_tolerance_rad=0.18, seed_sets=seed_sets,
                  audit_valid=not failures, audit_failures=failures,
                  per_scenario=per_scenario,
                  scoring_note='Experiment success uses dock_tolerance + 0.06 m and heading < 0.25 rad; strict criteria shown separately.',
                  caveat='Physical chassis front support is an experimental simulation approximation.')
    (folder/'audit_v16.json').write_text(json.dumps(result, indent=2)+'\n',encoding='utf-8')
    with (folder/'audit_v16.csv').open('w',encoding='utf-8',newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDS)
        writer.writeheader()
        for r in enriched:
            row = {f:r.get(f) for f in FIELDS}
            row['bypassed_intermediate_waypoints'] = json.dumps(row['bypassed_intermediate_waypoints'])
            writer.writerow(row)
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder',type=Path)
    args=parser.parse_args()
    report=audit_suite(args.folder)
    print('=== v0.16 ROBUSTNESS + STRICT DOCKING AUDIT ===')
    for row in report['per_scenario']:
        print(f"{row['scenario']:12s} simulator={row['reported_successes']}/{row['trials']}  "
              f"strict_pose={row['strict_pose_passes']}/{row['trials']}  "
              f"bypass_trials={row['bypass_trials']}  "
              f"collisions={row['collisions']}  "
              f"mean_error={row['mean_goal_error_m']:.4f}m  "
              f"no_route={row['mean_no_route_fraction']:.3f}")
    print('Physical disturbance audit:', 'PASS' if report['audit_valid'] else 'FAIL')
    print('Report:', args.folder/'audit_v16.json')
    if not report['audit_valid']:
        for fail in report['audit_failures']:
            print('ERROR:',fail)
        raise SystemExit(1)

if __name__=='__main__':
    main()
