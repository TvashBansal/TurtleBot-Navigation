"""Audit the built-in MuJoCo robustness suite: performance + disturbance validity.

Usage: python scripts/review_robustness_v14.py outputs/robustness_v14_smoke
Writes audit.json and audit.csv into the suite directory; does not alter trials.
"""
import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path
from statistics import mean

SCENARIOS = ('nominal', 'noise', 'blocked', 'disturbance', 'camera_loss')
FIELDS = ('scenario', 'seed', 'success', 'collision', 'duration_s',
          'position_error_m', 'heading_error_rad', 'rmse_x_m', 'rmse_y_m',
          'camera_detection_rate', 'accepted_camera_updates',
          'planner_no_route_fraction', 'disturbance_applied_steps')


def audit_suite(folder: Path):
    summary = json.loads((folder / 'aggregate.json').read_text(encoding='utf-8'))
    if summary.get('backend') != 'mujoco':
        raise ValueError('This assessment audit requires the MuJoCo backend')
    rows = summary['scenario_results']
    if summary['count'] != len(rows):
        raise ValueError('Suite count is inconsistent')
    groups = defaultdict(list)
    failures = []
    for r in rows:
        scen, seed = r['scenario'], int(r['seed'])
        if scen not in SCENARIOS:
            raise ValueError(f'Unexpected scenario: {scen}')
        folder_summary = folder / scen / f'seed_{seed}' / 'summary.json'
        if not folder_summary.is_file():
            raise ValueError(f'Missing physics trial summary: {folder_summary}')
        if json.loads(folder_summary.read_text(encoding='utf-8')) != r:
            raise ValueError(f'Aggregate does not match individual summary: {scen} seed {seed}')
        applied = r.get('disturbance_applied_steps')
        if applied is None:
            failures.append(f'{scen} seed {seed}: missing external-force telemetry (old backend?)')
        elif scen == 'disturbance' and applied <= 0:
            failures.append(f'{scen} seed {seed}: external disturbance never applied')
        elif scen != 'disturbance' and applied != 0:
            failures.append(f'{scen} seed {seed}: unexpected external disturbance')
        groups[scen].append(r)
    seeds_by_scenario = {name: sorted(int(r['seed']) for r in groups[name]) for name in SCENARIOS}
    if len({tuple(s) for s in seeds_by_scenario.values()}) != 1:
        failures.append('Scenario groups did not use identical random seed sets')
    per_scenario = []
    for name in SCENARIOS:
        trial = groups[name]
        if not trial:
            failures.append(f'No trials recorded for scenario {name}')
            continue
        per_scenario.append({
            'scenario': name, 'trials': len(trial),
            'successes': sum(bool(r['success']) for r in trial),
            'success_rate': round(mean(bool(r['success']) for r in trial), 3),
            'collisions': sum(bool(r['collision']) for r in trial),
            'mean_position_error_m': round(mean(r['position_error_m'] for r in trial), 4),
            'mean_heading_error_rad': round(mean(r['heading_error_rad'] for r in trial), 4),
            'mean_no_route_fraction': round(mean(r['planner_no_route_fraction'] for r in trial), 3),
            'mean_accepted_camera_updates': round(mean(r['accepted_camera_updates'] for r in trial), 2),
            'disturbance_confirmed_trials': sum((r.get('disturbance_applied_steps') or 0) > 0 for r in trial),
        })
    result = {
        'backend': 'mujoco', 'count': len(rows), 'seed_sets': seeds_by_scenario,
        'disturbance_schedule_valid': not failures,
        'audit_failures': failures, 'per_scenario': per_scenario,
        'note': 'Measured physics robustness, not a guarantee of real TurtleBot3 hardware equivalence; front support remains experimental.',
    }
    (folder / 'audit.json').write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
    with (folder / 'audit.csv').open('w', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS)
        writer.writeheader()
        for r in rows:
            writer.writerow({k:r.get(k) for k in FIELDS})
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder', type=Path, help='Completed MuJoCo suite directory')
    args = parser.parse_args()
    report = audit_suite(args.folder)
    print('=== v0.14 ROBUSTNESS AUDIT ===')
    for r in report['per_scenario']:
        print(f"{r['scenario']:13} {r['successes']}/{r['trials']} success, "
              f"collisions={r['collisions']}, goal error={r['mean_position_error_m']:.4f} m, "
              f"camera updates={r['mean_accepted_camera_updates']:.1f}, "
              f"disturbance applied={r['disturbance_confirmed_trials']}/{r['trials']}")
    if report['audit_failures']:
        for err in report['audit_failures']:
            print('INVALID EVIDENCE:', err)
        raise SystemExit(1)
    print('PASS: each disturbance trial physically applied the external wrench.')
    print('Audit:', args.folder / 'audit.json')
    print('CSV:', args.folder / 'audit.csv')

if __name__ == '__main__':
    main()
