"""Diagnose v0.14 blocked trial without changing controllers, maps or scoring.

Usage:
    python scripts/diagnose_blocked_v15.py outputs/robustness_v14_smoke

Reads real archived MuJoCo trial data, reports planner-state/waypoint causes,
compares strictly configured docking tolerance to simulator success, and writes
blocked_diagnosis_v15.json in the suite folder. Environment geometry is read
ONLY for offline analysis; neither navigator nor EKF receives it.
"""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import json
import math
from pathlib import Path

from turtlebot_integrated.config import Config
from turtlebot_integrated.geometry import point_rect_distance
from turtlebot_integrated.scenarios import get_scenario


def compact_row(row: dict) -> dict:
    return {
        k: row.get(k) for k in (
            't', 'truth', 'estimate', 'waypoint_index', 'planner_status',
            'nav_stop_reason', 'front_clearance_m', 'planner_path_nodes',
            'planner_visible_nodes', 'cmd', 'camera_seen',
            'camera_update_accepted', 'distance_to_goal'
        )
    }


def trial_report(folder: Path, cfg: Config) -> dict:
    summary_path = folder / 'summary.json'
    rows_path = folder / 'trajectory.json'
    summary = json.loads(summary_path.read_text(encoding='utf-8'))
    rows = json.loads(rows_path.read_text(encoding='utf-8'))
    if not isinstance(rows, list) or not rows:
        raise ValueError(f'No trajectory records: {rows_path}')
    by_waypoint: dict[str, dict] = {}
    for index in sorted(set(int(row.get('waypoint_index') or 0) for row in rows)):
        selected = [row for row in rows if int(row.get('waypoint_index') or 0) == index]
        by_waypoint[str(index)] = {
            'steps': len(selected),
            'time_first_s': round(float(selected[0]['t']), 3),
            'time_last_s': round(float(selected[-1]['t']), 3),
            'planner_states': dict(Counter(str(row.get('planner_status')) for row in selected)),
            'stop_reasons': dict(Counter(str(row.get('nav_stop_reason')) for row in selected)),
            'first_estimate': selected[0].get('estimate'),
            'last_estimate': selected[-1].get('estimate'),
        }
    path_length = sum(
        math.dist(prev['truth'][:2], curr['truth'][:2])
        for prev, curr in zip(rows, rows[1:])
    )
    first_route_loss = next((compact_row(row) for row in rows if row.get('planner_status') == 'no_astar_route'), None)
    first_safety_brake = next((compact_row(row) for row in rows if row.get('nav_stop_reason') == 'safety_brake'), None)
    first_no_visible = next((compact_row(row) for row in rows if row.get('planner_status') == 'no_visible_onward_path'), None)
    front = [float(row['front_clearance_m']) for row in rows if row.get('front_clearance_m') is not None]
    front.sort()
    true_end = rows[-1].get('truth')
    est_end = rows[-1].get('estimate')
    error = float(summary['position_error_m'])
    heading = float(summary['heading_error_rad'])
    return {
        'scenario': summary.get('scenario'),
        'seed': summary.get('seed'),
        'summary': summary,
        'configured_position_tolerance_m': cfg.dock_tolerance,
        'simulator_scoring_position_threshold_m': cfg.dock_tolerance + 0.06,
        'inside_configured_position_tolerance': error < cfg.dock_tolerance,
        'inside_simulator_scoring_position_threshold': error < cfg.dock_tolerance + 0.06,
        'inside_simulator_scoring_heading_threshold': heading < 0.25,
        'steps': len(rows),
        'distance_travelled_m': round(path_length, 4),
        'first_no_astar_route': first_route_loss,
        'first_no_visible_onward_path': first_no_visible,
        'first_safety_brake': first_safety_brake,
        'final_true_pose': true_end,
        'final_estimated_pose': est_end,
        'final_estimation_xy_disagreement_m': round(math.dist(true_end[:2], est_end[:2]),4) if true_end and est_end else None,
        'stop_reasons': dict(Counter(str(row.get('nav_stop_reason')) for row in rows)),
        'planner_states': dict(Counter(str(row.get('planner_status')) for row in rows)),
        'camera_detections': sum(bool(row.get('camera_seen')) for row in rows),
        'camera_updates': sum(bool(row.get('camera_update_accepted')) for row in rows),
        'front_clearance_m_p10_p50_p90': [round(front[min(len(front)-1, int(frac*(len(front)-1)))],3) for frac in (0.1,0.5,0.9)] if front else None,
        'by_waypoint': by_waypoint,
        'last_six_rows': [compact_row(row) for row in rows[-6:]],
    }


def geometry_report(cfg: Config) -> dict:
    scenario = get_scenario('blocked')
    normal = get_scenario('nominal')
    additional = scenario.obstacles[len(normal.obstacles):]
    return {
        'additional_blocked_obstacles': additional,
        'map_inflation_m': cfg.map_inflation,
        'map_grid_resolution_m': cfg.map_resolution,
        'robot_radius_m': cfg.robot_radius,
        'notes': 'Straight-line obstacle clearances are diagnostic ONLY. They do not establish a safe planned route or accurately reproduce grid dilation.',
        'waypoint_distance_to_new_obstacle_m': [
            {'waypoint_index': i, 'waypoint': list(wp),
             'closest_added_rect_m': round(min(point_rect_distance(*wp, ob) for ob in additional), 4)}
            for i,wp in enumerate(cfg.waypoints)
        ] if additional else [],
    }


def analyze_suite(suite: Path) -> dict:
    block_path = suite / 'blocked' / 'seed_0'
    config_path = block_path / 'config_snapshot.json'
    cfg = Config.load(None)
    if config_path.exists():
        values = json.loads(config_path.read_text(encoding='utf-8'))
        for key, val in values.items():
            if not hasattr(cfg, key):
                raise ValueError(f'Unexpected key in snapshot: {key}')
            setattr(cfg,key, tuple(tuple(p) for p in val) if key == 'waypoints' else tuple(val) if key in ('dock_pose','marker_xy','start_pose') else val)
    blocked = trial_report(block_path, cfg)
    nominal_path = suite / 'nominal' / 'seed_0'
    nominal = trial_report(nominal_path, cfg) if nominal_path.joinpath('summary.json').exists() else None
    report = {'blocked': blocked, 'nominal_reference': nominal, 'geometry': geometry_report(cfg),
              'evidence_type': 'post-run analysis; controller did not consume ground truth'}
    (suite / 'blocked_diagnosis_v15.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    return report


def print_report(report: dict, suite: Path) -> None:
    print('=== v0.15 BLOCKED SCENARIO / STRICT TOLERANCE DIAGNOSIS ===')
    for label, item in (('blocked',report['blocked']), ('nominal', report['nominal_reference'])):
        if item is None:
            continue
        summary = item['summary']
        print(f"{label.upper()}: reported_success={summary['success']}, "
              f"strict_position_pass={item['inside_configured_position_tolerance']}, "
              f"goal_error={summary['position_error_m']:.4f}m, "
              f"length={item['distance_travelled_m']:.3f}m, "
              f"final_true_xy={item['final_true_pose'][:2]}, "
              f"final_EKF_xy={item['final_estimated_pose'][:2]}")
        print('  planner_states:', item['planner_states'])
        print('  stop_reasons:', item['stop_reasons'])
        for k,value in item['by_waypoint'].items():
            print(f"  waypoint_{k}: steps={value['steps']}, "
                  f"planner={value['planner_states']}, stops={value['stop_reasons']}")
        for event in ('first_no_astar_route','first_no_visible_onward_path','first_safety_brake'):
            example = item[event]
            if example:
                print(' ', event, {'t':round(float(example['t']),3),
                                   'waypoint_index':example['waypoint_index'],
                                   'estimate':example['estimate'],
                                   'front_clearance_m':example['front_clearance_m']})
    print('EXTRA BLOCKED OBSTACLE / WAYPOINT CLEARANCES:')
    print(' ', report['geometry']['waypoint_distance_to_new_obstacle_m'])
    print('NOTE: scoring position threshold = configured tolerance + 0.06 m; keep both metrics.')
    print('Full report:', suite / 'blocked_diagnosis_v15.json')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('suite',nargs='?',type=Path, default=Path('outputs/robustness_v14_smoke'))
    args=parser.parse_args()
    report=analyze_suite(args.suite)
    print_report(report,args.suite)


if __name__=='__main__':
    main()
