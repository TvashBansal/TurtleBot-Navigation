"""Test offline strict scoring audit using independent temporary files."""
import importlib.util
import json
from pathlib import Path


def _script():
    path=Path(__file__).resolve().parents[1]/'scripts'/'review_robustness_v16.py'
    spec=importlib.util.spec_from_file_location('review_robustness_v16', path)
    mod=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_audit_separates_simulator_success_from_strict_tolerance(tmp_path):
    mod=_script()
    rows=[]
    for scenario in mod.SCENARIOS:
        r=dict(backend='mujoco',scenario=scenario,seed=0,success=True,
               collision=False,position_error_m=0.19,heading_error_rad=0.10,
               planner_no_route_fraction=0.,accepted_camera_updates=2,
               disturbance_applied_steps=5 if scenario=='disturbance' else 0,
               bypassed_intermediate_waypoints=[1] if scenario=='blocked' else [])
        rowdir=tmp_path/scenario/'seed_0'
        rowdir.mkdir(parents=True)
        (rowdir/'summary.json').write_text(json.dumps(r))
        rows.append(r)
    (tmp_path/'aggregate.json').write_text(json.dumps(dict(backend='mujoco',count=len(rows),scenario_results=rows)))
    (tmp_path/'config_snapshot.json').write_text(json.dumps(dict(dock_tolerance=0.18)))
    report=mod.audit_suite(tmp_path)
    assert report['audit_valid']
    assert all(x['reported_successes']==1 for x in report['per_scenario'])
    assert all(x['strict_position_passes']==0 for x in report['per_scenario'])
    assert next(x for x in report['per_scenario'] if x['scenario']=='blocked')['bypass_trials']==1
