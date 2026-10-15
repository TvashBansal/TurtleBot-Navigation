"""Compare saved nominal MuJoCo trials without implying successful docking.

Use after running one or more v0.6 configs. Only previously saved summary
and trajectory JSONs are read. Does not run physics or touch past archives.
"""
import json
from pathlib import Path

CASES = {
    'v04_odom': 'outputs/mujoco_nominal_v04_candidate',
    'v05_gyro': 'outputs/mujoco_nominal_v05_gyro',
    'v06_camera_only': 'outputs/nominal_v06_camera_only',
    'v06_conservative': 'outputs/nominal_v06_conservative',
}


def evaluate(label: str, folder: Path):
    summary_path = folder / 'summary.json'
    if not summary_path.exists():
        return None
    summary = json.loads(summary_path.read_text(encoding='utf-8'))
    if summary.get('backend') != 'mujoco' or summary.get('scenario') != 'nominal':
        raise ValueError(f'Unexpected experiment at {summary_path}: use nominal MuJoCo only')
    row = {
        'name': label,
        'source_dir': str(folder),
        'seed': summary.get('seed'),
        'success': summary.get('success', False),
        'collision': summary.get('collision'),
        'duration_s': summary.get('duration_s'),
        'position_error_m': summary.get('position_error_m'),
        'rmse_heading_rad': summary.get('rmse_heading_rad'),
        'camera_measurements': summary.get('measurements', 0),
        'accepted_camera_updates': summary.get('accepted_camera_updates', 0),
    }
    trajectory = folder / 'trajectory.json'
    if trajectory.exists():
        rows = json.loads(trajectory.read_text(encoding='utf-8'))
        if rows:
            row['percent_near_zero_forward'] = round(100*sum(float(x['cmd'][0]) <= .02 for x in rows)/len(rows), 1)
            row['percent_high_turn'] = round(100*sum(abs(float(x['cmd'][1])) > .7 for x in rows)/len(rows), 1)
    return row


def main():
    rows = [r for label, path in CASES.items() if (r := evaluate(label, Path(path))) is not None]
    if not rows:
        raise SystemExit('No nominal MuJoCo summaries present under the expected outputs/ folders.')
    seeds = {r['seed'] for r in rows}
    print('=== NOMINAL MUJOCO EXPERIMENT COMPARISON ===')
    if len(seeds)>1:
        print('WARNING: trials have different random seeds; comparisons are not controlled.')
    for r in rows:
        print(f"{r['name']:20} success={str(r['success']):5}  goal_error={r['position_error_m']} m "
              f"heading_rmse={r['rmse_heading_rad']} rad  detections={r['camera_measurements']} "
              f"accepted={r['accepted_camera_updates']}  stopped%={r.get('percent_near_zero_forward','?')}")
    folder=Path('outputs/diagnostics/nominal_v06_comparison')
    folder.mkdir(parents=True,exist_ok=True)
    report={'status':'experimental_comparison_not_assessment_success', 'seed_consistent': len(seeds)==1,
            'backend':'mujoco','scenario':'nominal','trials':rows}
    (folder/'comparison.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print('Saved:', folder/'comparison.json')


if __name__=='__main__':
    main()
