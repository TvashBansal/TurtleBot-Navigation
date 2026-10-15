"""Compare real MuJoCo run summaries and measured controller stop reasons."""
from pathlib import Path
from collections import Counter
import json

EXPERIMENTS={
 'v05_gyro':Path('outputs/mujoco_nominal_v05_gyro'),
 'v06_conservative':Path('outputs/nominal_v06_conservative'),
 'v07_steering':Path('outputs/nominal_v07_steering'),
 'v07_stable':Path('outputs/nominal_v07_stable'),
}

def main():
    for label,folder in EXPERIMENTS.items():
        path=folder/'summary.json'
        if not path.exists():
            print(f'{label}: not run yet')
            continue
        summary=json.loads(path.read_text())
        print(f'{label}: success={summary["success"]} '
              f'goal_error={summary["position_error_m"]}m '
              f'heading_rmse={summary["rmse_heading_rad"]}rad '
              f'camera_updates={summary.get("accepted_camera_updates",0)} '
              f'collision={summary["collision"]}')
        trajectory=folder/'trajectory.json'
        if trajectory.exists():
            rows=json.loads(trajectory.read_text())
            counts=Counter(r.get('nav_stop_reason','unavailable') for r in rows)
            print('  command causes: '+', '.join(f'{k}={100*v/len(rows):.1f}%' for k,v in sorted(counts.items())))
            idx=Counter(r.get('waypoint_index','unavailable') for r in rows)
            print('  waypoint residence: '+', '.join(f'{k}={100*v/len(rows):.1f}%' for k,v in sorted(idx.items(),key=lambda a: str(a[0]))))

if __name__=='__main__':main()
