"""Offline physics/encoder consistency experiment (no navigation truth feedback).

Compares original wheel odometry with calibrated odometry on identical commands.
Ground truth is used only AFTER each step, for evaluation and calibration report.
No simulator truth enters the EKF or controller in the main experiment.
"""
import json
import math
import os
from pathlib import Path

os.environ.setdefault('MUJOCO_GL', 'osmesa')
os.environ.setdefault('MPLBACKEND', 'Agg')

import numpy as np
from turtlebot_integrated.config import Config
from turtlebot_integrated.ekf import PoseEKF
from turtlebot_integrated.geometry import wrap
from turtlebot_integrated.mujoco_backend import MuJoCoBackend
from turtlebot_integrated.scenarios import get_scenario

OUT = Path('outputs/diagnostics/odometry_v04')
OUT.mkdir(parents=True, exist_ok=True)


def probe(v, w, steps, cfg, seed=7):
    sim = MuJoCoBackend(cfg, get_scenario('nominal', seed), seed, record_camera=False)
    estimator = PoseEKF(cfg, np.array(cfg.start_pose, dtype=float))
    try:
        t0 = sim.get_truth()
        before = sim.observe()
        left, right = before.encoder_left, before.encoder_right
        for _ in range(steps):
            sim.step(v, w)
            p = sim.observe()
            estimator.predict(p.encoder_left-left, p.encoder_right-right)
            left, right = p.encoder_left, p.encoder_right
        truth = sim.get_truth()
        pos_err = float(np.linalg.norm(truth[:2]-estimator.x[:2]))
        heading_err = float(wrap(truth[2]-estimator.x[2]))
        return {
            'command_v_mps': v, 'command_w_radps': w, 'duration_s': steps*cfg.dt,
            'truth_pose': np.round(truth, 4).tolist(),
            'estimated_pose': np.round(estimator.x, 4).tolist(),
            'position_error_m': round(pos_err, 4),
            'heading_error_rad': round(heading_err, 4),
            'physics_delta_heading_rad': round(wrap(float(truth[2]-t0[2])), 4),
        }
    finally:
        sim.close()


def main():
    tests = [(.10, 0, 25),(.16, 0, 25),(0, .45, 25),
             (0, -.45, 25),(.10, .35, 25),(.10, -.35, 25)]
    configs = {'nominal': Config(), 'odom_candidate': Config.load('configs/mujoco_odom_candidate.yaml')}
    results = {}
    for name, cfg in configs.items():
        print('\n'+'='*12+' '+name+' '+'='*12)
        rows = [probe(v,w,n,cfg) for v,w,n in tests]
        for r in rows:
            print(f"v={r['command_v_mps']:+.2f}, w={r['command_w_radps']:+.2f} | "
                  f"position error={r['position_error_m']:.3f} m, "
                  f"heading error={r['heading_error_rad']:+.3f} rad")
        results[name] = {'experiments':rows,
                         'mean_position_error_m':round(float(np.mean([r['position_error_m'] for r in rows])),4),
                         'mean_absolute_heading_error_rad':round(float(np.mean([abs(r['heading_error_rad']) for r in rows])),4)}
        print('Mean position error:',results[name]['mean_position_error_m'],'m')
        print('Mean absolute heading error:',results[name]['mean_absolute_heading_error_rad'],'rad')
    out=OUT/'calibration_comparison.json'
    out.write_text(json.dumps({'method':'offline truth-based measurement only; online EKF encoder-only',
                                'results':results},indent=2)+'\n')
    print('\nSaved:', out)
    print('NOTE: this does not establish successful closed-loop navigation or camera updates.')

if __name__=='__main__':
    main()
