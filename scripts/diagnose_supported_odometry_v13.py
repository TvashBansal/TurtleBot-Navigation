"""v0.13 diagnostic: physically measure effective encoder radius on supported MuJoCo model.

Offline physics truth is used for evaluation of calibration candidates ONLY.
No true pose is passed to the EKF, navigation, or controller.
This diagnostic does not alter the repository's physics, LiDAR, or success criteria.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

from turtlebot_integrated.config import Config
from turtlebot_integrated.mujoco_backend import MuJoCoBackend
from turtlebot_integrated.scenarios import get_scenario
from turtlebot_integrated.geometry import wrap


BASELINE = Path('configs/mujoco_v12_front_support_probe.yaml')
CANDIDATE = Path('configs/mujoco_v13_supported_odom_trial.yaml')
OUT = Path('outputs/diagnostics/v13_supported_odometry')


def straight_probe(config: Config, speed: float, duration: float, seed: int = 7) -> dict:
    sim = MuJoCoBackend(config, get_scenario('nominal', seed), seed=seed, record_camera=False)
    try:
        # Allow initial chassis/wheels to settle on the ground before measuring.
        for _ in range(int(round(0.8 / config.dt))):
            sim.step(0.0, 0.0)
        start = sim.observe()
        start_truth = sim.get_truth().copy()
        for _ in range(int(round(duration / config.dt))):
            sim.step(speed, 0.0)
        end = sim.observe()
        end_truth = sim.get_truth().copy()
        dl = float(end.encoder_left - start.encoder_left)
        dr = float(end.encoder_right - start.encoder_right)
        displacement = end_truth[:2] - start_truth[:2]
        # Forward projection on initial heading (straight drive), with transverse
        # drift and true yaw reported independently for model sanity checking.
        ct, st = math.cos(start_truth[2]), math.sin(start_truth[2])
        observed_m = float(ct * displacement[0] + st * displacement[1])
        transverse_m = float(-st * displacement[0] + ct * displacement[1])
        wheel_sum = dl + dr
        effective_radius_m = 2 * observed_m / wheel_sum if abs(wheel_sum) > 1e-7 else None
        nominal_prediction_m = config.wheel_radius * wheel_sum * 0.5
        old_radius = Config.load(BASELINE).odom_wheel_radius
        new_radius = Config.load(CANDIDATE).odom_wheel_radius
        if old_radius is None or new_radius is None:
            raise ValueError('Both odometry trial configs must specify odom_wheel_radius')
        old_pred_m = old_radius * wheel_sum * 0.5
        new_pred_m = new_radius * wheel_sum * 0.5
        return {
            'speed_command_m_s': speed,
            'duration_s': duration,
            'observed_forward_m': round(observed_m, 5),
            'observed_transverse_m': round(transverse_m, 5),
            'actual_delta_heading_rad': round(wrap(end_truth[2] - start_truth[2]), 5),
            'encoder_delta_l_rad': round(dl, 5),
            'encoder_delta_r_rad': round(dr, 5),
            'measured_effective_radius_m': None if effective_radius_m is None else round(effective_radius_m, 5),
            'predicted_with_model_radius_m': round(nominal_prediction_m, 5),
            'error_old_radius_m': round(old_pred_m - observed_m, 5),
            'error_new_radius_m': round(new_pred_m - observed_m, 5),
        }
    finally:
        sim.close()


def main() -> None:
    baseline = Config.load(BASELINE)
    candidate = Config.load(CANDIDATE)
    if not (baseline.mujoco_front_support_enabled and candidate.mujoco_front_support_enabled):
        raise RuntimeError('Both trials must use the SAME supported chassis')
    if baseline.dock_tolerance != candidate.dock_tolerance:
        raise RuntimeError('Docking tolerance must be identical')
    if baseline.wheel_radius != candidate.wheel_radius or baseline.wheel_separation != candidate.wheel_separation:
        raise RuntimeError('The physics wheel geometry must be unchanged')
    segments = [(0.10, 2.4), (0.15, 2.4), (0.15, 5.0)]
    data = [straight_probe(baseline, speed, duration) for speed, duration in segments]
    old_mean = float(np.mean([abs(r['error_old_radius_m']) for r in data]))
    new_mean = float(np.mean([abs(r['error_new_radius_m']) for r in data]))
    report = {
        'type': 'offline_ground_truth_calibration_diagnostic',
        'baseline_radius_m': baseline.odom_wheel_radius,
        'candidate_radius_m': candidate.odom_wheel_radius,
        'same_supported_mujoco_physics': True,
        'controller_receives_true_pose': False,
        'segments': data,
        'mean_abs_distance_error_old_m': round(old_mean, 5),
        'mean_abs_distance_error_candidate_m': round(new_mean, 5),
        'candidate_improves_short_straight_trials': bool(new_mean < old_mean),
        'note': 'Open-loop only; full EKF, visual updates, and docking need separate validation',
    }
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / 'supported_wheel_calibration.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print('=== v0.13 STABILISED-CHASSIS ODOMETRY COMPARISON ===')
    for r in data:
        print(f"v={r['speed_command_m_s']:.2f}m/s for {r['duration_s']:.1f}s"
              f"  physics={r['observed_forward_m']:.4f}m"
              f"  inferred_radius={r['measured_effective_radius_m']}m"
              f"  old_error={r['error_old_radius_m']:+.4f}m"
              f"  new_error={r['error_new_radius_m']:+.4f}m")
    print('Mean |distance error| old:', report['mean_abs_distance_error_old_m'], 'm')
    print('Mean |distance error| candidate:', report['mean_abs_distance_error_candidate_m'], 'm')
    print('Candidate improves open-loop distance:', report['candidate_improves_short_straight_trials'])
    print('Saved:', OUT / 'supported_wheel_calibration.json')
    print('NOTE: Diagnostic truth used for offline evaluation ONLY.')


if __name__ == '__main__':
    main()
