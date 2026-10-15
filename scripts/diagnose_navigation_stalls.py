"""Offline v0.6 navigation-stall attribution from archived StepLog records.

Replays only the LQR command from logged EKF pose and target; it DOES NOT
replay the simulator or expose ground truth to the online controller.
The difference between reconstructed and recorded linear commands estimates
how often the reactive LiDAR safety layer slowed or stopped the robot.

Usage:
  python scripts/diagnose_navigation_stalls.py
  python scripts/diagnose_navigation_stalls.py --runs outputs/nominal_v06_camera_only outputs/nominal_v06_conservative
"""
from __future__ import annotations

import argparse
import json
import math
from dataclasses import asdict
from pathlib import Path

import numpy as np

from turtlebot_integrated.config import Config
from turtlebot_integrated.control import LQRTracker
from turtlebot_integrated.geometry import wrap

DEFAULT_RUNS = (
    "outputs/mujoco_nominal_v05_gyro",
    "outputs/nominal_v06_camera_only",
    "outputs/nominal_v06_conservative",
)


def contiguous_intervals(mask: np.ndarray, times: np.ndarray, dt: float, limit: int = 5):
    blocks = []
    start = None
    for i, marked in enumerate(mask):
        if marked and start is None:
            start = i
        if start is not None and (not marked or i == len(mask) - 1):
            end = i + 1 if marked else i
            blocks.append({
                "start_s": round(float(times[start]), 2),
                "end_s": round(float(times[end - 1]), 2),
                "duration_s": round(float((end - start) * dt), 2),
                "steps": end - start,
            })
            start = None
    return sorted(blocks, key=lambda x: x["steps"], reverse=True)[:limit]


def analysis(folder: Path):
    log_file = folder / "trajectory.json"
    cfg_file = folder / "config_snapshot.json"
    if not log_file.is_file() or not cfg_file.is_file():
        raise FileNotFoundError(f"Missing trajectory.json or config_snapshot.json: {folder}")
    records = json.loads(log_file.read_text(encoding="utf-8"))
    if not records:
        raise ValueError(f"Empty trajectory: {folder}")
    cfg = Config(**json.loads(cfg_file.read_text(encoding="utf-8")))
    tracker = LQRTracker(cfg)
    times = np.array([r["t"] for r in records], dtype=float)
    commands = np.array([r["cmd"] for r in records], dtype=float)
    poses = np.array([r["estimate"] for r in records], dtype=float)
    targets = np.array([r["target"] for r in records], dtype=float)
    nominal_cmd = []
    for pose, target in zip(poses, targets):
        dist_to_dock = math.dist(pose[:2], cfg.dock_pose[:2])
        # The final-heading mode is only selected for the final docking target.
        heading = (cfg.dock_pose[2] if dist_to_dock < .3 and
                   math.dist(target, cfg.dock_pose[:2]) < 1e-6 else None)
        nominal_cmd.append(tracker.track(pose, tuple(target), heading))
    nominal = np.asarray(nominal_cmd)
    # nav.tick can only lower linear speed vs LQR (or stop at collision risk).
    stopped = commands[:, 0] <= .020
    controller_turn_stop = stopped & (nominal[:, 0] <= .020)
    safety_stop = stopped & (nominal[:, 0] > .020)
    safety_slow = (~stopped) & (nominal[:, 0] - commands[:, 0] > .035)
    otherwise_moving = ~(controller_turn_stop | safety_stop | safety_slow)
    saturated_turn = np.abs(commands[:, 1]) >= .95 * cfg.max_omega
    target_angles = np.array([
        wrap(math.atan2(y - p[1], x - p[0]) - p[2])
        for p, (x, y) in zip(poses, targets)
    ])
    n = len(records)
    percent = lambda x: round(float(np.mean(x) * 100), 1)
    report = {
        "run": str(folder),
        "steps": n,
        "duration_s": round(float(times[-1]), 2),
        "linear_motion_classification_percent": {
            "controller_selected_stop": percent(controller_turn_stop),
            "safety_override_stopped": percent(safety_stop),
            "safety_override_slowed": percent(safety_slow),
            "remaining_moving": percent(otherwise_moving),
        },
        "high_turn_saturation_percent": percent(saturated_turn),
        "abs_heading_to_target_over_1p05_percent": percent(np.abs(target_angles) > 1.05),
        "avg_abs_heading_to_target_rad": round(float(np.mean(np.abs(target_angles))), 3),
        "mean_recorded_linear_speed_mps": round(float(np.mean(commands[:, 0])), 4),
        "mean_reconstructed_lqr_linear_speed_mps": round(float(np.mean(nominal[:, 0])), 4),
        "mean_recorded_abs_angular_speed_radps": round(float(np.mean(np.abs(commands[:, 1]))), 4),
        "longest_safety_stops": contiguous_intervals(safety_stop, times, cfg.dt),
        "longest_controller_stops": contiguous_intervals(controller_turn_stop, times, cfg.dt),
        "summary": json.loads((folder / "summary.json").read_text(encoding="utf-8")) if (folder / "summary.json").is_file() else None,
        "caveats": [
            "Safety-override attribution is inferred from recorded commands and replayed LQR using EKF pose and target; the LiDAR ranges were not logged.",
            "Simulator true pose is not used in LQR reconstruction.",
            "A change in the controller source since the original run may invalidate the inferred cause. Use the saved Git commit to confirm code parity.",
        ],
    }
    return report


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--runs", nargs="+", default=list(DEFAULT_RUNS))
    p.add_argument("--out", default="outputs/diagnostics/v06_navigation_stalls.json")
    args = p.parse_args()
    results = []
    for name in args.runs:
        folder = Path(name)
        if not folder.is_dir():
            print(f"SKIP {folder}: no such folder")
            continue
        try:
            result = analysis(folder)
        except (OSError, KeyError, ValueError, TypeError) as e:
            print(f"ERROR {folder}: {e}")
            continue
        results.append(result)
        print(f"\n===== {folder} =====")
        for key, val in result["linear_motion_classification_percent"].items():
            print(f"{key:34s}: {val:5.1f}%")
        print(f"turn-command saturation: {result['high_turn_saturation_percent']:.1f}%")
        print(f"target bearing > 1.05rad: {result['abs_heading_to_target_over_1p05_percent']:.1f}%")
        print("longest safety stops:", result["longest_safety_stops"][:3])
        print("longest controller stops:", result["longest_controller_stops"][:3])
    if not results:
        raise SystemExit("No readable runs found; check --runs paths")
    path = Path(args.out)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(results, indent=2) + "\n", encoding="utf-8")
    print("\nSaved:", path)


if __name__ == "__main__":
    main()
