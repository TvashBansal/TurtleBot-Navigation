"""Read-only failure analysis of a completed MuJoCo TurtleBot3 navigation run.

Offline use of simulator truth is ONLY for diagnosis, never for online navigation
or estimation. Does not change the project config, motor model, or saved run.

Usage:
  python scripts/diagnose_nominal_v04.py
  python scripts/diagnose_nominal_v04.py --skip-camera
  python scripts/diagnose_nominal_v04.py --trajectory outputs/my_run/trajectory.json

The MuJoCo camera probe repositions a separate, temporary simulation instance
at recorded *truth* poses to investigate why the ArUco tag is not detected.
Those replay renders are DIAGNOSTIC evidence only, not a successful test run.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "osmesa")
os.environ.setdefault("MPLBACKEND", "Agg")

import numpy as np

from turtlebot_integrated.config import Config
from turtlebot_integrated.geometry import wrap


def _float(value: float) -> float:
    return round(float(value), 4)


def _cam_geometry(pose: np.ndarray, cfg: Config) -> dict:
    dx = cfg.marker_xy[0] - pose[0]
    dy = cfg.marker_xy[1] - pose[1]
    rng = math.hypot(dx, dy)
    bearing = wrap(math.atan2(dy, dx) - pose[2])
    approx_in_view = (
        rng < cfg.camera_max_range
        and abs(bearing) < math.radians(cfg.camera_half_fov_deg)
    )
    return {
        "range_m": _float(rng),
        "bearing_rad": _float(bearing),
        "approximately_in_fov": bool(approx_in_view),
    }


def analyse_trajectory(rows: list[dict], cfg: Config) -> dict:
    if not rows:
        raise ValueError("Trajectory contains no recorded steps")
    truth = np.asarray([r["truth"] for r in rows], dtype=float)
    est = np.asarray([r["estimate"] for r in rows], dtype=float)
    commands = np.asarray([r["cmd"] for r in rows], dtype=float)
    times = np.asarray([r["t"] for r in rows], dtype=float)
    pos_err = np.linalg.norm(truth[:, :2] - est[:, :2], axis=1)
    head_err = (truth[:, 2] - est[:, 2] + math.pi) % (2 * math.pi) - math.pi
    g = np.array([_cam_geometry(p, cfg) for p in truth], dtype=object)
    candidate = [i for i, item in enumerate(g) if item["approximately_in_fov"]]
    camera_seen = [i for i, r in enumerate(rows) if r.get("camera_seen", False)]
    # Camera only processes a frame every fifth simulation step. Geometric
    # visibility at every step is NOT equivalent to a missed camera detection.
    sampled = [i for i in candidate if i % 5 == 0]
    summary = {
        "steps": len(rows),
        "duration_s": _float(times[-1]),
        "final_true_pose": [_float(z) for z in truth[-1]],
        "final_ekf_pose": [_float(z) for z in est[-1]],
        "final_position_estimation_error_m": _float(pos_err[-1]),
        "final_abs_heading_estimation_error_rad": _float(abs(head_err[-1])),
        "max_position_estimation_error_m": _float(max(pos_err)),
        "time_first_pos_error_over_0_25m_s": (
            _float(times[next(i for i, v in enumerate(pos_err) if v > 0.25)])
            if np.any(pos_err > 0.25) else None
        ),
        "time_first_pos_error_over_0_50m_s": (
            _float(times[next(i for i, v in enumerate(pos_err) if v > 0.50)])
            if np.any(pos_err > 0.50) else None
        ),
        "fraction_forward_under_0_02": _float(np.mean(commands[:, 0] <= 0.02)),
        "fraction_high_turn_over_0_7": _float(np.mean(np.abs(commands[:, 1]) >= 0.70)),
        "mean_abs_turn_command_rad_s": _float(np.mean(np.abs(commands[:, 1]))),
        "range_to_marker_at_final_pose_m": g[-1]["range_m"],
        "approx_geometric_fov_steps": len(candidate),
        "approx_geometric_fov_camera_sample_steps": len(sampled),
        "actual_camera_detections": len(camera_seen),
        "note": (
            "Geometry-only viewing condition ignores robot/obstacle occlusion, "
            "screen projection, lighting, image resolution, detection thresholds, "
            "and camera dropout. Not equivalent to real image detectability."
        ),
    }
    checkpoints = []
    for i in np.unique(np.linspace(0, len(rows) - 1, min(9, len(rows)), dtype=int)):
        checkpoints.append({
            "t_s": _float(times[i]),
            "true_pose": [_float(z) for z in truth[i]],
            "ekf_pose": [_float(z) for z in est[i]],
            "command_v_w": [_float(z) for z in commands[i]],
            "target_xy": [_float(z) for z in rows[i]["target"]],
            "position_estimation_error_m": _float(pos_err[i]),
            "heading_estimation_error_rad": _float(head_err[i]),
            "marker_geometry": g[i],
        })
    return {"summary": summary, "checkpoints": checkpoints,
            "geometrically_visible_indices": candidate}


def _set_pose(sim, pose: np.ndarray) -> None:
    """Offline renderer only. Never use this function in run_trial or Navigator."""
    x, y, yaw = map(float, pose)
    sim.data.qpos[:7] = [x, y, 0.088, math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
    sim.mj.mj_forward(sim.model, sim.data)


def render_probe(cfg: Config, rows: list[dict], analysis: dict,
                 outdir: Path, seed: int) -> list[dict]:
    import cv2
    from turtlebot_integrated.camera import marker_from_rgb
    from turtlebot_integrated.mujoco_backend import MuJoCoBackend
    from turtlebot_integrated.scenarios import get_scenario

    outdir.mkdir(parents=True, exist_ok=True)
    replay = []
    visible_idx = analysis["geometrically_visible_indices"]
    poses: list[tuple[str, np.ndarray]] = []

    # Fixed sanity checks: pointing toward the marker versus keeping yaw=0.
    for label, x, y, aimed in [
        ("center_reference", 0.50, 0.0, False),
        ("left_unrotated", 0.50, -0.30, False),
        ("left_aimed", 0.50, -0.30, True),
        ("right_unrotated", 0.50, 0.30, False),
        ("right_aimed", 0.50, 0.30, True),
        ("near_dock_aimed", 1.05, -0.15, True),
    ]:
        yaw = math.atan2(cfg.marker_xy[1] - y, cfg.marker_xy[0] - x) if aimed else 0.0
        poses.append((label, np.array([x, y, yaw], dtype=float)))

    # Include physically reached poses recorded by the *failed* run.
    if visible_idx:
        picks = np.linspace(0, len(visible_idx) - 1, min(6, len(visible_idx)), dtype=int)
        for j, ix in enumerate(sorted({visible_idx[p] for p in picks})):
            poses.append((f"actual_pose_{j:02d}_step_{ix:04d}",
                          np.asarray(rows[ix]["truth"], dtype=float)))
    else:
        for ix in np.linspace(0, len(rows) - 1, min(3, len(rows)), dtype=int):
            poses.append((f"actual_pose_step_{ix:04d}",
                          np.asarray(rows[ix]["truth"], dtype=float)))

    sim = MuJoCoBackend(cfg, get_scenario("nominal", seed), seed=seed)
    try:
        for label, pose in poses:
            _set_pose(sim, pose)
            image = sim.render_rgb(camera="front_camera")
            destination = outdir / f"{label}.png"
            if image is None:
                raise RuntimeError("MuJoCo did not return a camera image")
            if not cv2.imwrite(str(destination), cv2.cvtColor(image, cv2.COLOR_RGB2BGR)):
                raise RuntimeError(f"Could not write camera image: {destination}")
            detection = marker_from_rgb(image, cfg)
            replay.append({
                "case": label, "pose": [_float(v) for v in pose],
                "geometry": _cam_geometry(pose, cfg),
                "detected": bool(detection is not None),
                "measured_range_m": _float(detection.distance) if detection else None,
                "measured_bearing_rad": _float(detection.bearing) if detection else None,
                "image": str(destination),
            })
            print(f"{label:35s} detected={str(detection is not None):5s} "
                  f"range={_cam_geometry(pose, cfg)['range_m']:5.2f} "
                  f"bearing={_cam_geometry(pose, cfg)['bearing_rad']:+.2f} "
                  f"image={destination}")
    finally:
        sim.close()
    return replay


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trajectory", type=Path,
                        default=Path("outputs/mujoco_nominal_v04_candidate/trajectory.json"))
    parser.add_argument("--config", type=Path,
                        default=Path("configs/mujoco_odom_candidate.yaml"))
    parser.add_argument("--output", type=Path,
                        default=Path("outputs/diagnostics/nominal_v04_replay"))
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--skip-camera", action="store_true",
                        help="Run log analysis without requiring MuJoCo/rendering")
    args = parser.parse_args()

    if not args.trajectory.is_file():
        raise SystemExit(f"Missing {args.trajectory}. Run the nominal v0.4 experiment first.")
    cfg = Config.load(args.config)
    rows = json.loads(args.trajectory.read_text(encoding="utf-8"))
    analysis = analyse_trajectory(rows, cfg)
    summary = analysis["summary"]

    print("\n=== V0.4 CLOSED-LOOP FAILURE ANALYSIS ===")
    for k, value in summary.items():
        if k != "note": print(f"{k}: {value}")
    print("\n=== TIME CHECKPOINTS ===")
    for cp in analysis["checkpoints"]:
        print(f"t={cp['t_s']:6.1f}s truth={cp['true_pose']} EKF={cp['ekf_pose']} "
              f"cmd={cp['command_v_w']} est_err={cp['position_estimation_error_m']:.3f}m "
              f"head_err={cp['heading_estimation_error_rad']:+.3f}rad")

    args.output.mkdir(parents=True, exist_ok=True)
    report = {"source_trajectory": str(args.trajectory),
              "source_config": str(args.config), "seed": args.seed,
              "trajectory_summary": summary,
              "time_checkpoints": analysis["checkpoints"],
              "camera_probe": []}
    if not args.skip_camera:
        print("\n=== OFFLINE CAMERA IMAGE REPLAY ===")
        report["camera_probe"] = render_probe(cfg, rows, analysis,
                                               args.output / "camera_frames", args.seed)
        print("\nNote: camera replay uses offline simulator truth poses for diagnostics only.")

    dest = args.output / "diagnostic_report.json"
    dest.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"\nSaved report: {dest}")


if __name__ == "__main__":
    main()
