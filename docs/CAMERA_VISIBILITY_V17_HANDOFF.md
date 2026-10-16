# v0.17: No-camera blocked-docking diagnosis (no online changes)

## Background
The v0.16 blocked MuJoCo seed-0 trial safely bypassed intermediate waypoint 1, reached the final waypoint and reported zero A* route blockage for most of its navigation. The run still failed: 0.3561 m true docking position error, 0.3563 rad heading error, **zero actual ArUco detections**. We must isolate camera FOV/occlusion versus rendered ArUco detection before changing the robot.

## Inputs
- `outputs/blocked_v16_bypass_seed0/trajectory.json` and `summary.json`
- `configs/mujoco_v16_blocked_bypass_trial.yaml` (exact configuration used for that run)

## Commands
```bash
source .venv/bin/activate
export MUJOCO_GL=osmesa
export MPLBACKEND=Agg
python -m pytest -q
python scripts/diagnose_camera_visibility_v17.py outputs/blocked_v16_bypass_seed0 \
  --config configs/mujoco_v16_blocked_bypass_trial.yaml
```

## Outputs
`outputs/blocked_v16_bypass_seed0/camera_visibility_v17/report.json` and up to 8 PNG camera snapshots rendered at reconstructed *approximately planar* logged true poses. The reconstructed frames are **not** original camera captures, because the trajectory records x/y/yaw, not exact z/roll/pitch or wheel-joint poses.

## Interpretation
- **Few geometry opportunities**: likely marker outside camera FOV or occluded along the actual trajectory. Inspect controller approach and camera placement before tuning ArUco detector.
- **Many geometry opportunities, reconstructed detector works**: the issue may involve dynamic attitude, frame timing, image dropout, occlusion or unrecorded physical orientations. Capture real frames in a subsequent controlled replay; do **not** infer a solved issue from reconstructions alone.
- **Many geometry opportunities, reconstructed detector fails**: inspect PNGs and validate marker texture, rendered camera orientation, UV mapping and OpenCV image pipeline.

Ground truth is used for offline assessment only and **must never feed navigation, sensing or EKF**. This update changes no physics, planner, controller, scoring or safety limits. Retain v0.13 success and v0.16 blocked failure for transparent comparison.
