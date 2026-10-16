# v0.18 Camera image: hide MuJoCo diagnostic rangefinder visualization

## Problem and observed evidence

The v0.16 blocked scenario safely bypasses intermediate waypoint 1, but has
zero ArUco measurements and misses strict docking. The v0.17 off-line scene
reconstruction displays the marker in the camera field-of-view, **crossed by
bright yellow lines**. The MuJoCo model contains 72 rangefinder sensors;
MuJoCo enables decorative **Range Finder** drawing in its standard camera
visualization settings. This rendering is not a real camera sensor phenomenon.

## Targeted change

`MuJoCoBackend.render_rgb()` now uses an `MjvOption` with
`mjVIS_RANGEFINDER=0`. It leaves other options at MuJoCo defaults. This only
hides decorative debugging rays in the RGB image; it **does not** change the
rangefinder model, sensor values, collision physics, EKF, A*, or control.
The same renderer function serves actual camera observations and offline
reconstructions.

## First run: compare BEFORE running full simulation

```bash
source .venv/bin/activate
export MUJOCO_GL=osmesa
export MPLBACKEND=Agg
python -m pytest -q
python scripts/diagnose_camera_overlay_v18.py \
  outputs/blocked_v16_bypass_seed0 \
  --config configs/mujoco_v16_blocked_bypass_trial.yaml
```

This reconstructs four level-chassis poses from the actual trajectory in a
new simulation object. It saves pairs of camera images (standard debug ray
visualization vs clean image) and reports detection outcomes. No prior
results are modified. Images are not original frames. If both fail, inspect
the `camera_overlay_comparison_*.png` before changing anything else.

## Follow-up if the camera check passes

```bash
python -m turtlebot_integrated.cli run \
  --backend mujoco --scenario blocked --seed 0 \
  --config configs/mujoco_v16_blocked_bypass_trial.yaml \
  --video --out outputs/blocked_v18_clean_camera_seed0
cat outputs/blocked_v18_clean_camera_seed0/summary.json
```

Inspect `measurements`, `accepted_camera_updates`, `strict_dock_position_pass`,
`strict_dock_pose_pass`, and `success`. A clean renderer does not guarantee
precision docking. Do not relax the tolerance or feed truth into the EKF.
