# v0.19: Conservative estimated-pose docking completion

## Evidence motivating this experiment

The verified v0.18 MuJoCo 5-scenario × 3-seed suite reported simulator success on 15/15 trials, no reported collisions, but only 7/15 passed the independently audited **strict** final-pose tolerances (position < 0.18 m, heading < 0.18 rad). A per-trial audit identified **seven position-only failures** (blocked: 3, disturbance: 2, camera loss: 2) and **one heading-only failure** (noise seed 2). Final EKF XY errors were around 0.011–0.031 m, suggesting the estimated-pose `goal_reached` test needs a guard margin. The experiment does not claim guaranteed improvements.

## What changes

This version adds two **optional, zero-by-default** Config fields:

- `nav_dock_position_margin_m` (v0.19 trial: 0.04 m)
- `nav_dock_heading_margin_rad` (v0.19 trial: 0.035 rad)

On the **final waypoint only**, the navigator considers the docking goal complete when the **EKF estimate** is closer than `dock_tolerance - nav_dock_position_margin_m` and its estimated absolute yaw error is less than `0.18 - nav_dock_heading_margin_rad`. In the provided trial configuration that means < 0.14 m and < 0.145 rad on estimated pose.

This is not a change to the final dock location, the truth-based simulator scoring (`position_error_m < dock_tolerance` for strict position; `< 0.18 rad` for strict heading), camera, ROS2 bridge, collision braking, obstacle inflation, EKF, or LQR tracking law. The **original v0.18 config** remains a control and default margins of zero preserve older behaviour.

The existing LQR enters rotate-in-place mode when distance < 0.13 m. The candidate's 0.14 m completion radius is above that threshold, so terminal drive can still complete before rotation-only mode.

## Local checks

- Python tests: **93 passed, 2 skipped** (MuJoCo not installed in the local test environment).
- Kinematic synthetic nominal trial, seed 0: success; strict position and heading pass, 0.131 m position error. **Not MuJoCo evidence**.
- Kinematic synthetic 5-scenario smoke: 4/5 reported success; blocked test-double failure remains, so the physical MuJoCo blocked scenario must be explicitly verified. **No claims about MuJoCo performance until rerun in Codespaces**.

## Codespaces single-trial verification

From the repo root after uploading files and committing:

```bash
cd /workspaces/TurtleBot-Navigation
git pull --ff-only origin main
source .venv/bin/activate
export MUJOCO_GL=osmesa
export MPLBACKEND=Agg
python -m pytest -q

python -m turtlebot_integrated.cli run --backend mujoco --scenario blocked --seed 0 \
  --config configs/mujoco_v19_precision_trial.yaml \
  --out outputs/blocked_v19_precision_seed0
cat outputs/blocked_v19_precision_seed0/summary.json
```

If first trial passes, test `noise` seed 2 and `camera_loss` seed 0 with the SAME trial config and distinct output folders, then run the full 15-trial suite. Do not overwrite prior v0.18 outputs or archives.

### Acceptance criteria

- `strict_dock_position_pass` and `strict_dock_pose_pass` true for the initial blocked trial, without new collisions or waypoint safety violations.
- No premature `goal_reached` without correctly estimated position and heading.
- `accepted_camera_updates` remains positive in the blocked MuJoCo trial.
- Confirm overall 15-trial results and compare to v0.18 **7/15 strict pose baseline**, rather than merely inspecting the simulator's more generous `success` flag.
- **Do not** update the project's overall readiness percentage until MuJoCo verifies improvements.

## Safety and scientific limits

All terminal checks use the EKF estimate. Simulator truth is used only for offline scoring. Margin values are empirically motivated by the observed residual error; they are not certified bounds and must be validated under other seeds and physical conditions. In particular, 15 test runs do not establish universal hardware reliability or that a passive MuJoCo support sphere matches the physical TurtleBot3 platform.
