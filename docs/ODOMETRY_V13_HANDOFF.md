# v0.13 — Support-adjusted odometry candidate

## What v0.12 proved

The supported MuJoCo chassis completed A* route planning, reached the estimated docking region at t=21.52 s and had no reported collision, but scoring was `success=false`. The final estimated pose was [1.18335, -0.01761, 0.17474] while the physical pose was [1.01115, 0.01525, 0.17362]. Estimated position was 0.168 m from the dock; true position 0.339 m, outside the required 0.18 m nominal docking tolerance (and 0.24 m scored tolerance).

The EKF's old `odom_wheel_radius=0.0356` came from tests on the *unstable* (pitched) chassis. v0.12 materially changed wheel-floor contacts. The resulting approximate 6% displacement overestimate is compatible with a now-too-large odometry radius, but this is a **hypothesis**, not a validated calibration. Do not claim visual docking succeeded: only one camera measurement was accepted in v0.12.

## v0.13 controlled hypothesis

This package changes ONLY `odom_wheel_radius: 0.0330` from v0.12 while maintaining front support, gyro settings, A*, LQR, vision settings, safety limits, dock tolerance and maximum duration. It does **not** alter the simulator's physical wheel radius or the success scoring rule.

First run `python scripts/diagnose_supported_odometry_v13.py`; compare absolute encoder-distance error for both odometry radii against physical displacement measured offline. If the candidate reduces distance error without strong sideways drift, run a separate v0.13 30-second nominal closed-loop experiment. Both remain experimental until MuJoCo validates them.

```bash
source .venv/bin/activate
export MUJOCO_GL=osmesa MPLBACKEND=Agg
python -m pytest -q
python scripts/diagnose_supported_odometry_v13.py
python -m turtlebot_integrated.cli run --backend mujoco --scenario nominal --seed 7 --config configs/mujoco_v13_supported_odom_trial.yaml --video --out outputs/nominal_v13_supported_odom_trial
```

Compare `summary.json` with `outputs/nominal_v12_front_support_30s/summary.json`; review `trajectory.json` and MP4. Zero collisions, actual goal position error and camera update count are necessary evidence; `planner_reported_done=true` by itself is insufficient. The physical front support remains an approximation and cannot be claimed as hardware-equivalent without checking the real TurtleBot3 chassis geometry.
