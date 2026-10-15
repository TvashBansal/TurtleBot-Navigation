# TurtleBot-Navigation v0.4 — odometry calibration experiment

## Diagnosis

The v0.3 trial diverged: no camera updates, growing pose error, and excessive turning. A 2-second isolated straight probe gave encoder distance 0.2218 m versus physical displacement about 0.2395 m. An isolated turn gave encoder heading 1.0808 rad versus physical heading 0.9429 rad. These are insufficient to assume a final calibration but justify testing separate *effective odometry parameters*.

## Changes

- `Config` gains `odom_wheel_radius` and `odom_wheel_separation`. Defaults are `None` and preserve previous behavior.
- `PoseEKF.predict` uses odometry-only parameters when supplied. MuJoCo actuation still uses nominal `wheel_radius` and `wheel_separation` — no modification to the simulated physics or motor commands.
- Candidate `.yaml` is inferred from the previous physical checks: radius ~0.0356 m and effective axle separation ~0.1980 m. Values are provisional.
- `scripts/diagnose_calibrated_odometry.py` tests straight, in-place, and curved commands for nominal versus candidate settings using simulator truth **offline for calibration evaluation only**.

## Run in Codespaces

```bash
cd /workspaces/TurtleBot-Navigation
source .venv/bin/activate
export MUJOCO_GL=osmesa MPLBACKEND=Agg
python -m pytest -q
python scripts/diagnose_calibrated_odometry.py
python -m turtlebot_integrated.cli run --backend mujoco --scenario nominal --seed 7 --config configs/mujoco_odom_candidate.yaml --out outputs/mujoco_nominal_v04_candidate
```

Compare v0.3 and v0.4 `summary.json` plus estimated-vs-true plots. If candidate improves short probes but fails in navigation, next diagnose steering sign, excessive turn commands, camera proximity/detection and map update feedback. Do not claim docking success merely because the unit tests pass.

No simulator truth is supplied to online EKF, planner, LQR, or controller. Do not upload unscreened results as successful evidence.
