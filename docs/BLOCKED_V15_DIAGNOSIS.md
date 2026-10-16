# v0.15: diagnose blocked trial using existing MuJoCo evidence

The v0.14 smoke suite passed nominal, noise, physical disturbance and camera-loss scenarios, but **blocked failed** with 2.4285 m final error (no reported collision). Do not replace this failure with kinematic evidence, suppress the collision checks or relax safety parameters.

## Why targeted diagnosis?

The additional obstacle in `blocked` occupies `x=[0.25, 0.60]`, `y=[-0.70,-0.34]` m. The second waypoint `(0.15,-0.30)` is only about **0.108 m** from its nearest corner, closer than configured map inflation `0.16` m. This suggests that an intermediate waypoint might be partially within the conservative inflated obstacle region, but it does **not** establish that this is the reason the robot failed. We must inspect which waypoint and planner state actually dominate the failed trial.

Run with existing suite data (NO additional physics run required):

```bash
cd /workspaces/TurtleBot-Navigation
source .venv/bin/activate
python -m pytest -q
python scripts/diagnose_blocked_v15.py outputs/robustness_v14_smoke
```

The script reads `blocked/seed_0/trajectory.json`, its summary/config snapshot, and nominal seed-0 reference, then writes `blocked_diagnosis_v15.json` at the root of the suite. Reports waypoint transitions, no-route/following/safety status, actual travel distance, first route/safety failures, final truth-vs-EKF disagreement, and minimum nominal waypoint clearance to the extra obstacle. Simulation ground truth is used *only for post-run diagnosis*, never navigation.

## Success flag versus strict docking tolerance

The experiment's `success` condition uses `position_error_m < dock_tolerance + 0.06` and heading `<0.25 rad`, provided the planner finishes and no collision is reported. Thus a summary success may have error **between 0.18 and 0.24 m** with the current configuration. Separately report `inside_configured_position_tolerance` (0.18 m) and the simulator's success flag. Do not alter tolerances silently or hide marginal trials.

No controller, LiDAR, mapping, physics geometry, or scoring is modified by this diagnostic. Subsequent changes require evidence from blocked seed 0 and must be rerun against all five scenarios.
