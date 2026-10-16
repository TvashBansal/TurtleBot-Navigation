# v0.14: physics-based robustness evidence

## Why this change is necessary

The original `disturbance` scenario specified `disturbance_at=26` s, even though the successful v0.13 nominal experiment finished in 22.4 s. More importantly, the MuJoCo backend never consumed `disturbance_at` or `wheel_slip`; the named disturbance trial would have been a normal physics run. That would not substantiate robustness against an external push. This update remedies *only* the external-disturbance scenario, while leaving the v0.13 successful nominal configuration and planner/controller/EKF unchanged.

## What the physics trial now does

- At **t=9.0 s** in the `disturbance` scenario only, apply an external **world-frame lateral force of 0.9 N and yaw torque of 0.025 N m for 0.4 s** to the robot body using MuJoCo's `xfrc_applied` (not a teleport).
- Explicitly clear this external wrench on subsequent steps. Track `disturbance_applied_steps` in each MuJoCo `summary.json`. The controller/EKF never sees the true state, wrench schedule or scenario name.
- Other scenarios receive no external force. `noise` has 2.5x sensor-noise multiplier; `camera_loss` has +35 percentage points dropout; `blocked` adds another obstacle. The `wheel_slip=0.045` setting remains **kinematic-test-double only**, not a verified MuJoCo slip manipulation. Do not claim MuJoCo wheel slip robustness from this suite.
- The v0.12 stabilising contact remains an experimental physical approximation of TurtleBot3. Check hardware geometry before claiming fidelity.

## Upload individual files

Replace `turtlebot_integrated/scenarios.py`, `turtlebot_integrated/mujoco_backend.py`, `turtlebot_integrated/experiment.py`. Add `tests/test_v14_physical_disturbance.py`, `scripts/review_robustness_v14.py` and this handoff document in `docs/`.

## Codespaces commands

```bash
cd /workspaces/TurtleBot-Navigation
git pull --ff-only origin main
source .venv/bin/activate
export MUJOCO_GL=osmesa MPLBACKEND=Agg
python -m pytest -q

# First one seed per scenario (five MuJoCo physics runs)
python -m turtlebot_integrated.cli suite --backend mujoco \
  --config configs/mujoco_v13_supported_odom_trial.yaml \
  --seeds 1 --out outputs/robustness_v14_smoke
python scripts/review_robustness_v14.py outputs/robustness_v14_smoke
cat outputs/robustness_v14_smoke/audit.json
```

If the smoke suite completes and audit passes, run **3 seeds per scenario** without video (15 MuJoCo trials):

```bash
python -m turtlebot_integrated.cli suite --backend mujoco \
  --config configs/mujoco_v13_supported_odom_trial.yaml \
  --seeds 3 --out outputs/robustness_v14_three_seeds
python scripts/review_robustness_v14.py outputs/robustness_v14_three_seeds
```

The suite uses seeds `0 .. seeds-1` for each scenario. The preserved v0.13 successful nominal case was seed **7**, and is *not* included in these suites. Do not overwrite that archived result. The suite stores each run's trajectory JSON, plots, metadata and its own config snapshot; it does **not** require rendering every video. For visual evidence, request individual `cli run --video` on selected representative cases later.

## Acceptance and reporting

The auditor validates a matching individual summary for each aggregate entry, consistent seeds across scenarios, and an actual external force in each `disturbance` trial. It writes `audit.json` and `audit.csv`. It does **not** claim success if the robot fails: report all failures, collisions, final errors, and camera observations. Run full suite only after reviewing the smoke results. Do not submit kinematic-only evidence as MuJoCo proof.

After inspecting and approving the final suite, archive with a unique name and verify hashes:

```bash
python scripts/archive_run.py outputs/robustness_v14_three_seeds --name robustness_v14_three_seeds
python scripts/verify_archive.py results
```
