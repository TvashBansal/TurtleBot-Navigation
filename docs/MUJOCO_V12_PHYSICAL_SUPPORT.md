# v0.12 – MuJoCo body pitch and false LiDAR floor hits

## Why this candidate exists

On the v0.11 nominal probe, the diagnostic reported approximately 29.62 degrees of persistent body pitch, 367 of 375 near-front scans with short ranges, and likely floor intersections among all 367 short-range scans. The laser rays are rigidly attached to the base; significant forward pitching can make nominally horizontal rays intersect the floor. If those hits are projected into a 2-D occupancy map, false obstacles can disconnect A*.

## Change and limitations

- Added `mujoco_front_support_enabled` (default `false`) to the configuration.
- When enabled, MuJoCo adds a **passive, low-friction spherical front support** near the robot footprint, to constrain unrealistic nose-diving of the simplified floating robot model.
- It is **not part of the nominal TurtleBot3 CAD geometry**, and must be acknowledged as a simulation-model change. It is meant to test the hypothesised chassis-contact problem, not to conceal it.
- Wheels, wheel actuators, camera, LiDAR, EKF, A*, LQR, the reactive collision brake and success metric remain unchanged. The existing v0.11 baseline remains reproducible because the support is OFF by default.
- Do not change LiDAR readings or feed body orientation / simulator ground truth into the navigation pipeline. True pitch and predicted floor intersections are used only for offline diagnosis.

## Verification order (GitHub Codespaces, Debian, Python 3.11)

```bash
cd /workspaces/TurtleBot-Navigation
git pull --ff-only origin main
source .venv/bin/activate
export MUJOCO_GL=osmesa MPLBACKEND=Agg
python -m pytest -q
python scripts/diagnose_v12_chassis_support.py
```

Look at `outputs/diagnostics/v12_chassis_support/pitch_and_lidar.png` and `comparison.json`. Success means **substantially lower pitch and fewer floor-intersection-like short ranges than the original**, without loss of motion. It does *not* mean docking succeeded.

Only if that physics comparison supports the change, run a 30-second nominal closed-loop test:

```bash
python -m turtlebot_integrated.cli run \
  --backend mujoco --scenario nominal --seed 7 \
  --config configs/mujoco_v12_front_support_probe.yaml \
  --out outputs/nominal_v12_front_support_30s
```

Compare the v0.11 30-second trial: 34.7% no-route in v0.9, 93.1% in v0.10; v0.11 had 349/375 no-route and no docking. At 30 seconds do not require completed docking; focus on path progress and low false obstacle ranges.

If pitch stays ~30 degrees or floor-hit-like ranges persist, **do not weaken LiDAR safety or adopt the altered model**. Investigate MuJoCo body contacts, wheel/caster geometry, and sensor mounting from the recorded diagnostic.

The test does not establish physical TurtleBot3 hardware validity. Before hardware transfer, check the actual base geometry and sensor-frame transforms separately.
