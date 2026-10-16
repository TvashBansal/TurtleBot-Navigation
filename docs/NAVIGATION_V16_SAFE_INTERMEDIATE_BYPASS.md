# v0.16 — Obstacle-aware intermediate waypoint bypass (experimental)

## Evidence and issue

The actual v0.14 MuJoCo blocked trial fails at intermediate waypoint index 1:
298/375 steps show `no_astar_route`, 13 show `goal_region_path`, and
final true docking error is 2.4285 m. Estimated and actual positions agree
near the stalled location, which rules out a major EKF discrepancy in this trial.
The extra obstacle comes within 0.1077 m of waypoint 1 while the safety-map
inflation is 0.16 m. The intermediate waypoint is not itself the dock.

## Change

Use `configs/mujoco_v16_blocked_bypass_trial.yaml` to enable two opt-in settings:

- `nav_bypass_unreachable_intermediate: true`
- `nav_bypass_no_route_after_s: 1.0`

After both the exact waypoint A* route and the existing bounded goal-region A*
route fail continuously for >= 1 s, the navigator MAY advance to the **next
intermediate waypoint only**, but ONLY if normal A* (or the same conservative
intermediate goal-region A*) returns a valid route to that destination.
The controller still receives only estimated pose and LiDAR-derived map data.
It does not read MuJoCo obstacle geometry or ground-truth pose.

No bypass is attempted if the following waypoint is the FINAL docking waypoint.
No direct path is fabricated across the inflated occupancy map. If every
candidate fails, the robot remains stopped; normal LiDAR reactive safety
braking is unchanged. The baseline v0.13 config remains OFF for this feature.

The new summary logs `bypassed_intermediate_waypoints` to distinguish a
planned route from one that had to skip an obstructed navigation hint.
The summary also exposes `strict_dock_position_pass` and
`strict_dock_pose_pass` separately from the historical scoring `success` flag.
Historical scoring uses tolerance+0.06 m and heading <0.25 rad, whereas
strict pose requires actual position error <0.18 m and heading error <0.18 rad.

## Validation (Codespaces)

```bash
cd /workspaces/TurtleBot-Navigation
git pull --ff-only origin main
source .venv/bin/activate
export MUJOCO_GL=osmesa MPLBACKEND=Agg
python -m pytest -q
python -m turtlebot_integrated.cli run \
  --backend mujoco --scenario blocked --seed 0 \
  --config configs/mujoco_v16_blocked_bypass_trial.yaml \
  --out outputs/blocked_v16_bypass_seed0 --video
cat outputs/blocked_v16_bypass_seed0/summary.json
```

If blocked succeeds and has no collisions, run the five-scenario suite:

```bash
python -m turtlebot_integrated.cli suite --backend mujoco \
  --config configs/mujoco_v16_blocked_bypass_trial.yaml \
  --seeds 1 --out outputs/robustness_v16_smoke
python scripts/review_robustness_v16.py outputs/robustness_v16_smoke
```

Evaluate ALL five scenarios, strict docking passes, actual physical disturbance,
A* no-route fractions and whether the bypass occurred. Do not claim success
before the actual MuJoCo results arrive. If no safe bypass A* route is found,
inspect the LiDAR occupancy connected component and safe replanning rather
than decreasing inflation or weakening physical collision protection.

## Local checks

The offline Python regression suite passed (76 tests, 2 MuJoCo-only tests
skipped). The additional kinematic test-double blocked trial did **not**
complete and did **not** trigger a bypass because it encountered different
path visibility failures; this does not predict the actual MuJoCo outcome.
Confirm whether `bypassed_intermediate_waypoints` is nonempty and the actual
MuJoCo blocked trial improves before accepting v0.16.

## Reproducibility limitation

The passive MuJoCo front support stabilises the simplified chassis but is not
verified hardware-equivalent TurtleBot3 geometry. The ROS2 hardware adapter
still requires inspection/testing against the actual TurtleBot3.
