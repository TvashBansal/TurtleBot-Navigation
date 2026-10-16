# v0.9 — Planner hold-state instrumentation and local waypoint progress

## Evidence
v0.8 MuJoCo nominal seed 7: 48 unit tests passed, final goal distance 2.9659 m, 90.4% near-zero forward commands, 88.4% of target points effectively at the estimated position, waypoint index 0 for all 1250 steps, no camera updates. MP4 generated but no docking. This is a **failure** trial and must never be labelled success.

## Changes (experimental)
- Preserve original v0.8 by default; only the new candidate YAML changes `nav_local_capture_radius_m` to 0.025 m (from 0.08 m).
- Controller can now advance slowly to a short, LiDAR-map-visible intermediate grid node that previously fell inside the controller's stop deadband.
- Navigator explicitly records whether A* had no route, a route had no onward line-of-sight, the available node is too near, or a usable path was followed. These labels are added to `trajectory.json` and `summary.json`.
- **Safety invariant:** no-route or no-visible-path still commands zero forward speed; LiDAR collision braking/slowing remains unchanged. No ground-truth pose is read by the navigator.

## Limitations
A short target is not necessarily the cause of the 88.4% v0.8 hold fraction. If v0.9 reports `no_astar_route` or `no_visible_onward_path` as dominant, changing the waypoint deadband cannot solve it. Investigate the MuJoCo LiDAR's physically measured rays and the inflated occupancy grid before changing safety thresholds. This patch has NOT been verified in MuJoCo.

## Codespaces after uploading each file to the correct GitHub folder
```
cd /workspaces/TurtleBot-Navigation
git pull --ff-only origin main
source .venv/bin/activate
export MUJOCO_GL=osmesa MPLBACKEND=Agg
python -m pytest -q
python -m turtlebot_integrated.cli run --backend mujoco --scenario nominal --seed 7 --config configs/mujoco_v09_short_probe.yaml --out outputs/nominal_v09_short_probe
# Review planner_states in summary.json. Only run the full 100s candidate if the robot is genuinely making progress.
# python -m turtlebot_integrated.cli run --backend mujoco --scenario nominal --seed 7 --config configs/mujoco_v09_local_progress.yaml --out outputs/nominal_v09_local_progress
```

Compare `summary.json` planner states, actual motion, collision, and camera updates to v0.8 before any further fix. Do not archive an unsuccessful trial as success.
