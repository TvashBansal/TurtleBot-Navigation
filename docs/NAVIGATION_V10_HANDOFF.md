# v0.10 — bounded intermediate-goal A* fallback

## Evidence that led to this change

The v0.9 MuJoCo A* probe (seed 7, 30 seconds) reported 145 `goal_in_inflated_obstacle` A* call failures, 1 disconnected-free-space failure, and 3 successful paths. A* often failed because the **exact intermediate waypoint cell** was flagged as occupied in the *LiDAR-built inflated occupancy map*. These diagnostic masks are not simulator ground truth. The first no-route time was 19.68 seconds.

## Change (limited to intermediate waypoints)

- v0.10 adds `OccupancyMap.plan_to_goal_region()`. It uses the same inflated LiDAR occupancy mask, robot-start exception, 8-neighbour A*, and diagonal corner-cut restriction as normal A*.
- A fallback route is considered **only** when the original A* route is empty, the destination is an *intermediate* waypoint, its nominal goal cell is marked occupied by the inflated map, and `nav_intermediate_goal_region_fallback` is enabled.
- It searches for a *reachable free cell* whose centre is strictly within the original 0.24m intermediate-waypoint capture region. It does not clear the blocked goal cell, cross occupied cells, or enlarge final docking tolerance.
- A genuine disconnected map, a fully blocked waypoint region, and a blocked final docking goal still cause a safe hold.
- Stale paths are discarded when a replan fails. LiDAR reactive braking and estimated-state-only planning remain in place.
- Default `nav_intermediate_goal_region_fallback: false` keeps the old *endpoint selection* behaviour; only the candidate YAML enables the new fallback. A separate safety fix discards invalidated old paths after a failed replan, so for byte-for-byte reproduction of earlier runs check out their recorded Git commit.

## Upload individually (not ZIP)

Replace: `turtlebot_integrated/config.py`, `turtlebot_integrated/mapping.py`, `turtlebot_integrated/navigation.py`.

Add: `tests/test_v010_goal_region.py`, `configs/mujoco_v10_goal_region_probe.yaml`, `docs/NAVIGATION_V10_HANDOFF.md`.

## Validate in GitHub Codespaces

```bash
cd /workspaces/TurtleBot-Navigation
git pull --ff-only origin main
source .venv/bin/activate
export MUJOCO_GL=osmesa
export MPLBACKEND=Agg
python -m pytest -q
python -m turtlebot_integrated.cli run \
  --backend mujoco --scenario nominal --seed 7 \
  --config configs/mujoco_v10_goal_region_probe.yaml \
  --out outputs/nominal_v10_goal_region_30s
cat outputs/nominal_v10_goal_region_30s/summary.json
```

Compare the existing `outputs/nominal_v09_short_probe/summary.json` with this probe: `planner_no_route_fraction`, `planner_states`, `position_error_m`, `collision`, `measurements`, and `accepted_camera_updates`. Specifically look for `goal_region_path` as an indicator that the fallback was used; it is not by itself evidence of completion.

## Limitations and next decision

These code changes have only been tested with automated Python tests and the lightweight kinematic backend here. **No MuJoCo success is claimed.** The pronounced discrepancy between LiDAR ranges and simplified obstacle geometry remains under investigation. If the fallback cannot find a safe path and no-route remains high, stop adjusting A* endpoints and investigate actual ray geometry, sensor pose, and dynamic robot attitude in MuJoCo. Do not lower obstacle inflation or disable emergency braking to force success.
