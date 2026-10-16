# v0.11 intermediate-waypoint region fallback correction

## Evidence and rationale

The v0.10 MuJoCo diagnostic reported **349 failed exact A* calls and zero
`region_fallback_calls`**, even though `nav_intermediate_goal_region_fallback`
was enabled. Previously, the navigator invoked the region search only when the
*exact waypoint cell* was occupied. A* can also fail when that cell is free
but isolated behind inflated occupied cells; the fallback was never tried.

## Change

`turtlebot_integrated/navigation.py` now attempts the existing
`OccupancyMap.plan_to_goal_region()` **for intermediate waypoints only**
whenever exact A* fails and the opt-in flag is enabled.

The region planner independently checks obstacle inflation, reachable cells,
no diagonal corner cutting, and that the destination cell centre lies within
the configured intermediate-waypoint capture radius. When there is no valid
route, it returns an empty path, and the navigator holds safely.

The final docking goal is not approximated. Nothing changes the reactive
LiDAR stop/slow thresholds, sensor model, MuJoCo physics, EKF, or LQR.

## Apply to GitHub

Replace `turtlebot_integrated/navigation.py`, add
`tests/test_v011_region_fallback.py`, and add this file in `docs/`.
Commit those **individual** files to the private repository, then pull main
into the Codespace. No ZIP upload is necessary.

## Run (MuJoCo Codespace)

```bash
cd /workspaces/TurtleBot-Navigation
git pull --ff-only origin main
source .venv/bin/activate
export MUJOCO_GL=osmesa
export MPLBACKEND=Agg
python -m pytest -q
python scripts/diagnose_v10_goal_region.py \
  --out outputs/diagnostics/v11_goal_region_probe
```

This replay uses the already-defined 30s v0.10 config and shows separate
`region_fallback_calls`, `region_success`, and any failures. **Do not** assume
that the fix works in MuJoCo from the unit tests alone.

If the fallback is called but has no reachable candidate, investigate LiDAR
map inflation / goal region free-space connectivity. If it succeeds, compare
planner states and goal error against v0.10 before running longer trials.

Do not archive a "successful" MuJoCo trial until the robot actually navigates
and visually docks; debug outputs can be archived as failed evidence with
honest labels.
