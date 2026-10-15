# v0.8 safe lookahead experiment — handoff

## Evidence motivating change
v0.7 nominal MuJoCo failed; controller stops ~0.8%, LiDAR brake ~30.4%, slow ~68.3%, waypoint 1 consumed 77.0% of the run. An offline audit reported ~39.5% of direct target segments came near obstacle buffers and ~31.4% of LiDAR distances were >0.15m shorter than a simplified 2D geometry estimate. **These are symptoms, not proof that the sensor is wrong.**

## Changes in this update
- `OccupancyMap.segment_clear` checks whether a local target is visible through the **LiDAR-built, inflated occupancy map**, sampled at quarter-cell intervals.
- `Navigator.tick` selects only forward targets on the computed A* path whose direct line is free in that map; it no longer blindly chooses the first path point beyond the lookahead radius. If no target is visible, or if A* fails to provide a route, it requests replanning and holds position.
- Emergency braking and slow-down thresholds are unchanged. No simulator truth is used by estimation or control.

## Limits / follow-up
- Local visibility uses a grid reconstructed from potentially incorrect LiDAR measurements. This **does not fix** the 31.4% clearance disagreement or guarantee that the robot finishes the task.
- The original sensor/physics problem may include tilt, contact geometry, self-intersections, and noise. Do not filter near readings merely because they disagree with the ground-truth diagnostic.
- **MuJoCo v0.8 has not been validated.** Keep this change experimental until repeatable closed-loop success and collision avoidance are demonstrated.

## Codespaces commands after committing individual files on GitHub
```bash
cd /workspaces/TurtleBot-Navigation
git pull --ff-only origin main
source .venv/bin/activate
export MUJOCO_GL=osmesa
export MPLBACKEND=Agg
python -m pytest -q
python -m turtlebot_integrated.cli run --backend mujoco --scenario nominal --seed 7 --config configs/mujoco_v07_steering.yaml --out outputs/nominal_v08_safe_lookahead
```
Do **not** commit a failure as successful evidence. Do not modify sensor safety thresholds as a workaround. Save `summary.json`, plots, and logs; compare to v0.7.
