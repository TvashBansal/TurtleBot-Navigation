TURTLEBOT-NAVIGATION — MuJoCo LiDAR self-hit correction + diagnostics

Files to place in the existing GitHub repository:
  turtlebot_integrated/mujoco_backend.py   [REPLACE existing file]
  scripts/diagnose_mujoco.py               [NEW]
  tests/test_mujoco_lidar.py               [NEW]

The previous LiDAR rays started on the robot's circumference at chassis
height. Some could intersect the front bumper, causing false short ranges.
The corrected rays originate at the simulated LiDAR head, above the chassis,
so no robot-radius offset is added to ranges.

NOTES:
- The correction is based on geometry/code inspection and diagnostic logs.
- The code has not been validated here against MuJoCo (unavailable here).
- The camera/ArUco issue is not yet fixed. The diagnostic script probes image
  flipping and prints wheel-encoder-to-truth discrepancies to find root causes.
- Do not label any run successful until summary.json reports success=true,
  actual camera detections are observed, and plots/video have been reviewed.

After copying files to their corresponding repository folders:
    cd /workspaces/TurtleBot-Navigation
    source .venv/bin/activate
    export MUJOCO_GL=osmesa MPLBACKEND=Agg
    python -m pytest -q
    python scripts/diagnose_mujoco.py

Then run a fresh nominal trial:
    python -m turtlebot_integrated.cli run --backend mujoco --scenario nominal \
      --seed 7 --out outputs/mujoco_nominal_lidarfix

Only commit code/tests when they pass. Keep diagnostic outputs in outputs/.
