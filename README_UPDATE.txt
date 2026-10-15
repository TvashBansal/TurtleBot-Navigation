TurtleBot-Navigation - camera UV correction, v0.3

WHY:
The real MuJoCo camera probe detected the ArUco tag only after reflecting
its RGB frame horizontally or vertically. Mirrored default MJCF box-face
texture coordinates are the likely reason. The correction reflects only the
image used as the MJCF marker texture, leaving the rendered camera frame
and its optical bearing coordinates unchanged. It does NOT modify your
original assets/aruco_0.png or any existing saved experiment.

FILES: upload the following, preserving folder paths:
  turtlebot_integrated/mujoco_backend.py  (REPLACE)
  scripts/diagnose_camera_uv.py           (NEW)
  tests/test_mujoco_camera_uv.py          (NEW)

IN CODESPACES:
  cd /workspaces/TurtleBot-Navigation
  git pull --ff-only origin main
  source .venv/bin/activate
  export MUJOCO_GL=osmesa MPLBACKEND=Agg
  python -m pytest -q
  python scripts/diagnose_camera_uv.py

Inspect outputs/diagnostics/camera_uv/{center,marker_left,marker_right}.png
and outputs/diagnostics/camera_uv/camera_uv_report.json. Send the diagnostic
console output, especially camera distances, bearings and EKF acceptance.
If center is detected and geometry seems physically plausible, rerun
nominal MuJoCo without video first:
  python -m turtlebot_integrated.cli run --backend mujoco --scenario nominal --seed 7 --out outputs/mujoco_nominal_camera_fix

IMPORTANT:
* This is a hypothesis-driven correction that could not be physics-tested
  in the package creation environment (MuJoCo unavailable there).
* The initial 2-second turning trial showed encoder-predicted yaw 1.0808
  rad versus physics yaw 0.9429 rad (~14.6% discrepancy). One data point
  does not justify modifying EKF wheel geometry. Evaluate repeat trials and
  camera innovation before calibrating effective odometry.
* Keep unsuccessful trials under outputs/ until reviewed. The controller
  must NEVER read MuJoCo truth to control the robot.
