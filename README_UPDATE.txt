TurtleBot-Navigation v0.5 — Experimental IMU yaw-rate support
==============================================================

This update is NOT proof of successful closed-loop MuJoCo navigation.
The prior v0.4 run failed; this is an opt-in test of the IMU hypothesis.

IMPORTANT: The ZIP is just for downloading all files to your laptop.
Extract it locally and upload each individual file into its EXACT matching
GitHub folder, then commit them to main. Do NOT upload the ZIP to GitHub.
Do NOT overwrite the whole repository.

REPLACE these existing files:
  turtlebot_integrated/config.py
  turtlebot_integrated/types.py
  turtlebot_integrated/ekf.py
  turtlebot_integrated/experiment.py
  turtlebot_integrated/mujoco_backend.py
  turtlebot_integrated/kinematic.py
  turtlebot_integrated/hardware.py
  turtlebot_integrated/ros2_bridge.py

ADD these new files:
  configs/mujoco_gyro_candidate.yaml
  configs/mujoco_gyro_conservative.yaml
  scripts/diagnose_gyro_motion.py
  tests/test_gyro_fusion.py
  docs/GYRO_V05_README.md

Codespaces, after GitHub commit:
  cd /workspaces/TurtleBot-Navigation
  git status --short
  git pull --ff-only origin main
  source .venv/bin/activate
  export MUJOCO_GL=osmesa MPLBACKEND=Agg
  python -m pytest -q
  python scripts/diagnose_gyro_motion.py

If diagnostic shows gyro aiding yaw accuracy, then follow the A/B closed-loop
commands in docs/GYRO_V05_README.md. Keep each run in outputs/ until verified.

Design boundaries:
  - MuJoCo `gyro` measures yaw rate, not simulator truth pose.
  - EKF prediction is still based on wheel encoders + optionally IMU yaw-rate.
  - EKF camera update still fuses ArUco range/bearing when available.
  - Old runs remain comparable by keeping gyro fusion opt-in and OFF by default.
  - Hardware ROS2 /imu interface is not calibrated or physically validated.
