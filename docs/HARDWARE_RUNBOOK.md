# ROS 2 TurtleBot3 laboratory integration runbook

**Status: interface prepared, no physical-robot validation yet.** Reserve a supervised lab session. Treat this file as a checklist, not proof of compatibility.

## Must be confirmed in the lab

1. Exact TurtleBot3 variant (Burger, Waffle, or modified), wheel radius and axle separation; actual encoder names, signs and units from `ros2 topic echo /joint_states`.
2. ROS 2 distribution and TurtleBot3 firmware, transport network, `ROS_DOMAIN_ID`, robot namespace and topic names (`ros2 topic list -t`). In particular `/cmd_vel` may need a robot-specific namespace.
3. 2D LiDAR availability and sensor frame `laser -> base_link` from TF. A scan facing +x is assumed; if offset/rotated, correct scan angles/translations before motion.
4. RGB camera installed/working and `camera_optical_frame -> base_link` transform. Actual camera forward offset in code is **0.105 m, only a simulation assumption**; change this for hardware. Verify `/camera/camera_info` with `K` and distortion `D`.
5. Physical ArUco marker: `DICT_4X4_50`, ID **0**, 0.22 m *printed black square* size (not necessarily the full paper including margins). Survey its coordinate in your chosen `world` frame and set `marker_xy` appropriately. The `dock_pose`, `start_pose` and waypoint world coordinates must match that survey.
6. Check travel direction, yaw sign, wheel velocity feedback and static wheel/robot dimensions with the robot **raised off the ground** before closed-loop driving. Test small commanded velocity with a separate lab-authorized utility, not this autonomy node.
7. Battery charge, open floor, physical E-stop/remote kill, 2-metre exclusion zone, spotter, lab rules and safe operating speed.

## Launch / inspect without motion

```bash
source /opt/ros/<YOUR_ROS_DISTRO>/setup.bash
# In the project venv / matching ROS Python environment:
python -m turtlebot_integrated.ros2_bridge --config configs/default.yaml
# By default this publishes zero Twist; investigate expected joint names/topics.
```

Ensure `rclpy`, `cv_bridge`, `sensor_msgs` and `geometry_msgs` are installed for the matching ROS distribution. If joint names are different, use `--left-joint <name> --right-joint <name>`. Check `/camera/camera_info` K/D and ArUco detection at two surveyed distances/bearings, and confirm EKF position trend. If the library installed from pip shadows ROS system Python, use a compatible ROS python environment; do not mix incompatible Python versions.

## Only after written operator/supervisor approval

```bash
# Robot may move. Only when everyone and everything above are verified:
python -m turtlebot_integrated.ros2_bridge \
  --config configs/hardware_lab_checked.yaml --enable-motion
```

The suggested `configs/hardware_lab_checked.yaml` is **intentionally not shipped**; create it after measuring lab robot/environment. `--enable-motion` is never the default. Safety interlocks stop if encoder or LiDAR messages are stale >0.35 s; robot speed is further limited to 0.12 m/s and 0.8 rad/s. Within 0.75 m of the dock, motion stops unless the visual marker has been seen within 2 s. This is an additional software layer, **not a replacement for hardware E-stop, operator supervision, proven obstacle safety or a certified controller**.

## Lab test sequence (2-hour target)

| Time | Task | Pass criteria |
|---|---|---|
| 0–20 min | Power-up/ROS 2, inspect topics and frames | JointState, scan, image, camera info present |
| 20–40 min | Verify calibration and encoder sign, stationary marker bearing | Camera range/bearing and wheel motion signs match known references |
| 40–60 min | Dry-run estimate and logs | EKF updated, values sensible; commands remain zero |
| 60–85 min | Low-speed navigation with spotter | Stops/reacts to LiDAR obstacles; no unexpected motion |
| 85–105 min | ArUco dock approach with reduced speed | Marker detected; robot stops inside safe designated area |
| 105–120 min | Record ROS bags + hardware footage and stop/pack up | Results reproducible and robot safely disabled |

If basic checks fail, **stop at dry-run** and document the limitation. A hardware attempt does not compensate for absent MuJoCo assessment evidence.

## Required calibration artefacts

Save `ros2 topic list`, robot description and TF frame diagram, camera calibration/intrinsics, joint encoder sign/radius comparison, marker physical dimension and surveyed marker pose, run logs, initial/final robot pose, lab supervisor approval if applicable, and video proof. Do not disclose private machine passwords, robot network credentials or identifying data in the repository.
