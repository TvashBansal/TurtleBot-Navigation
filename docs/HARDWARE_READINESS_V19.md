# ROS 2 hardware readiness and simulation-to-reality limitations

**Status as of final v0.19 MuJoCo validation:** Software ROS 2 bridge prepared, **not tested on a physical TurtleBot3**. Hardware is optional for this assessment and requires the university's supervised induction and approval; simulation evidence stands independently.

## What exists

- `turtlebot_integrated/ros2_bridge.py`: ROS 2 subscriptions for `/joint_states`, `/scan`, `/imu`, `/camera/image_raw`, `/camera/camera_info`; publisher for `/cmd_vel`. These are **assumed topic names** that require validation for the actual robot and namespace.
- `turtlebot_integrated/hardware.py`: sensor adapter, staleness checks, zero-motion fallback, speed limits and visual-marker proximity interlock.
- The bridge defaults to **no physical motion**. Motion requires explicit `--enable-motion`, and even then assumes frames, joints and environment have been calibrated.
- Nominal simulation model: 0.033 m wheel radius, 0.160 m wheel separation; **odometry-effective separation** 0.198 m and supported-model odometry radius 0.033 m in validated v0.19 configuration. This disparity must be calibrated/reconciled for real hardware rather than copied blindly.
- Simulated camera offset 0.105 m, height 0.20 m, 0.22 m ArUco black marker (ID 0, DICT_4X4_50), `marker_xy=(1.82,0)` and dock `(1.35,0,0)` in the simulation world frame. None of these locations is automatically valid in the laboratory.

## Physical model limitation that must be disclosed

The v0.12+ MuJoCo model uses a **simplified low-friction passive front support** because the earlier freely pitching chassis produced nonphysical LiDAR floor hits. In the controlled comparison, 95th-percentile pitch fell from 29.92° to 1.50°. This improves the **simulation contact model** but does not establish geometric equivalence to Burger/Waffle hardware or its ball caster. Do not claim a real TurtleBot3 has the same contact support configuration, inertia, wheel slip, motor dynamics or camera alignment.

The camera simulation uses an ideal rendered marker; visual overlay artefacts were corrected in v0.18. Real optical calibration must still account for lens distortion, camera frame TF, image timing, noise, tag size, lighting and printed marker geometry. Real LiDAR scan angles and sensor mounting differ from the MuJoCo rangefinders.

## Mandatory onsite inspection before any robot motion

| Check | Expected evidence | Status |
|---|---|---|
| Supervisor induction and approval | Written lab authorization, trained spotter, reachable E-stop | **Not verified** |
| Robot model and power | TurtleBot variant, wheel radius, separation, battery, firmware | **Not verified** |
| ROS distribution/network | `ros2 topic list -t`, namespace and domain ID | **Not verified** |
| Wheel joint mapping | `/joint_states` left/right joint names, units, signs, wheel-lift check | **Not verified** |
| Scan geometry | `LaserScan` frame and TF to `base_link`, direction and angular coverage | **Not verified** |
| Optical calibration | `CameraInfo` K/D, camera_optical_frame TF, two surveyed tag ranges/bearings | **Not verified** |
| Initial world pose and marker survey | World frame, robot start, obstacle coordinates, fixed marker pose | **Not verified** |
| Velocity guard and emergency stop | Zero-command test, watchdog, supervised low-speed stop | **Not verified** |
| Recording and evidence | ROS bags, run settings, supervisor sign-off, error metrics | **Not verified** |

**Safe no-motion inspection** on an appropriately configured ROS 2 machine:

```bash
source /opt/ros/<installed-distro>/setup.bash
ros2 topic list -t
ros2 topic echo /joint_states --once
ros2 topic echo /scan --once
ros2 topic echo /camera/camera_info --once
# Dry run: sends zero velocity, NOT autonomous motion
python -m turtlebot_integrated.ros2_bridge --config configs/default.yaml
```

The example requires ROS 2 Python packages and cv_bridge from a compatible distro and interpreter; the Codespace's Python 3.11 venv may not be binary-compatible with those ROS packages. `configs/hardware_lab_checked.yaml` is **not supplied** until the laboratory measurements have been recorded. Do not attempt `--enable-motion` solely on the strength of successful MuJoCo validation. Follow `docs/HARDWARE_RUNBOOK.md` for the controlled two-hour plan.

## Optional HD extension, not a completion blocker

If an authorized, supervised lab trial becomes possible, first obtain no-motion evidence and verify TF/calibration; then test low-speed open-space movement and safety stops. Only attempt visual docking when both sensing and physical emergency-stop controls work. Record quantitative pose/heading errors and compare them against simulation. If the lab session cannot be completed, submit and discuss simulation-only evidence honestly. The rubric allows simulation-only work; it does not award automatic HD marks for an unvalidated hardware adapter.
