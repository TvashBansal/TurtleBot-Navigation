# Reproducible validation record

**Date:** 2026-10-15. **Environment:** Python and NumPy/SciPy/OpenCV, no installed `mujoco` Python package and no accessible package index during development; ROS 2 (`rclpy`) not installed. All tests here are **offline software checks only**.

| Component | Tested here? | Evidence / limitation |
|---|---|---|
| SE(2) EKF numerical operations and camera outlier gating | Yes | `tests/test_ekf.py` |
| Model local controllability/observability numerical ranks | Yes | `tests/test_model.py` and `scripts/analyse_model.py` |
| LQR nominal driving and heading turn response | Yes | `tests/test_control_map.py` |
| LiDAR occupancy and path planning | Yes | `tests/test_control_map.py` |
| ROS-neutral hardware safety interlocks | Yes | `tests/test_safety.py`; **not actual ROS 2** |
| MJCF string includes wheel actuators, encoder, LiDAR, camera | Yes (text construction only) | `tests/test_scene.py`; **XML not compiled** |
| End-to-end low-fidelity test double | Yes | `tests/test_smoke.py` and run scenario examples below |
| MuJoCo model compile/contact/renderer | **No** | `mujoco` unavailable; must validate prior to submission |
| Rendered-camera ArUco pose and marker visibility in MuJoCo | **No** | Camera-texture pipeline unverified; visually inspect and calibrate |
| ROS 2 subscription/publishing with physical TurtleBot3 | **No** | Interface only; validate topic/TF/camera in the lab |
| Physics-based robustness/HD-level evidence | **No** | Do not submit test-double metrics as assessment results |

## Kinematic test-double sample, seed 7

| Scenario | Reached docking target | Synthetic collision | Final position error (m) | Duration (sim s) |
|---|---|---|---:|---:|
| nominal | yes | no | 0.1164 | 15.04 |
| noise | yes | no | 0.1509 | 15.52 |
| blocked | yes | no | 0.1627 | 15.28 |
| disturbance | yes | no | 0.1560 | 15.52 |
| camera_loss | yes | no | 0.1155 | 15.04 |

This is **one seed per scenario**, not a statistically meaningful performance study, and it uses kinematic ray casting and synthetic marker sensing. The “disturbance” is a small pose displacement in the test double. Only real MuJoCo repeats justify physics-simulation claims.

## Outstanding integration checks

- Build MJCF via `MuJoCoBackend(...)`, check successful compile and model inertial/contact geometry.
- Check wheel velocity sign and straight-line speed against encoder-inferred displacement, ensure no slipping/falling/penetration.
- Confirm rangefinder rays are horizontal and physically detect obstacles without self intersections.
- View rendered front-camera frames and confirm ArUco marker is textured on correct box face with detectMarkers success; verify RGB orientation and PnP range/bearing values at known distances.
- Ensure all five MuJoCo scenarios complete; if not, diagnose per-scenario cause and tune model, noise and controller, documenting changes.
- Run 5+ seeds/scenario; quantify RMSE, successful dock fraction, collision fraction, tracking errors, NIS/2σ and control effort.
- Capture real MuJoCo video using the overhead/camera/diagnostics capture showing robot, estimated state, covariance alongside truth **for evaluation**, and disturbance; prepare 5-min demonstration.
- Validate robot ROS 2 topics, calibration and real sensors; record hardware trial only if available/safe.
