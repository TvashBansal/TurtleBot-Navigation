# v0.5 — Experimental yaw-rate sensor fusion (NOT a validated success)

## Reason for this change

The recorded v0.4 nominal MuJoCo trial ended with a 2.728 rad absolute heading estimate error and no camera detections. Its navigation was driven by estimated state (correct), but strong repeated turns caused odometry drift. Camera replay saw the marker only from a small range of viewing geometries. No successful MuJoCo docking run has been established.

This patch tests the hypothesis that an *actual angular-velocity sensor* stabilizes heading while wheel encoders provide translation and the camera provides range/bearing when detected. No EKF or control function reads MuJoCo ground-truth pose.

## What changed

- The MuJoCo base now has a 3-axis `gyro` sensor at `imu_site`, with configurable noise and bias. The observation packet transports `gyro_z_rad_s`.
- An **opt-in** EKF prediction blends encoder-derived yaw increment with integrated measured gyro yaw-rate; defaults **OFF** to preserve prior comparisons.
- The kinematic offline test double supports synthetic gyro samples, clearly labelled as test-double evidence.
- The ROS 2 adapter optionally subscribes to `/imu`; in a physical run using `use_gyro_yaw: true`, stale/missing IMU data blocks motion. Physical test and frame calibration remain outstanding. Verify real `/imu` frame, sign, time stamps and bias; keep `--enable-motion` off until reviewed in the lab.
- The experiment logs `gyro_z_rad_s`, `gyro_used`, and `camera_update_accepted` separately from `camera_seen`. This distinguishes raw detections from EKF accepted updates.

## Upload individual files

Extract the package on your laptop. Upload **each file** to its matching path inside the GitHub repository, then commit. Do not upload a ZIP to the repository root. In Codespaces: `git pull --ff-only origin main`.

## Test sequence

```bash
source .venv/bin/activate
export MUJOCO_GL=osmesa MPLBACKEND=Agg
python -m pytest -q
python scripts/diagnose_gyro_motion.py
```

Assess whether gyro-aided yaw RMS is better than encoder-only yaw RMS on the **same physically simulated motion**. If not, investigate IMU frame, sign, sample timing and calibration before any full run.

If it improves, run the closed-loop A/B trials in **separate output folders**:

```bash
python -m turtlebot_integrated.cli run --backend mujoco --scenario nominal --seed 7 --config configs/mujoco_odom_candidate.yaml --out outputs/mujoco_nominal_v05_wheel_only
python -m turtlebot_integrated.cli run --backend mujoco --scenario nominal --seed 7 --config configs/mujoco_gyro_candidate.yaml --out outputs/mujoco_nominal_v05_gyro
```

If the gyro mode reduces error but navigation still stalls while turning rapidly, compare the additional conservative-turn configuration:

```bash
python -m turtlebot_integrated.cli run --backend mujoco --scenario nominal --seed 7 --config configs/mujoco_gyro_conservative.yaml --out outputs/mujoco_nominal_v05_gyro_conservative
```

**Do not commit these to `results/` as successful trials unless the final summary actually reports success.** Failed trials may be archived and labelled *diagnostic/failure evidence* for an honest engineering discussion.

## Remaining limitations

1. Gyro drift/bias is not estimated online. This is yaw-*rate* support, not absolute pose correction; eventual visual measurements still matter.
2. Camera detector succeeds only under particular pose/scale conditions in v0.4. This patch does **not** claim to fix its off-axis/close-range failure.
3. The recorded 38 geometrically-in-FOV camera sampling instants are not a guarantee of detectability—obstacles, orientation, clipping, marker size and occlusion matter.
4. All physical TurtleBot3 hardware configurations (including ROS topic names and IMU transform) remain to be verified in the lab. This is an experimental adapter, not a completed physical hardware integration.
