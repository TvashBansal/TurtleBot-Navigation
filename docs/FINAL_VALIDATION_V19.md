# Final technical validation — MuJoCo Track A (v0.19)

**Evidence provenance:** results under `results/mujoco/robustness_v19_strict_15of15/` (15 runs), `aggregate.json`, `audit_v16.json`, `audit_v16.csv`, `suite_metadata.json`, `config_snapshot.json`, per-run `summary.json`, `trajectory.json` and plots, plus integrity manifest. The results below were obtained by the student in GitHub Codespaces on 16 October 2026; this report analyses those user-run outputs, not an independently executed physical robot.

**Important:** The audit script is called `review_robustness_v16.py` because it was initially written for v0.16. The archived trial configuration is **v0.19**, including the conservative dock-stop margins. The archival `manifest.json` protects files from accidental modification, but integrity checking is not itself proof of model fidelity.

## Test protocol and acceptance criteria

- Simulator: MuJoCo 3D contact physics, simulated wheel encoders, 2D rangefinder LiDAR, rendered RGB camera, optional yaw-rate fusion.
- Fixed model configuration: `configs/mujoco_v19_precision_trial.yaml`; **scenario** varied among nominal, noise, blocked, disturbance and camera_loss, with **seeds 0, 1 and 2** each.
- Planner/controller use only EKF-estimated state and sensor-derived occupancy; MuJoCo ground truth is read by the runner for evaluation and reports.
- The simulator's `success` flag permits a **position error up to 0.24 m** (configured 0.18 m plus scoring allowance 0.06 m); it is **not** the same as the strict assessment-style criteria. The separate `strict_dock_pose_pass` records both **true position error below 0.18 m** and **true heading error below 0.18 rad**. This 0.18 rad heading threshold is a project reporting criterion, not necessarily a separately stated rubric threshold.
- `nav_dock_position_margin_m: 0.04` and `nav_dock_heading_margin_rad: 0.035` change **when estimated-state navigation stops** (estimated error below 0.14 m and 0.145 rad). They do **not** change the truth-based strict thresholds.
- Collision means the simulator's implemented contact/collision detection reported a collision, **not** a guarantee that all physically unsafe contacts or near misses were eliminated.
- A 15-run success rate describes only these five conditions × three seeds, not a statistically justified universal success probability.

## Final metrics

| Scenario | Simulator | Strict true-pose docking | Mean true docking XY error (m) | Reported collisions | Notes |
|---|---:|---:|---:|---:|---|
| Nominal | 3/3 | 3/3 | 0.1420 | 0 | Complete route and dock |
| Noise | 3/3 | 3/3 | 0.1369 | 0 | Increased sensor noise |
| Blocked | 3/3 | 3/3 | 0.1539 | 0 | Waypoint index 1 safely bypassed in each run |
| Disturbance | 3/3 | 3/3 | 0.1409 | 0 | Physical external wrench applied in all 3 |
| Camera loss | 3/3 | 3/3 | 0.1421 | 0 | Reduced marker availability |
| **Overall** | **15/15** | **15/15** | **0.14316** | **0** | Mean combined XY estimation RMSE: **0.03806 m** |

**Validation progression (same five scenarios, three seeds):** v0.18 had 15/15 simulator successes but **7/15 strict-pose passes**, with mean docking XY error **0.16657 m**. v0.19 maintained 15/15 simulator successes and achieved **15/15 strict-pose passes**, with mean XY error **0.14316 m**. The mean reduced by 0.02341 m, approximately **14.1%**. Across these runs this supports the claim that conservative estimate-side stopping improved accuracy; it is not a general proof of controller optimality.

## Engineering explanation and model limitations

### Part A: Model and control/observation properties

The planar model `q=[x,y,θ]ᵀ` satisfies `ẋ=v cos θ`, `ẏ=v sin θ`, `θ̇=ω`; wheel radius 0.033 m and track width 0.160 m define nominal differential-drive command conversion. These are planar kinematics; MuJoCo simulates 3D rigid-body and wheel contacts. The local tracking state is `e=[e_x,e_y,e_θ]ᵀ`; the discrete-time LQR feedback uses the linearised dynamics of the body-frame tracking error. The locally linearised controllability rank is 3 while translating at nonzero reference speed and 2 at stationary reference speed. Camera range+bearing yields two scalar observations from one known marker. The static measurement Jacobian has rank 2; propagation during motion can give a full-rank local stacked observability matrix (see `scripts/analyse_model.py`). This should not be represented as global pose observability with no measurements.

### Part B: Sensing, noise and coordinate frames

RGB observation uses OpenCV ArUco ID 0, 4×4 dictionary and PnP with a simulated 0.22 m black square. `world`, `base_link` (x forward, y left, z up), camera optical (x right, y down, z forward) and the fixed marker frame are distinguished. The simulated camera origin is 0.105 m ahead of the base and 0.200 m above the ground. LiDAR is converted into a local occupancy grid and inflated by 0.16 m. These are configured simulation dimensions; hardware requires direct calibration.

A consequential rendering defect was diagnosed: MuJoCo's **visualised rangefinder rays** crossed the marker in the camera image and impaired ArUco detection. Turning off the *display of those rays* while retaining functional LiDAR improved a blocked run from **0 to 31 accepted camera measurements** (v0.16 to v0.18), with true docking error falling from 0.3561 m to 0.1953 m. These measurements alone do not establish experimental optical-camera accuracy in a laboratory.

### Part C: EKF design and consistency limitations

Wheel increments drive prediction; optional simulated gyro yaw-rate helps heading; ArUco-derived range/bearing correct position and heading with innovation gating and Joseph-form covariance update. The final v0.19 suite's mean combined XY estimation RMSE is **0.03806 m**. This is a per-run RMSE summary, not a full uncertainty-calibration/NEES study. **No completed NIS consistency sweep, Q/R parameter sweep, or hardware noise-identification exercise is claimed.** The code includes an NIS gate, but a gate is not evidence that covariance is statistically consistent under all conditions. A more rigorous extension would compare observed innovations against predicted covariance and plot NIS distributions against chi-square thresholds.

### Part D: LQR and safe completion

The controller applies estimated-state LQR/path-tracking commands to successive planned targets, subject to speed and reactive LiDAR proximity restrictions. The revised endpoint policy requires estimated XY error <0.14 m and estimated heading error <0.145 rad. The stricter stop margin is motivated by residual EKF error observed near the dock (typically around 0.01–0.03 m in the diagnosed v0.18 trials), allowing a buffer while leaving assessment thresholds fixed. The LQR controller itself, obstacle inflation and physical collision checks were not relaxed for the v0.19 change. Control-effort and trajectory plots reside within each archived trial; a systematic quantitative energy/control-effort trade-off is not claimed.

### Part E: Integration, robustness and root-cause correction

- **Chassis pitch:** Original MuJoCo free body settled at approximately 29.92° (95th percentile); 97/101 controlled reference scans had likely floor hits. A simplified front support reduced the pitch percentile to 1.50° and likely floor-hit samples to 0/101. The passive sphere is an **approximation** of contact support, not a documented TurtleBot3 CAD caster.
- **Waypoint obstruction:** In the blocked scenario the second waypoint was 0.108 m from added obstacle geometry, inside the 0.160 m inflation buffer. A safe policy can bypass this *intermediate* waypoint only after a persistent A* failure and a confirmed obstacle-free A* route toward the next intermediate waypoint. The final dock may not be skipped.
- **Visual fault:** Rangefinder overlays were removed only from the rendered camera, restoring sensor updates without switching off real LiDAR measurements.
- **Precision:** The stopping margin led to all 15 runs satisfying the independently scored true-pose requirement. The robot is still not a verified real-world docking system.

### Trade-off and risk register

| Design choice | Benefit demonstrated | Remaining caveat |
|---|---|---|
| Front support in MuJoCo | Eliminates floor-ray artefact in controlled comparison | Alters chassis/contact dynamics from an exact TurtleBot3 model |
| LiDAR inflation + bounded waypoint bypass | Preserves a safety buffer and avoids an unreachable route in all 3 blocked trials | A* still has finite map/resolution/visibility constraints; bypass is not general task replanning |
| Camera render cleaned | Allows marker measurements through rendered camera | Ideal marker, lighting, camera calibration, rolling shutter and real optical noise not covered |
| EKF + gyro and vision | Mean XY estimation RMSE ≈3.81 cm in final suite | No covariance consistency or Q/R sweep reported |
| Tighter estimate-side terminal stop | Strict pose improvement from 7/15 to 15/15 | Future lighting/contact/noise may require recalibration; strict result validated for 3 seeds only |

## Repeatability and marker rubric

The rubric requires five simulation criteria: **A model/controllability/observability; B calibrated sensing, noise and frames; C EKF tuning/performance; D state-feedback control results; E integration/robustness**. The 5-minute video must show completed task, a changed/disturbed run, and estimate/uncertainty alongside truth. The student must be ready to explain Q/R, camera visibility, local observability, LQR weights, and the concrete failure-driven improvements.

**No physical TurtleBot3 run, ROS 2 message/TF verification, wheel-radius hardware calibration, or hardware-versus-simulation quantitative comparison is claimed.** Hardware requires separate approval, sensor calibration and safe on-site demonstration. See `docs/HARDWARE_READINESS_V19.md`.

### Reproduction

```bash
python -m pytest -q
python scripts/verify_archive.py results
python scripts/submission_preflight.py
python -m turtlebot_integrated.cli run --backend mujoco --scenario blocked --seed 0 \
  --config configs/mujoco_v19_precision_trial.yaml \
  --video --out outputs/recheck_blocked_v19
```

The archive stores original per-run Git versions and settings. New reproductions must report their new Git revision, dependency versions and machine. Use `results/mujoco/robustness_v19_strict_15of15/audit_v16.csv` for trial-level numerical inspection.
