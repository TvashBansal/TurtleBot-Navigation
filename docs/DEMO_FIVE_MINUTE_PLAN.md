# Five-minute MuJoCo demonstration — production plan (300 seconds)

**Deliverable status:** A storyboard and speaking guide, **not** a completed edited video. The student must execute the final captures, inspect both MP4s, record narration if desired, edit the clips, and submit the video link. Do not describe unrecorded footage as filmed.

## Record footage using the validated v0.19 configuration

From the repository root in Codespaces (video quality depends on the renderer and ffmpeg availability):

```bash
source .venv/bin/activate
export MUJOCO_GL=osmesa
export MPLBACKEND=Agg

python -m turtlebot_integrated.cli run --backend mujoco --scenario nominal --seed 0 \
  --config configs/mujoco_v19_precision_trial.yaml --video \
  --out outputs/final_demo_nominal_v19

python -m turtlebot_integrated.cli run --backend mujoco --scenario blocked --seed 0 \
  --config configs/mujoco_v19_precision_trial.yaml --video \
  --out outputs/final_demo_blocked_v19

cat outputs/final_demo_nominal_v19/summary.json
cat outputs/final_demo_blocked_v19/summary.json
ls -lh outputs/final_demo_*/demo_capture.mp4
```

Validate that both runs report `backend=mujoco`, `success=true`, `strict_dock_pose_pass=true` and `collision=false`; blocked trial should show waypoint 1 bypass and accepted camera updates. Open both MP4s before archiving to confirm the overhead robot, camera image and state estimate/uncertainty are visible; check video duration and images are not blank. If the test does not meet these checks, document the failure and repair, **do not substitute kinematic footage**.

```bash
python scripts/archive_run.py outputs/final_demo_nominal_v19 --name final_demo_nominal_v19
python scripts/archive_run.py outputs/final_demo_blocked_v19 --name final_demo_blocked_v19
python scripts/verify_archive.py results
# Review git diff/status before committing.
git add results/mujoco/final_demo_nominal_v19 results/mujoco/final_demo_blocked_v19
git commit -m "Archive v0.19 final nominal and blocked MuJoCo demonstration MP4s"
git push origin main
```

Archive names are immutable; if an archive already exists, do not overwrite it. Check that final MP4s meet GitHub size limits before staging. Additional scenario footage may be selected from a new recorded `disturbance` run if desired.

## Exactly five minutes: suggested timeline and shots

| Timecode | Visual evidence | Talking points |
|---|---|---|
| **0:00–0:25** | Project title + MuJoCo scene and docking target | Identify Track A, estimated-state rule, TurtleBot3-like robot and task |
| **0:25–1:10** | Architecture / frame diagram from README; show MuJoCo camera and LiDAR | Wheel encoders, gyro, LiDAR, RGB; world/base/optical/marker frames; 0.16 m inflated occupancy |
| **1:10–1:55** | Model and controller equations; estimation/covariance plot | State-space kinematics, local controllability vs stationary, EKF range/bearing, LQR uses estimated pose |
| **1:55–2:40** | v0.19 nominal MP4 (full run or annotated replay), ending at dock | Point out route, live robot and marker, pose estimate with uncertainty and ground truth for **evaluation only** |
| **2:40–3:30** | v0.19 blocked MP4 with obstacle, waypoint bypass and dock | How A* safely bypasses an obstructed intermediate waypoint, without removing collision inflation |
| **3:30–4:20** | Static results charts from `results/...` + summary table | 15/15 simulator and strict passes, 0 reported collisions, 0.14316 m mean error; 3 actual disturbances verified |
| **4:20–4:50** | Failure-analysis images or simple text panels: pitching, camera overlay, endpoint margin | Explain **why** changes were needed and limits of their validity |
| **4:50–5:00** | Final conclusions and reference to GitHub | State physical hardware is NOT verified; invite repo reproduction |

To fill longer sections, slow down on analysis plots, add annotated callouts, and narrate; **do not fabricate robot motion or stretch footage in a way that misrepresents performance.** Ensure total duration is 5:00 or slightly under. The five-minute cutoff matters: content after it may not be graded.

## Draft spoken narration (adapt to your actual edit)

> This is my Integrated Project for Track A: LiDAR-assisted waypoint navigation and ArUco visual docking in MuJoCo. The robot must estimate its pose from sensors and navigate using that estimated state, not simulator ground truth. I use a TurtleBot3-like differential-drive model with wheel encoders, a simulated yaw-rate sensor, ray LiDAR and a rendered front camera.
>
> The state is planar position x and y and heading theta. Differential-drive kinematics provide the prediction model, and a local tracking-error linearisation is used to calculate discrete-time LQR feedback. LiDAR constructs an occupancy grid with an inflation margin; A-star finds safe waypoint paths and the reactive guard brakes near obstacles. The true simulated pose is logged separately for offline performance assessment, never supplied to the controller.
>
> ArUco detections provide range and bearing to a fixed marker. The EKF predicts from encoder increments, optionally assisted by yaw rate, and corrects on accepted marker measurements. The image panel shows what the robot camera renders. This estimate and its uncertainty are compared with ground truth in the plots, so we can measure tracking and localisation performance.
>
> Here the nominal trial navigates the route and docks. In the blocked trial, an added obstacle makes an intermediate waypoint unreachable. Rather than crossing the inflated obstacle, the planner skips that intermediate target only when A-star finds a safe path to the next one. The final docking target is never bypassed. You can see the marker observations help correct the robot pose as it approaches the station.
>
> I evaluated three seeds across each of five scenarios: nominal, extra sensor noise, blocked obstacle, physical disturbance and camera dropout. In the final v0.19 suite all fifteen simulations passed the simulator success rule and our stricter true-pose dock rule of 18 centimetres and 0.18 radians. There were zero reported collisions, and the mean final docking error was about 14.3 centimetres. All three physical disturbance trials recorded the disturbance being applied. The mean XY estimation RMSE was about 3.8 centimetres.
>
> The development process also revealed model and sensing failures. Excessive simulated chassis pitch made LiDAR detect the floor; I tested a simplified contact support. Rangefinder display lines interfered with ArUco detection, so I removed the visual overlay without disabling LiDAR. Finally, I introduced a conservative estimate-side stop margin without relaxing the strict scoring tolerance. This improved strict passes from seven to fifteen out of fifteen on the tested seeds. The passive support is a simplification, and the results do not prove real hardware performance. The repository contains source code, experiment settings, reproducible archived results and my analysis.

This is a **suggested spoken draft**, not proof that all visuals have been captured. Rehearse at your own natural speed and edit narration to match the video and final recorded evidence. Show the relevant estimation and control plots clearly enough to read; static results alone do not replace working video.

## Video verification checklist

- [ ] Exactly 5 minutes or shorter; title/track visible.
- [ ] Nominal **v0.19** MuJoCo task completion, not test double.
- [ ] Varied case (blocked or disturbance) shown completing, clearly labeled.
- [ ] Camera image showing ArUco marker at dock.
- [ ] Estimate + covariance/uncertainty displayed **beside** true state (truth labelled offline evaluation).
- [ ] Model, EKF, LQR and safe route planning each explained.
- [ ] Results/limits accurately reflect the 15 archived runs.
- [ ] Final edited video uploaded and accessible with permissions before Canvas submission.
