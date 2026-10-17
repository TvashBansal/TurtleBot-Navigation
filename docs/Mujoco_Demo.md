# MuJoCo demonstration

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
