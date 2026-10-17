# MuJoCo demonstration

## Record footage using the validated v0.19 configuration

From the repository root in Codespaces (video quality depends on the renderer and ffmpeg availability):

```bash

set -euo pipefail

cd /workspaces/TurtleBot-Navigation
source .venv/bin/activate

export MUJOCO_GL=osmesa
export MPLBACKEND=Agg

# Generate a unique Run ID for every execution.
RUN_ID="$(date -u +%Y%m%dT%H%M%SZ)_$(python -c 'import uuid; print(uuid.uuid4().hex[:8])')"

RUN_BASE="outputs/final_demo_v19_${RUN_ID}"
mkdir -p "$RUN_BASE"

echo "Starting new MuJoCo simulations: $RUN_ID"

# Fresh nominal simulation.
python -m turtlebot_integrated.cli run \
  --backend mujoco \
  --scenario nominal \
  --seed 0 \
  --config configs/mujoco_v19_precision_trial.yaml \
  --video \
  --out "$RUN_BASE/nominal"

# Fresh blocked simulation.
python -m turtlebot_integrated.cli run \
  --backend mujoco \
  --scenario blocked \
  --seed 0 \
  --config configs/mujoco_v19_precision_trial.yaml \
  --video \
  --out "$RUN_BASE/blocked"

# Confirm expected output files exist and are non-empty.
for scenario in nominal blocked; do
  test -s "$RUN_BASE/$scenario/summary.json"
  test -s "$RUN_BASE/$scenario/demo_capture.mp4"
done

# Save the latest complete run for the archive block.
printf '%s\n' "$RUN_ID" > outputs/.latest_final_demo_v19_run

echo "RUN ID: $RUN_ID"
echo "OUTPUT FOLDER: $RUN_BASE"

cat "$RUN_BASE/nominal/summary.json"
cat "$RUN_BASE/blocked/summary.json"

ls -lh "$RUN_BASE"/{nominal,blocked}/demo_capture.mp4

```

Validate that both runs report `backend=mujoco`, `success=true`, `strict_dock_pose_pass=true` and `collision=false`; blocked trial should show waypoint 1 bypass and accepted camera updates. Open both MP4s before archiving to confirm the overhead robot, camera image and state estimate/uncertainty are visible; check video duration and images are not blank. If the test does not meet these checks, document the failure and repair, **do not substitute kinematic footage**.

```bash

set -euo pipefail

cd /workspaces/TurtleBot-Navigation
source .venv/bin/activate

# Retrieve the latest completed simulation Run ID.
RUN_ID="$(cat outputs/.latest_final_demo_v19_run)"

RUN_BASE="outputs/final_demo_v19_${RUN_ID}"

NOMINAL_ARCHIVE="final_demo_nominal_v19_${RUN_ID}"
BLOCKED_ARCHIVE="final_demo_blocked_v19_${RUN_ID}"

# Confirm both source folders exist.
test -d "$RUN_BASE/nominal"
test -d "$RUN_BASE/blocked"

# Never overwrite an earlier archived run.
test ! -e "results/mujoco/$NOMINAL_ARCHIVE"
test ! -e "results/mujoco/$BLOCKED_ARCHIVE"

# Archive the fresh simulation results.
python scripts/archive_run.py \
  "$RUN_BASE/nominal" \
  --name "$NOMINAL_ARCHIVE"

python scripts/archive_run.py \
  "$RUN_BASE/blocked" \
  --name "$BLOCKED_ARCHIVE"

# Verify archived experiments.
python scripts/verify_archive.py results

# Stage only the newly created archives.
git add \
  "results/mujoco/$NOMINAL_ARCHIVE" \
  "results/mujoco/$BLOCKED_ARCHIVE"

# Review what is being committed.
git diff --cached --stat
git status

# Commit and upload to GitHub.
git commit -m "Archive MuJoCo v0.19 demonstration $RUN_ID"
git push origin main

echo "Archived and pushed Run ID: $RUN_ID"

```

Archive names are immutable; if an archive already exists, do not overwrite it. Check that final MP4s meet GitHub size limits before staging. Additional scenario footage may be selected from a new recorded `disturbance` run if desired.
