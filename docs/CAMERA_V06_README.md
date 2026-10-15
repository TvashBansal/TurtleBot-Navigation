# v0.6: camera detection and steering-control experiments

**Status:** Experimental. MuJoCo docking has NOT been established. `results/` evidence must only be committed with accurate success/failure labels.

## Hypotheses

1. The true rendered camera tag may be too small or low-contrast at useful approach distances. Optional multiscale ArUco detection retries the **same RGB frame** at native, contrast-enhanced and doubled resolution. Only genuine detected marker corners and solvePnP can yield measurements. No ground-truth injection.
2. Excessive steering may cause wheel slip and accumulated heading error. The conservative config caps commanded `max_omega` at 0.75 rad/s. Do not treat this as calibrated before a full experiment.

`configs/mujoco_v06_camera_only.yaml` changes **only camera detection** relative to v0.5. `configs/mujoco_v06_conservative.yaml` additionally caps turning. Comparisons should use **the same MuJoCo backend, scenario and seed**.

## Steps in Codespaces

```bash
cd /workspaces/TurtleBot-Navigation
git pull --ff-only origin main
source .venv/bin/activate
export MUJOCO_GL=osmesa MPLBACKEND=Agg
python -m pytest -q
python scripts/diagnose_camera_v06.py
```

Review `outputs/diagnostics/camera_v06/comparison.json` and real rendered PNGs. If improved, run the camera-only trial first:

```bash
python -m turtlebot_integrated.cli run --backend mujoco --scenario nominal --seed 7 \
  --config configs/mujoco_v06_camera_only.yaml --out outputs/nominal_v06_camera_only
```

If it fails, test *only one additional variable*: turn cap.

```bash
python -m turtlebot_integrated.cli run --backend mujoco --scenario nominal --seed 7 \
  --config configs/mujoco_v06_conservative.yaml --out outputs/nominal_v06_conservative
```

Collect summaries, trajectory plots and available camera replay evidence. Report failures honestly. Do not commit debug videos or runs into verified `results/` until labelled correctly.

## Limits

- Upsampling cannot recover markers hidden by obstacles, behind the camera, or entirely out of view. It may also fail for severely blurred, clipped or steeply angled markers.
- The ARUCO detector is supplemental image processing, **not** a new physical sensor. Camera calibration remains necessary for the real robot.
- This update does not change wheel physics, EKF, navigator or controller code. It cannot by itself resolve cumulative yaw drift.
- Optional camera processing increases CPU time. Check frame rate before hardware deployment.
