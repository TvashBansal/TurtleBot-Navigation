# v0.7 experimental LQR tracking and direct stop-cause logging

The historical v0.4–v0.6 configurations still use the prior controller defaults.

## Why this exists

The original LQR tracking produces `w=1.6 rad/s` even for ~0.3 rad bearing error, while continuing to command `v=0.22 m/s`. MuJoCo trials show angular-command saturation in >80% of samples, with ~60–91% controller-selected stopping. A gentler steering scale and heading-dependent translation speed are evaluated as an **experimental intervention**, not assumed to be a success.

## Changes

- `control.py`: optionally scale LQR angular feedback before clipping; schedule speed as heading error increases; adjust large-angle rotate-in-place threshold and gain; cap forward speed. The existing LQR law, model and gains are not bypassed.
- `navigation.py`: configurable path lookahead, and explicit `last_stop_reason` telemetry. Physical LiDAR safety stop remains unchanged.
- `experiment.py` and `types.py`: save measured controller/safety reasons, nearest frontal LiDAR range, waypoint index and bearing error in each new `trajectory.json` record. Existing archives remain readable and unchanged.
- Two candidate YAML configurations: `mujoco_v07_steering.yaml` and `mujoco_v07_stable.yaml`.

## Procedure

In GitHub Codespaces, after extracting and uploading **individual files** into their respective repository locations, commit changes in GitHub, then:

```bash
cd /workspaces/TurtleBot-Navigation
git pull --ff-only origin main
source .venv/bin/activate
export MUJOCO_GL=osmesa MPLBACKEND=Agg
python -m pytest -q
python -m turtlebot_integrated.cli run --backend mujoco --scenario nominal --seed 7 --config configs/mujoco_v07_steering.yaml --out outputs/nominal_v07_steering
python scripts/compare_v07.py
```

If the steering candidate still fails or stalls, run the slower stable comparison as a separate experiment:

```bash
python -m turtlebot_integrated.cli run --backend mujoco --scenario nominal --seed 7 --config configs/mujoco_v07_stable.yaml --out outputs/nominal_v07_stable
python scripts/compare_v07.py
```

**Do not archive either run as a successful MuJoCo experiment unless docking is confirmed, collision is false and the reported metrics match.** Failed trials can be committed as explicitly labelled diagnostics.

This change does not resolve all camera visibility/marker-detection limitations and does not guarantee docking. The estimator and collision safeguards remain unchanged. No simulator ground-truth poses are used for feedback.
