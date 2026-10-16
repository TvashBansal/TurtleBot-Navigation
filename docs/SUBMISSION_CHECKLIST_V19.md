# Submission checklist — UTS Integrated Project, Track A

**Assessment source:** 41014 Sensors and Control in Mechatronics Systems, Assessment 4 Integrated Project Marking Guide; 42043 students may work individually or in pairs. **Weight 40%**, due **Exam Week**, submitted through Canvas with GitHub page and 5-minute video; individual viva voce follows. Confirm exact deadline in Canvas.

## Final requirements and status

- [x] **Working compulsory simulation:** 15 MuJoCo runs; 15/15 simulator success, 15/15 strict pose, no reported collisions. `results/mujoco/robustness_v19_strict_15of15/`.
- [x] **State-space and feedback model:** differential-drive model, local controllability analysis, LQR, estimated-state control. `docs/MODEL_AND_DESIGN.md`, `scripts/analyse_model.py`.
- [x] **Sensor/EKF pipeline in simulation:** LiDAR, rendered ArUco, encoder odometry, gyro assistance and EKF. Measurement update and Q/R design explained in engineering docs; caveat about statistical consistency.
- [x] **Repeatability and disturbances:** five scenarios × seeds 0–2; archived evaluation/analysis and verified physical disturbance triggers.
- [x] **Contribution + GenAI text:** draft included in root `README.md`.
- [ ] **Two final v0.19 video source captures**, nominal and blocked, visually inspected and archived to `results/mujoco/`.
- [ ] **One edited five-minute video** showing nominal + changed/disturbed trial, live camera, EKF estimate and uncertainty with labelled simulator truth, narrated explanation; check actual duration. `docs/DEMO_FIVE_MINUTE_PLAN.md`.
- [ ] **Fresh-clone reproduction / archive preflight:** a second clean Codespace/environment should install, run tests and reproduce a sample simulation. `python scripts/submission_preflight.py --strict` checks artifacts; human checks still necessary.
- [ ] **Project page final proofread:** README gives Track, frames, model, parameter values, calibration limitations, comparisons and direct result/video links. Confirm all GitHub links resolve.
- [ ] **Canvas submission:** submit GitHub URL + hosted video URL with correct access, before the actual Exam Week deadline.
- [ ] **Viva preparation:** explain all pipeline components and why each significant failure occurred, including limitations of front support and hardware transfer.
- [ ] **Optional real robot:** not yet tested; do not mark complete or imply HD hardware contribution. See `docs/HARDWARE_READINESS_V19.md`.

## Rubric coverage, not a predicted grade

| Criterion | Evidence to point out to examiner | Remaining caution |
|---|---|---|
| A — Model/controllability/observability (20) | Derivations and rank reproduction in `docs/MODEL_AND_DESIGN.md`, `scripts/analyse_model.py` | Local linearisation, not global or fully electromechanical robot model |
| B — Sensing/calibration/frame handling (20) | ArUco/PnP, range bearing, LiDAR map/frames, camera overlay diagnosis | Camera/wheel values mainly simulation assumptions; real calibration not done |
| C — KF/EKF and metrics (20) | EKF code and 0.03806 m mean XY RMSE; plot true/estimated/covariance | Formal NIS consistency study / Q-R ablation not completed |
| D — Control/tracking (20) | Estimated-state LQR, final 0.14316 m mean error, conservative stopping | More detailed control-effort tradeoff optional |
| E — Integration and robustness (20) | 15/15 repeat runs, no reported collisions, safely bypassed obstacle, physical disturbance audit | Only 3 seeds per scenario; limited model/environment diversity |

A five-minute video and accessible GitHub page are explicit deliverables; omitting them can affect the marking-band ceiling even when code works. Hardware is optional. For high-band work, avoid claiming a result without showing how you measured it.

## Final Codespaces commands

```bash
source .venv/bin/activate
export MUJOCO_GL=osmesa
export MPLBACKEND=Agg
python -m pytest -q
python scripts/analyse_model.py
python scripts/verify_archive.py results
python scripts/submission_preflight.py --strict --report outputs/final_submission_preflight.json
git status --short
```

`--strict` requires both nominal and blocked **archived successful** MuJoCo MP4s, but cannot verify edited five-minute-video quality or Canvas submission. Commit the completed documents and selected video archives; inspect `git status` for uncommitted files. **Never commit `.venv`, `.pytest_cache`, `outputs` or `*.egg-info`.**
