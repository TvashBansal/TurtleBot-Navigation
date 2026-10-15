# Evidence checklist aligned with Integrated Project marking guide

**Important:** These are planned evidence items, not yet all demonstrated.

| Part (20 marks each) | Evidence from files | Still to produce for a strong result |
|---|---|---|
| A Modelling | `docs/MODEL_AND_DESIGN.md`, `scripts/analyse_model.py`, `mujoco_backend.py` | Validated parameters, frame diagram, derivation, measured/physical comparison |
| B Sensing | ArUco PnP, pinhole intrinsics, LiDAR simulation and noise | Actual MuJoCo camera frames, calibration study, correct transform verification |
| C Estimation | EKF, Jacobians, covariance, NIS, errors plots | MuJoCo RMSE/NIS/consistency across seeds and ablation vs odometry-only |
| D Control | LQR on estimated state, waypoint mapping, safety brake | MuJoCo tracking and effort results, gain sweep, stable docking |
| E Integration / robustness | Five scenarios, suite runner, success metrics | MuJoCo repeated trials, failure diagnosis and limits discussion |

Distinction requires systematic repeated trials with documented failure causes, quantitative trade-offs and clean reproducible code. High Distinction additionally needs an independent extension (or **working** physical platform, as allowed in the guide) and rigorous evaluation. Hardware is optional and does not replace missing simulation evidence.

## Exact items for GitHub project page

- Clearly identify Track A, team/solo contribution, input parameters, complete model equations and coordinate frame diagram.
- Explain sensor calibration and noise assumptions with evidence.
- Explain EKF state/covariance and camera correction, Q/R from data or justified assumptions.
- Explain control linearisation/LQR weights, controller constraints and safety conditions.
- Add MuJoCo experiment screenshots, plots, repeated-trial table, failures and retuning log.
- Add fresh-machine installation instructions and tested environment versions.
- Embed or link a five-minute demo video with normal and disturbed trials and visible estimate + uncertainty and simulator truth (not fed to controller).
- Include a generative AI declaration with specific tools and purposes, and state contributions accurately.
- Prepare for individual viva: observability, Q/R sensitivity, frames, limitations, covariance and system failures.

## Suggested 5-minute video order

1. 0:00–0:35: problem, platform, architecture and truth-isolation constraint.
2. 0:35–1:15: state-space model and local observability/controllability.
3. 1:15–2:00: real MuJoCo LiDAR + camera ArUco, calibrated frames and EKF uncertainty.
4. 2:00–3:00: nominal navigation and complete ArUco docking.
5. 3:00–4:10: disturbance/noise/blocked-route trial, comparison of errors and controller commands.
6. 4:10–4:45: repeated-trial performance table and diagnosed failure/trade-off.
7. 4:45–5:00: contribution, limitations, next hardware test/working hardware proof if validated.

## Viva quick questions

**Why no ground truth in control?** Real robots lack perfect global pose; only sensors feed EKF. True pose in the experiment loop only scores evaluation and creates plots.

**What if the marker disappears?** EKF predicts from encoders; P grows; map and controls continue on the estimated pose. Real adapter conservatively stops near dock without recent visual confirmation.

**Q/R?** Encoder/Q from an assumed sensor + motion uncertainty, camera/R from assumed pixel-to-range/bearing residuals; report sensitivity and fit to data when available. R lower means camera trusted more, but can cause overconfident/unstable estimates if underestimated.

**Observability?** One static range+bearing measurement gives two constraints on three pose states; motion makes local 3-state observability possible under nondegenerate conditions, stationary one-marker case remains underdetermined.

**LQR?** Riccati solution penalises lateral/heading tracking error plus control effort. Time-varying local linear model, conservative saturation and obstacle guard. These assumptions do not prove stability in all environments.
