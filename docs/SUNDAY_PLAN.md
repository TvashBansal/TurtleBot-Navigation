# Completion plan: Thu 15 → Sun 18 October 2026

## Thursday: baseline complete

- [x] Modular Python architecture with both MuJoCo backend implementation and low-fidelity test double.
- [x] State prediction, camera measurement update, LQR, lidar-driven occupancy and A*.
- [x] Nominal closed-loop offline smoke test.
- [ ] Verify MuJoCo installs and renders on your development computer.

## Friday: validate mandatory simulator

- [ ] Run nominal physics simulation, inspect wheels/contacts and camera frame.
- [ ] Fix MuJoCo model bugs; collect reliable nominal-video evidence.
- [ ] Confirm ArUco 3D pose distance/bearing against known geometric positions.

## Saturday: robustness and hardware interface

- [x] Software structure for five scenarios and ROS 2 adapter.
- [ ] MuJoCo disturbed/noisy/blocked/camera-loss trials, diagnose any failures.
- [ ] Confirm physical robot model parameters and ROS 2 topic list remotely if possible.
- [ ] Add 5+ seeds and quantitative comparison.

## Sunday: package evidence and test reproductions

- [x] GitHub-ready structured source + runbook + baseline tests.
- [ ] Merge verified MuJoCo metrics, screenshots and recorded simulator video.
- [ ] Make 5-minute final demonstration plan and assemble required evidence.
- [ ] Prepare oral explanations and finalise project-page content.

**Most important milestone:** do **not** wait until Sunday to try MuJoCo. The test double is useful development infrastructure, but the rubric explicitly requires real MuJoCo simulation. Physical lab integration remains a later optional task.
