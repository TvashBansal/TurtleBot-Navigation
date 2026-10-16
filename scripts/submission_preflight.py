"""Read-only pre-submission audit of a TurtleBot-Navigation checkout.

Usage (from any location):
    python scripts/submission_preflight.py
    python scripts/submission_preflight.py --strict --report outputs/final_preflight.json

--strict fails if recommended MuJoCo demonstration MP4s have not been archived.
This cannot certify physical hardware or that a five-minute edited video was submitted.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import subprocess

SCENARIOS = ("nominal", "noise", "blocked", "disturbance", "camera_loss")
ROOT = Path(__file__).resolve().parents[1]
SUITE = Path("results/mujoco/robustness_v19_strict_15of15")


def _load(path: Path) -> dict:
    with path.open(encoding="utf-8") as stream:
        return json.load(stream)


def _git_tracked(root: Path, rel: str) -> bool:
    try:
        cp = subprocess.run(["git", "ls-files", "--error-unmatch", "--", rel],
                            cwd=root, capture_output=True, text=True, check=False)
        return cp.returncode == 0
    except OSError:
        return False


def inspect(root: Path = ROOT, require_demo: bool = False) -> dict:
    """Inspect files only; never modify archives or execute a simulator."""
    root = Path(root)
    failures, warnings, passed = [], [], []
    def check(ok: bool, label: str):
        (passed if ok else failures).append(label)

    core = ["README.md", "pyproject.toml", "configs/mujoco_v19_precision_trial.yaml",
            "turtlebot_integrated/experiment.py", "turtlebot_integrated/navigation.py",
            "turtlebot_integrated/ekf.py", "turtlebot_integrated/ros2_bridge.py",
            "docs/FINAL_VALIDATION_V19.md", "docs/DEMO_FIVE_MINUTE_PLAN.md",
            "docs/HARDWARE_READINESS_V19.md", "docs/SUBMISSION_CHECKLIST_V19.md"]
    for rel in core:
        check((root / rel).is_file(), f"Required project file: {rel}")

    archive = root / SUITE
    check((archive / "manifest.json").is_file(), "v0.19 suite archived with manifest")
    check((archive / "aggregate.json").is_file(), "v0.19 suite aggregate recorded")
    check((archive / "config_snapshot.json").is_file(), "v0.19 suite config recorded")
    check((archive / "suite_metadata.json").is_file(), "v0.19 suite provenance recorded")
    check((archive / "audit_v16.json").is_file(), "Strict docking audit archived")
    summary = {"total": 0, "simulator_passes": 0, "strict_pose_passes": 0,
               "collisions": 0, "physical_disturbance_trials": 0,
               "mean_position_error_m": None, "scenarios": {}}
    errs = []
    for scenario in SCENARIOS:
        per_scenario = []
        for seed in range(3):
            p = archive / scenario / f"seed_{seed}" / "summary.json"
            if not p.is_file():
                failures.append(f"Missing trial: {scenario} seed {seed}")
                continue
            try:
                s = _load(p)
            except (OSError, ValueError) as exc:
                failures.append(f"Unreadable trial: {p.relative_to(root)} ({exc})")
                continue
            check(s.get("backend") == "mujoco" and s.get("scenario") == scenario
                  and s.get("seed") == seed,
                  f"Authentic trial identifiers: {scenario} seed {seed}")
            summary["total"] += 1
            summary["simulator_passes"] += int(s.get("success") is True)
            summary["strict_pose_passes"] += int(s.get("strict_dock_pose_pass") is True)
            summary["collisions"] += int(s.get("collision") is True)
            errs.append(float(s["position_error_m"]))
            if scenario == "disturbance":
                summary["physical_disturbance_trials"] += int(s.get("disturbance_applied_steps", 0) > 0)
            per_scenario.append(s)
        summary["scenarios"][scenario] = {
            "trials": len(per_scenario),
            "simulator_passes": sum(s.get("success") is True for s in per_scenario),
            "strict_pose_passes": sum(s.get("strict_dock_pose_pass") is True for s in per_scenario),
            "collisions": sum(s.get("collision") is True for s in per_scenario),
        }
    if errs:
        summary["mean_position_error_m"] = round(sum(errs) / len(errs), 5)
    check(summary["total"] == 15, "15 archived MuJoCo trials")
    check(summary["simulator_passes"] == 15, "15/15 simulator successes")
    check(summary["strict_pose_passes"] == 15, "15/15 strict pose passes")
    check(summary["collisions"] == 0, "Zero reported collisions")
    check(summary["physical_disturbance_trials"] == 3, "All three external-force trials applied disturbance")

    agg_path = archive / "aggregate.json"
    if agg_path.is_file():
        try:
            agg = _load(agg_path)
            check(agg.get("count") == 15 and agg.get("backend") == "mujoco",
                  "Suite aggregate contains 15 MuJoCo trials")
        except (OSError, ValueError):
            failures.append("Suite aggregate is malformed")

    cfg_path = archive / "config_snapshot.json"
    if cfg_path.is_file():
        try:
            cfg = _load(cfg_path)
            check(cfg.get("dock_tolerance") == 0.18
                  and cfg.get("nav_dock_position_margin_m") == 0.04
                  and cfg.get("nav_dock_heading_margin_rad") == 0.035,
                  "Archived precision configuration and strict tolerance")
        except (OSError, ValueError):
            failures.append("Suite configuration is malformed")

    manifests = sorted((root / "results").rglob("manifest.json"))
    if not manifests:
        failures.append("No archive manifests found")
    else:
        validated_files = 0
        for manifest in manifests:
            try:
                inv = _load(manifest)["file_inventory"]
                for entry in inv:
                    relative = Path(entry["path"])
                    if relative.is_absolute() or ".." in relative.parts:
                        failures.append(f"Unsafe manifest entry in {manifest.relative_to(root)}")
                        continue
                    f = manifest.parent / relative
                    if not f.is_file():
                        failures.append(f"Missing archive file: {f.relative_to(root)}")
                        continue
                    h = hashlib.sha256()
                    with f.open("rb") as fh:
                        for chunk in iter(lambda: fh.read(1024 * 1024), b""):
                            h.update(chunk)
                    if f.stat().st_size != entry["bytes"] or h.hexdigest() != entry["sha256"]:
                        failures.append(f"Archive integrity mismatch: {f.relative_to(root)}")
                    else:
                        validated_files += 1
            except (OSError, ValueError, KeyError, TypeError) as exc:
                failures.append(f"Invalid manifest: {manifest.relative_to(root)} ({exc})")
        passed.append(f"Verified {len(manifests)} manifests and {validated_files} archive files" )

    # Requiring final v0.19 proof is stronger than finding *any* older video.
    demo_scenarios = set()
    for kind in ("nominal", "blocked"):
        demo = root / "results/mujoco" / f"final_demo_{kind}_v19"
        video, spath, cpath = (demo / "demo_capture.mp4",
                               demo / "summary.json", demo / "config_snapshot.json")
        if not (video.is_file() and spath.is_file() and cpath.is_file()
                and video.stat().st_size > 0):
            continue
        try:
            s, c = _load(spath), _load(cpath)
            if (s.get("backend") == "mujoco" and s.get("scenario") == kind
                    and s.get("success") is True and s.get("strict_dock_pose_pass") is True
                    and c.get("nav_dock_position_margin_m") == .04
                    and c.get("nav_dock_heading_margin_rad") == .035):
                demo_scenarios.add(kind)
        except (ValueError, OSError):
            continue
    summary["archived_v19_demo_scenarios"] = sorted(demo_scenarios)
    for kind in ("nominal", "blocked"):
        if kind not in demo_scenarios:
            message = f"Missing verified final v0.19 {kind} MP4 archive with strict-pose success"
            (failures if require_demo else warnings).append(message)
    warnings.append("Edited five-minute narration/video must be visually reviewed and submitted manually")
    warnings.append("ROS 2 physical hardware and sensor-frame calibration NOT verified by this audit")
    warnings.append("Human review of AI/contribution statements, rubric and ethical disclosure required")

    suite_rel = SUITE / "manifest.json"
    if (root / suite_rel).is_file() and not _git_tracked(root, suite_rel.as_posix()):
        warnings.append("Suite manifest exists locally but Git does not track it; commit and push results")

    return {"ok": not failures, "failures": failures, "passed": passed,
            "warnings": warnings, "metrics": summary,
            "note": "Checks archived evidence; cannot certify real-world hardware or video quality."}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=ROOT)
    parser.add_argument("--strict", action="store_true", help="Require nominal and blocked archived MP4s")
    parser.add_argument("--report", type=Path, help="Write machine-readable JSON report (outputs/ recommended)")
    args = parser.parse_args()
    report = inspect(args.root, args.strict)
    m = report["metrics"]
    print("=== TURTLEBOT V0.19 SUBMISSION PREFLIGHT ===")
    print(f"Archived runs: {m['total']}/15 | simulator success: {m['simulator_passes']}/15 | "
          f"strict pose: {m['strict_pose_passes']}/15 | collisions: {m['collisions']}")
    print(f"Physical disturbance applied: {m['physical_disturbance_trials']}/3 trials | "
          f"mean position error: {m['mean_position_error_m']} m")
    for scenario, item in m["scenarios"].items():
        print(f"  {scenario:12s}: {item['strict_pose_passes']}/{item['trials']} strict pose")
    print(f"Archive integrity: {'PASS' if not any('archive' in e.lower() or 'manifest' in e.lower() for e in report['failures']) else 'FAIL'}")
    for err in report["failures"]: print("FAIL:", err)
    for msg in report["warnings"]: print("NOTE:", msg)
    print("PREFLIGHT:", "PASS (with manual items)" if report["ok"] else "FAIL")
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print("Report:", args.report)
    if not report["ok"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
