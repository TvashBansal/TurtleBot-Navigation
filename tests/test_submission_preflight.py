"""Pure-Python tests of the read-only final submission audit (no MuJoCo required)."""
from __future__ import annotations

import importlib.util
import json
from pathlib import Path


MODULE = Path(__file__).resolve().parents[1] / "scripts" / "submission_preflight.py"
spec = importlib.util.spec_from_file_location("submission_preflight", MODULE)
preflight = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(preflight)


def fixture(root: Path) -> Path:
    for item in (
        "README.md", "pyproject.toml", "configs/mujoco_v19_precision_trial.yaml",
        "turtlebot_integrated/experiment.py", "turtlebot_integrated/navigation.py",
        "turtlebot_integrated/ekf.py", "turtlebot_integrated/ros2_bridge.py",
        "docs/FINAL_VALIDATION_V19.md", "docs/DEMO_FIVE_MINUTE_PLAN.md",
        "docs/HARDWARE_READINESS_V19.md", "docs/SUBMISSION_CHECKLIST_V19.md"
    ):
        p = root / item
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text("test data")
    suite = root / preflight.SUITE
    suite.mkdir(parents=True, exist_ok=True)
    config = {"dock_tolerance": .18, "nav_dock_position_margin_m": .04,
              "nav_dock_heading_margin_rad": .035}
    (suite / "config_snapshot.json").write_text(json.dumps(config))
    (suite / "suite_metadata.json").write_text("{}")
    (suite / "aggregate.json").write_text(json.dumps({"count": 15, "backend": "mujoco"}))
    (suite / "audit_v16.json").write_text("{}")
    (suite / "manifest.json").write_text(json.dumps({"file_inventory": []}))
    for scenario in preflight.SCENARIOS:
        for seed in range(3):
            dest = suite / scenario / f"seed_{seed}"
            dest.mkdir(parents=True, exist_ok=True)
            (dest / "summary.json").write_text(json.dumps({
                "backend": "mujoco", "scenario": scenario, "seed": seed,
                "success": True, "strict_dock_pose_pass": True,
                "position_error_m": 0.14, "collision": False,
                "disturbance_applied_steps": 4 if scenario == "disturbance" else 0
            }))
    for name in ("nominal", "blocked"):
        demo = root / "results/mujoco" / f"final_demo_{name}_v19"
        demo.mkdir(parents=True, exist_ok=True)
        (demo / "demo_capture.mp4").write_bytes(b"video stub")
        (demo / "summary.json").write_text(json.dumps({"backend": "mujoco", "success": True,
                                                       "scenario": name, "strict_dock_pose_pass": True}))
        (demo / "config_snapshot.json").write_text(json.dumps(config))
    return suite


def test_complete_fixture(tmp_path):
    fixture(tmp_path)
    outcome = preflight.inspect(tmp_path, require_demo=True)
    assert outcome["ok"], outcome["failures"]
    assert outcome["metrics"]["total"] == 15
    assert outcome["metrics"]["strict_pose_passes"] == 15
    assert outcome["metrics"]["physical_disturbance_trials"] == 3


def test_strict_docking_failure_caught(tmp_path):
    suite = fixture(tmp_path)
    p = suite / "blocked/seed_1/summary.json"
    data = json.loads(p.read_text())
    data["strict_dock_pose_pass"] = False
    p.write_text(json.dumps(data))
    outcome = preflight.inspect(tmp_path)
    assert not outcome["ok"]
    assert outcome["metrics"]["strict_pose_passes"] == 14


def test_missing_trial_caught(tmp_path):
    suite = fixture(tmp_path)
    (suite / "noise/seed_2/summary.json").unlink()
    outcome = preflight.inspect(tmp_path)
    assert not outcome["ok"]
    assert any("Missing trial" in item for item in outcome["failures"])


def test_unchanged_sim_metrics_but_demo_strict_fails(tmp_path):
    fixture(tmp_path)
    (tmp_path / "results/mujoco/final_demo_blocked_v19/demo_capture.mp4").unlink()
    relaxed = preflight.inspect(tmp_path)
    strict = preflight.inspect(tmp_path, require_demo=True)
    assert relaxed["ok"]
    assert not strict["ok"]
    assert strict["metrics"]["simulator_passes"] == 15


def test_manifest_hash_integrity_check(tmp_path):
    suite = fixture(tmp_path)
    demo = suite / "sample.txt"
    demo.write_text("clean")
    import hashlib
    (suite / "manifest.json").write_text(json.dumps({
        "file_inventory": [{"path": "sample.txt", "bytes": 5,
                            "sha256": hashlib.sha256(b"clean").hexdigest()}]
    }))
    assert preflight.inspect(tmp_path)["ok"]
    demo.write_text("changed")
    outcome = preflight.inspect(tmp_path)
    assert not outcome["ok"]
    assert any("integrity mismatch" in item for item in outcome["failures"])
