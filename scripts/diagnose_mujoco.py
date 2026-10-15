"""Non-invasive MuJoCo LiDAR, wheel-motion, and ArUco-camera diagnostics.

Outputs go only to outputs/diagnostics (ignored by Git). The controller never
reads simulator truth; truth is used here only to check physical actuation.
"""
from __future__ import annotations

import json
import math
import os
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "osmesa")
os.environ.setdefault("MPLBACKEND", "Agg")

import cv2
import numpy as np
from turtlebot_integrated.camera import marker_from_rgb
from turtlebot_integrated.config import Config
from turtlebot_integrated.geometry import wrap
from turtlebot_integrated.mujoco_backend import MuJoCoBackend
from turtlebot_integrated.scenarios import get_scenario


OUT = Path("outputs/diagnostics")
OUT.mkdir(parents=True, exist_ok=True)
CFG = Config()


def drive_probe(v: float, w: float, steps: int = 25) -> dict:
    sim = MuJoCoBackend(CFG, get_scenario("nominal", 7), seed=7, record_camera=False)
    try:
        first = sim.get_truth().copy()
        first_pkt = sim.observe()
        for _ in range(steps):
            sim.step(v, w)
        last = sim.get_truth().copy()
        last_pkt = sim.observe()
        dl = float(last_pkt.encoder_left - first_pkt.encoder_left)
        dr = float(last_pkt.encoder_right - first_pkt.encoder_right)
        predicted_ds = CFG.wheel_radius * (dl + dr) / 2
        predicted_dtheta = CFG.wheel_radius * (dr - dl) / CFG.wheel_separation
        return {
            "command": [v, w], "duration_s": round(steps * CFG.dt, 3),
            "truth_delta_xy_m": np.round(last[:2] - first[:2], 4).tolist(),
            "truth_delta_heading_rad": round(wrap(float(last[2] - first[2])), 4),
            "encoder_delta_rad": [round(dl, 4), round(dr, 4)],
            "encoder_predicted_distance_m": round(predicted_ds, 4),
            "encoder_predicted_heading_rad": round(predicted_dtheta, 4),
        }
    finally:
        sim.close()


def main() -> None:
    sim = MuJoCoBackend(CFG, get_scenario("nominal", 7), seed=7, record_camera=True)
    try:
        p = sim.observe()
        front = np.abs(p.scan.angles) < 0.4
        near = sorted(zip(p.scan.angles[front], p.scan.ranges[front]),
                      key=lambda ar: ar[1])[:5]
        result = {"initial_front_min_m": round(float(min(p.scan.ranges[front])), 4),
                  "initial_shortest_front_rays": [[round(float(a), 3), round(float(d), 4)]
                                                   for a, d in near]}
        print("Initial front minimum range (m):", result["initial_front_min_m"])
        print("Five nearest front rays [angle_rad, distance_m]:", result["initial_shortest_front_rays"])

        # Isolate the camera from waypoint/obstacle navigation.
        sim.data.qpos[:7] = [0.50, 0.0, 0.088, 1, 0, 0, 0]
        sim.mj.mj_forward(sim.model, sim.data)
        rgb = sim.render_rgb("front_camera")
        cv2.imwrite(str(OUT / "camera_original.png"), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        variants = {
            "original": rgb,
            "horizontal_flip": np.ascontiguousarray(rgb[:, ::-1]),
            "vertical_flip": np.ascontiguousarray(rgb[::-1]),
            "both_flips": np.ascontiguousarray(rgb[::-1, ::-1]),
        }
        result["aruco_detected_by_image_orientation"] = {}
        for name, image in variants.items():
            detection = marker_from_rgb(image, CFG)
            result["aruco_detected_by_image_orientation"][name] = detection is not None
        result["image_mean_rgb"] = np.round(rgb.mean(axis=(0, 1)), 2).tolist()
        print("ArUco detected by orientation:", result["aruco_detected_by_image_orientation"])
        print("Camera image:", OUT / "camera_original.png")
    finally:
        sim.close()

    result["straight_drive"] = drive_probe(0.12, 0.0)
    result["turn_in_place"] = drive_probe(0.0, 0.7)
    print("Straight-drive physical check:", result["straight_drive"])
    print("Turn-in-place physical check:", result["turn_in_place"])
    (OUT / "mujoco_diagnostics.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print("Diagnostic summary:", OUT / "mujoco_diagnostics.json")


if __name__ == "__main__":
    main()
