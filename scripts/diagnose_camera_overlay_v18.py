"""Compare default MuJoCo rangefinder debug overlay vs clean front camera.

This is an OFFLINE camera diagnostic. It reconstructs recorded true poses
solely to render evaluation images; ground truth never enters control/EKF.
The live camera and LiDAR measurement code are not changed by this script.
"""
import argparse
import json
import math
from pathlib import Path

import cv2
import mujoco
import numpy as np

from turtlebot_integrated.camera import marker_from_rgb
from turtlebot_integrated.config import Config
from turtlebot_integrated.mujoco_backend import MuJoCoBackend
from turtlebot_integrated.scenarios import get_scenario


def prepare_pose(sim: MuJoCoBackend, xyt: list[float]):
    """Static level-chassis pose reconstruction (not an original frame)."""
    jid = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_JOINT, 'base_freejoint')
    adr = int(sim.model.jnt_qposadr[jid])
    z = float(sim.data.qpos[adr + 2])
    x, y, yaw = map(float, xyt[:3])
    sim.data.qpos[adr:adr+3] = [x, y, z]
    sim.data.qpos[adr+3:adr+7] = [math.cos(yaw/2), 0, 0, math.sin(yaw/2)]
    sim.data.qvel[:] = 0
    sim.data.xfrc_applied[:] = 0
    mujoco.mj_forward(sim.model, sim.data)


def yellow_count(rgb: np.ndarray) -> int:
    return int(np.count_nonzero((rgb[..., 0] > 170) &
                                (rgb[..., 1] > 165) &
                                (rgb[..., 2] < 110)))


def detection_info(rgb: np.ndarray, cfg: Config) -> dict:
    gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    corners, ids, rejected = cv2.aruco.ArucoDetector(dictionary).detectMarkers(gray)
    detection = marker_from_rgb(rgb, cfg)
    return dict(
        detected=bool(detection is not None),
        visible_ids=[] if ids is None else ids.flatten().tolist(),
        rejected_candidates=len(rejected),
        range_m=round(float(detection.distance), 3) if detection is not None else None,
        bearing_deg=round(math.degrees(float(detection.bearing)), 2) if detection is not None else None,
        yellow_pixels=yellow_count(rgb),
    )


def compare(trial: Path, config: Path, out: Path, n: int = 4):
    summary = json.loads((trial / 'summary.json').read_text(encoding='utf-8'))
    if summary.get('backend') != 'mujoco':
        raise ValueError('An actual MuJoCo run is required')
    rows = json.loads((trial / 'trajectory.json').read_text(encoding='utf-8'))
    cfg = Config.load(config)
    if not rows:
        raise ValueError('Empty trajectory')
    seed = int(summary['seed'])
    sim = MuJoCoBackend(cfg, get_scenario(str(summary['scenario']), seed), seed,
                        record_camera=True)
    out.mkdir(parents=True, exist_ok=True)
    report = []
    try:
        # Four time-spaced views from 50-85% of the original recorded run,
        # where the v0.17 reconstructed views show the tag clearly.
        indices = sorted(set(int(round(frac*(len(rows)-1))) for frac in
                             np.linspace(0.50, 0.82, n)))
        for idx in indices:
            row = rows[idx]
            prepare_pose(sim, row['truth'])
            # Control: default MuJoCo options (rangefinder rays visible).
            sim.renderer.update_scene(sim.data, camera='front_camera')
            dirty = sim.renderer.render().copy()
            # Treatment: SAME pose, clean visualization flags only.
            clean = sim.render_rgb('front_camera')
            a, b = detection_info(dirty, cfg), detection_info(clean, cfg)
            h, w = dirty.shape[:2]
            pair = np.concatenate([cv2.cvtColor(dirty, cv2.COLOR_RGB2BGR),
                                   cv2.cvtColor(clean, cv2.COLOR_RGB2BGR)], axis=1)
            cv2.putText(pair, f"DEFAULT rays: ArUco={a['detected']}",
                        (12, 25), cv2.FONT_HERSHEY_SIMPLEX, .6, (30, 60, 230), 2)
            cv2.putText(pair, f"CLEAN camera: ArUco={b['detected']}",
                        (w + 12, 25), cv2.FONT_HERSHEY_SIMPLEX, .6, (30, 220, 30), 2)
            output = out / f'camera_overlay_comparison_{idx:04d}.png'
            if not cv2.imwrite(str(output), pair):
                raise OSError(f'Failed writing {output}')
            report.append(dict(index=idx, time_s=float(row.get('t', 0)),
                               default=a, clean=b, image=str(output)))
            print(f"t={float(row.get('t',0)):.2f}s | "
                  f"default ArUco={a['detected']} yellow={a['yellow_pixels']} | "
                  f"clean ArUco={b['detected']} yellow={b['yellow_pixels']}")
    finally:
        sim.close()
    result = dict(source_run=str(trial), scenario=summary['scenario'], seed=seed,
                  default_detections=sum(r['default']['detected'] for r in report),
                  clean_detections=sum(r['clean']['detected'] for r in report),
                  views=report, note='Camera-only off-line render comparison; not an original video frame')
    (out / 'comparison.json').write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
    print('Report:', out / 'comparison.json')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trial', type=Path)
    parser.add_argument('--config', type=Path,
                        default=Path('configs/mujoco_v16_blocked_bypass_trial.yaml'))
    parser.add_argument('--out', type=Path,
                        default=Path('outputs/diagnostics/v18_camera_overlay_comparison'))
    parser.add_argument('--views', type=int, default=4)
    args = parser.parse_args()
    if not (1 <= args.views <= 12):
        parser.error('--views must be between 1 and 12')
    compare(args.trial, args.config, args.out, args.views)
