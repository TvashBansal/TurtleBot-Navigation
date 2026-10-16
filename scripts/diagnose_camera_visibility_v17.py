"""Offline v0.17 ArUco camera opportunity diagnosis for an existing MuJoCo trial.

Reads logged truth ONLY for offline validation. It does not change navigation,
EKF, scene geometry, sensor measurements or online robot state.

If MuJoCo is available, reconstructs approximate planar snapshots at logged
truth poses and tests the actual rendered-image ArUco detector. These images
are intentionally NOT claimed to be exact frames from the completed trial:
logged pose omits physical roll, pitch, suspension and wheel-joint details.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from turtlebot_integrated.config import Config
from turtlebot_integrated.scenarios import get_scenario


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def segment_intersects_rect(a: tuple[float, float], b: tuple[float, float],
                            rect: tuple[float, float, float, float]) -> bool:
    """2D line-segment/axis-aligned rectangle intersection (closed edges)."""
    x1, x2, y1, y2 = rect
    lo = 0.0
    hi = 1.0
    for p, d, mn, mx in ((a[0], b[0] - a[0], x1, x2),
                         (a[1], b[1] - a[1], y1, y2)):
        if abs(d) < 1e-12:
            if p < mn or p > mx:
                return False
            continue
        t1, t2 = (mn - p) / d, (mx - p) / d
        lo = max(lo, min(t1, t2))
        hi = min(hi, max(t1, t2))
        if lo > hi:
            return False
    return True


def visibility_at_pose(xyt: list[float], cfg: Config,
                       obstacles: list[tuple[float, float, float, float]]) -> dict:
    """Geometric opportunity estimate, not a sensor reading or actual detector result."""
    x, y, theta = map(float, xyt[:3])
    camera_xy = (x + cfg.camera_offset_x_m * math.cos(theta),
                 y + cfg.camera_offset_x_m * math.sin(theta))
    landmark_xy = tuple(cfg.marker_xy)
    dx, dy = landmark_xy[0] - camera_xy[0], landmark_xy[1] - camera_xy[1]
    dist = math.hypot(dx, dy)
    bearing = wrap(math.atan2(dy, dx) - theta)
    half_vertical = math.radians(cfg.camera_fovy_deg) / 2
    half_horizontal = math.atan(math.tan(half_vertical) * cfg.camera_width / cfg.camera_height)
    elevation = math.atan2(0.19 - cfg.camera_height_m, max(dist, 1e-6))
    occluded_by = [i for i, r in enumerate(obstacles)
                   if segment_intersects_rect(camera_xy, landmark_xy, r)]
    in_range = dist < cfg.camera_max_range
    in_fov = abs(bearing) < half_horizontal and abs(elevation) < half_vertical
    # Margin excludes cases where a tag is likely cropped near the image edge.
    well_inside_fov = abs(bearing) < 0.75 * half_horizontal and abs(elevation) < 0.75 * half_vertical
    opportunity = bool(in_range and well_inside_fov and not occluded_by)
    return dict(range_m=round(dist, 4), bearing_deg=round(math.degrees(bearing), 2),
                elevation_deg=round(math.degrees(elevation), 2),
                horizontal_half_fov_deg=round(math.degrees(half_horizontal), 2),
                in_range=bool(in_range), in_fov=bool(in_fov),
                well_inside_fov=bool(well_inside_fov),
                occluded_obstacle_indices=occluded_by, opportunity=opportunity)


def evenly_spaced_indices(values: list[int], n: int) -> list[int]:
    if n <= 0 or not values:
        return []
    if len(values) <= n:
        return values
    return sorted(set(values[round(i * (len(values) - 1) / (n - 1))]
                      for i in range(n))) if n > 1 else [values[-1]]


def render_reconstructed_views(cfg: Config, scenario_name: str, seed: int,
                               rows: list[dict], selected_indices: list[int],
                               out: Path) -> list[dict]:
    import cv2
    import numpy as np
    from turtlebot_integrated.camera import marker_from_rgb
    from turtlebot_integrated.mujoco_backend import MuJoCoBackend
    import mujoco

    sim = MuJoCoBackend(cfg, get_scenario(scenario_name, seed), seed, record_camera=True)
    info = []
    try:
        jid = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_JOINT, 'base_freejoint')
        adr = int(sim.model.jnt_qposadr[jid])
        initial_z = float(sim.data.qpos[adr + 2])
        for i in selected_indices:
            row = rows[i]
            x, y, yaw = map(float, row['truth'][:3])
            sim.data.qpos[adr:adr+3] = [x, y, initial_z]
            sim.data.qpos[adr+3:adr+7] = [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
            sim.data.qvel[:] = 0.0
            sim.data.xfrc_applied[:] = 0.0
            mujoco.mj_forward(sim.model, sim.data)
            rgb = sim.render_rgb('front_camera')
            detection = marker_from_rgb(rgb, cfg)
            # Overlays are added AFTER detection; they do not affect its result.
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            label = (f"t={float(row.get('t', 0)):.2f}s reconstructed; "
                     f"ArUco={'YES' if detection is not None else 'NO'}")
            cv2.putText(bgr, label, (12, 25), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, (30, 220, 30) if detection is not None else (40, 40, 220), 2)
            path = out / f'camera_reconstruction_{i:04d}.png'
            if not cv2.imwrite(str(path), bgr):
                raise OSError(f'Could not write {path}')
            info.append(dict(index=i, t_s=round(float(row.get('t', 0)), 3),
                             detected=bool(detection is not None),
                             range_m=round(float(detection.distance), 3) if detection is not None else None,
                             bearing_deg=round(math.degrees(float(detection.bearing)), 2) if detection is not None else None,
                             file=str(path)))
    finally:
        sim.close()
    return info


def diagnose(trial: Path, config: Path, max_images: int = 8,
             render: bool = True) -> dict:
    summary = json.loads((trial / 'summary.json').read_text(encoding='utf-8'))
    rows = json.loads((trial / 'trajectory.json').read_text(encoding='utf-8'))
    if not rows or not isinstance(rows, list):
        raise ValueError('Nonempty trajectory list expected')
    if summary.get('backend') != 'mujoco':
        raise ValueError('Requires an actual MuJoCo run, not a kinematic test double')
    cfg = Config.load(config)
    scenario_name = str(summary['scenario'])
    scenario = get_scenario(scenario_name, int(summary['seed']))
    # Camera rendering is attempted on sim.observe() when tick % 5 == 0;
    # row i reports pose AFTER step i, so these are near (not identical to)
    # the actual observation poses.
    potential = []
    for i, row in enumerate(rows):
        if i % 5 != 0:
            continue
        geo = visibility_at_pose(row['truth'], cfg, scenario.obstacles)
        potential.append(dict(index=i, t_s=round(float(row['t']), 3),
                              logged_camera_seen=bool(row.get('camera_seen', False)),
                              **geo))
    opportunities = [r for r in potential if r['opportunity']]
    out = trial / 'camera_visibility_v17'
    out.mkdir(parents=True, exist_ok=True)
    # Render a time-diverse set, preferring real visual opportunities.
    selected = evenly_spaced_indices([r['index'] for r in opportunities], max_images)
    if not selected:
        selected = evenly_spaced_indices([r['index'] for r in potential], max_images)
    reconstructions = []
    render_error = None
    if render:
        try:
            reconstructions = render_reconstructed_views(
                cfg, scenario_name, int(summary['seed']), rows, selected, out)
        except Exception as exc:
            render_error = f'{type(exc).__name__}: {exc}'
    first_opportunity = opportunities[0] if opportunities else None
    last_opportunity = opportunities[-1] if opportunities else None
    final_truth = rows[-1]['truth']
    final_estimate = rows[-1]['estimate']
    position_bias = math.dist(final_truth[:2], final_estimate[:2])
    report = dict(trial=str(trial), config=str(config), scenario=scenario_name,
                  seed=int(summary['seed']), steps=len(rows),
                  sampled_observation_slots=len(potential),
                  geometry_opportunities=len(opportunities),
                  observed_camera_measurements=int(summary.get('measurements', 0)),
                  accepted_camera_updates=int(summary.get('accepted_camera_updates', 0)),
                  first_geometry_opportunity=first_opportunity,
                  last_geometry_opportunity=last_opportunity,
                  final_ekf_truth_xy_difference_m=round(position_bias, 4),
                  reconstructed_views=reconstructions,
                  reconstructed_detections=sum(r['detected'] for r in reconstructions),
                  render_error=render_error,
                  notes=[
                      'Visibility estimates use recorded ground truth ONLY in post-run diagnosis.',
                      '2D line-of-sight and FOV estimates ignore physical pitch, roll, the full marker footprint, and camera dropout.',
                      'Reconstructed camera images use recorded x/y/yaw with nominal base z and zero pitch/roll; they are NOT actual recorded trial frames.',
                      'All offline geometry and rendering must remain outside the online controller/EKF.',
                  ])
    (out / 'report.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print('=== V0.17 CAMERA VISIBILITY / FINAL DOCK DIAGNOSIS ===')
    print(f"Trial: {scenario_name} seed={summary['seed']}, steps={len(rows)}")
    print(f'Camera observation slots: {len(potential)}')
    print(f'Approximate unobstructed geometric opportunities: {len(opportunities)}')
    print(f"Actual camera observations / EKF accepted: {report['observed_camera_measurements']} / {report['accepted_camera_updates']}")
    print(f"Final EKF-versus-truth XY difference: {report['final_ekf_truth_xy_difference_m']:.4f} m")
    print('First opportunity:', first_opportunity)
    print('Last opportunity:', last_opportunity)
    print(f"Reconstructed static images: {len(reconstructions)}, detected in {report['reconstructed_detections']}")
    if render_error:
        print('Image reconstruction unavailable:', render_error)
    print('Report:', out / 'report.json')
    print('WARNING: Off-line diagnostic; reconstructed static images are not original trial frames.')
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trial', type=Path, help='Completed MuJoCo run directory')
    parser.add_argument('--config', type=Path,
                        default=Path('configs/mujoco_v16_blocked_bypass_trial.yaml'))
    parser.add_argument('--max-images', type=int, default=8)
    parser.add_argument('--no-render', action='store_true')
    args = parser.parse_args()
    if args.max_images < 0 or args.max_images > 24:
        parser.error('--max-images must be between 0 and 24')
    diagnose(args.trial, args.config, args.max_images, not args.no_render)


if __name__ == '__main__':
    main()
