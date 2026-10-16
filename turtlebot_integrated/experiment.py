"""Run closed-loop experiments; truth is only used for logging/scoring.
The controller and navigator never receive the truth state.
"""
from dataclasses import asdict
from datetime import datetime, timezone
import importlib.metadata
import platform
import subprocess
import sys
from pathlib import Path
import json
import math
import numpy as np
from .config import Config
from .ekf import PoseEKF
from .navigation import Navigator
from .kinematic import KinematicTestBackend
from .scenarios import get_scenario
from .types import StepLog
from .geometry import wrap


def _git_head() -> str | None:
    result = subprocess.run(['git', 'rev-parse', 'HEAD'], capture_output=True, text=True, check=False)
    return result.stdout.strip() if result.returncode == 0 else None


def _versions() -> dict:
    versions = {}
    for package in ('numpy', 'scipy', 'matplotlib', 'mujoco', 'opencv-contrib-python', 'imageio'):
        try:
            versions[package] = importlib.metadata.version(package)
        except importlib.metadata.PackageNotFoundError:
            versions[package] = None
    return versions


def run_trial(cfg:Config,backend:str,scenario_name:str,seed:int,outdir:Path, capture_frames:bool=False):
    scenario=get_scenario(scenario_name,seed)
    if backend=='mujoco':
        from .mujoco_backend import MuJoCoBackend
        sim=MuJoCoBackend(cfg,scenario,seed)
    elif backend=='kinematic':
        sim=KinematicTestBackend(cfg,scenario,seed)
    else:raise ValueError('Backend must be mujoco or kinematic')
    outdir.mkdir(parents=True,exist_ok=True)
    (outdir/'config_snapshot.json').write_text(json.dumps(asdict(cfg),indent=2)+'\n',encoding='utf-8')
    run_metadata = dict(
        backend=backend, scenario=scenario_name, seed=seed,
        created_utc=datetime.now(timezone.utc).isoformat(),
        git_commit_at_run=_git_head(),
        python=sys.version.split()[0], platform=platform.platform(),
        dependencies=_versions(), video_requested=bool(capture_frames),
        sensor_note=('Kinematic test double: synthetic marker and ray measurements'
                     if backend=='kinematic' else 'MuJoCo: rendered RGB image and simulated sensors'),
    )
    (outdir/'run_metadata.json').write_text(json.dumps(run_metadata,indent=2)+'\n',encoding='utf-8')
    ekf=PoseEKF(cfg,np.array(cfg.start_pose))
    nav=Navigator(cfg)
    prev=None
    logs=[]
    writer=None
    if capture_frames and backend=='mujoco':
        try:
            import imageio.v2 as imageio
            writer=imageio.get_writer(str(outdir/'demo_capture.mp4'),fps=2,codec='libx264')
        except Exception as ex:
            (outdir/'video_error.txt').write_text('MP4 initialization failed: '+repr(ex),encoding='utf-8')
    steps=int(cfg.max_duration/cfg.dt)
    try:
        for k in range(steps):
            packet=sim.observe()
            if prev is not None:
                gyro_delta=(packet.gyro_z_rad_s*cfg.dt if packet.gyro_z_rad_s is not None else None)
                ekf.predict(packet.encoder_left-prev[0],packet.encoder_right-prev[1],
                            gyro_delta_yaw=gyro_delta)
            prev=(packet.encoder_left,packet.encoder_right)
            accepted=ekf.update_camera(packet.camera,cfg.marker_xy,scenario.noise_multiplier)
            (v,w),target=nav.tick(sim.time,ekf.x.copy(),packet)
            # Physics backend and test double receive the same wheel velocity commands.
            sim.step(v,w)
            truth=sim.get_truth() # metrics ONLY: not used by EKF/controller/navigator
            goal=np.array(cfg.dock_pose)
            logs.append(StepLog(
                t=sim.time,truth=truth.tolist(),estimate=ekf.x.tolist(),
                uncertainty=np.sqrt(np.maximum(np.diag(ekf.P),0)).tolist(),cmd=[v,w],
                target=list(target),camera_seen=packet.camera is not None,
                distance_to_goal=float(np.linalg.norm(truth[:2]-goal[:2])),
                collision=sim.collision,innovation_nis=ekf.last_nis,
                gyro_z_rad_s=packet.gyro_z_rad_s,gyro_used=ekf.last_used_gyro,
                camera_update_accepted=bool(accepted),
                odom_minus_gyro_delta_yaw_rad=ekf.last_odom_minus_gyro,
                nav_stop_reason=nav.last_stop_reason,
                front_clearance_m=(nav.last_front_clearance_m
                                   if math.isfinite(nav.last_front_clearance_m) else None),
                waypoint_index=nav.idx,
                target_heading_error_rad=nav.last_target_bearing_rad,
                planner_status=nav.last_planner_status,
                planner_path_nodes=nav.last_path_nodes,
                planner_visible_nodes=nav.last_visible_nodes,
                planner_target_distance_m=nav.last_target_distance_m))
            if writer is not None and k%6==0:
                try:
                    from .video import diagnostic_frame
                    camera=sim.render_rgb(camera='front_camera')
                    overhead=sim.render_rgb(camera='overhead')
                    writer.append_data(diagnostic_frame(overhead,camera,logs[-1]))
                except Exception as ex:
                    (outdir/'video_error.txt').write_text('Video render failed: '+repr(ex),encoding='utf-8')
                    writer.close();writer=None
            if nav.done:break
        endtruth=sim.get_truth()
        endest=ekf.x
        errors=np.array([[log.truth[0]-log.estimate[0],log.truth[1]-log.estimate[1],
                          wrap(log.truth[2]-log.estimate[2])] for log in logs])
        rmse=np.sqrt(np.mean(errors**2,axis=0))
        goal=np.array(cfg.dock_pose)
        position_error=float(np.linalg.norm(endtruth[:2]-goal[:2]))
        heading_error=abs(wrap(endtruth[2]-goal[2]))
        success=bool(nav.done and position_error<cfg.dock_tolerance+.06 and heading_error<.25 and not sim.collision)
        from collections import Counter
        planner_counts=Counter(x.planner_status for x in logs)
        summary=dict(backend=backend,scenario=scenario_name,seed=seed,success=success,
            planner_reported_done=bool(nav.done),collision=bool(sim.collision),
            duration_s=round(sim.time,3),position_error_m=round(position_error,4),
            heading_error_rad=round(heading_error,4),
            rmse_x_m=round(float(rmse[0]),4),rmse_y_m=round(float(rmse[1]),4),
            rmse_heading_rad=round(float(rmse[2]),4),
            camera_detection_rate=round(float(np.mean([a.camera_seen for a in logs])),3),
            measurements=int(sum(x.camera_seen for x in logs)),
            accepted_camera_updates=int(sum(x.camera_update_accepted for x in logs)),
            gyro_used_steps=int(sum(x.gyro_used for x in logs)),
            planner_states=dict(planner_counts),
            planner_no_route_fraction=round(float(sum(
                x.planner_status in {'no_astar_route','no_visible_onward_path'} for x in logs
            )/len(logs)),3),
            steps=len(logs),warning=('KINEMATIC TEST DOUBLE ONLY; NOT ASSESSMENT SIMULATION' if backend=='kinematic' else 'MuJoCo physics and rendered-camera trial'))
        if backend=='mujoco':
            summary['disturbance_applied_steps']=int(sim.disturbance_applied_steps)
        (outdir/'summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
        (outdir/'trajectory.json').write_text(json.dumps([asdict(a) for a in logs]),encoding='utf-8')
        from .reporting import plot_trial
        plot_trial(logs,summary,outdir)
        return summary
    finally:
        if writer is not None:writer.close()
        if hasattr(sim,'close'):sim.close()
