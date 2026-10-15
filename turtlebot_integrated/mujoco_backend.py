"""Compulsory MuJoCo back end: physics contacts, wheel encoders, ray LiDAR, rendered camera.

NOTE: Requires MuJoCo with a working OpenGL renderer. No hidden kinematic fallback.
"""
import math
import tempfile
from pathlib import Path
import cv2
import numpy as np
from .config import Config
from .camera import make_marker, marker_from_rgb
from .scenarios import Scenario
from .types import Scan,SensorPacket


def scene_xml(c: Config,s:Scenario, marker_path:Path) -> str:
    """Produce an MJCF scene using a floating differential-drive body and wheel joints."""
    walls=''
    xmin,xmax,ymin,ymax=s.bounds
    boxes=[(xmin-.06,xmin+.06,ymin-.1,ymax+.1),
           (xmax-.06,xmax+.06,ymin-.1,ymax+.1),
           (xmin,xmax,ymin-.06,ymin+.06),
           (xmin,xmax,ymax-.06,ymax+.06)]
    for i,rect in enumerate(boxes+s.obstacles):
        x1,x2,y1,y2=rect
        cx,cy=(x1+x2)*.5,(y1+y2)*.5
        sx,sy=max(.01,(x2-x1)*.5),max(.01,(y2-y1)*.5)
        color='0.3 0.35 0.4 1' if i<4 else '0.7 0.45 0.27 1'
        walls+=f'<geom name="obstacle_{i}" type="box" pos="{cx:.4f} {cy:.4f} 0.12" size="{sx:.4f} {sy:.4f} 0.12" rgba="{color}"/>\n'
    lidar_sites='';lidar_sensors=''
    # +Z site direction is a MuJoCo rangefinder ray, rotated horizontal.
    for i,angle in enumerate(np.linspace(-math.pi,math.pi,c.lidar_beams,endpoint=False)):
        co,si=math.cos(angle),math.sin(angle)
        quat=f'{math.sqrt(.5):.7f} {-si*math.sqrt(.5):.7f} {co*math.sqrt(.5):.7f} 0'
        # Ray origin is the physical LiDAR head ABOVE the chassis/bumper.
        # Perimeter sites at chassis height can intersect the robot itself,
        # producing false near-zero ranges and an endless safety-brake turn.
        lidar_sites+=f'<site name="ray_{i:03d}" pos="0 0 0.105" quat="{quat}" size="0.002" rgba="0 0.9 0 0"/>\n'
        lidar_sensors+=f'<rangefinder name="range_{i:03d}" site="ray_{i:03d}"/>\n'
    xml=f'''<mujoco model="turtlebot3_visual_docking">
    <compiler angle="radian"/>
    <option timestep="0.004" gravity="0 0 -9.81" integrator="implicitfast"/>
    <visual><global offwidth="960" offheight="720"/></visual>
    <default><geom friction="1.2 0.005 0.0001" condim="3"/></default>
    <asset>
        <texture name="marker_tex" type="2d" file="{str(marker_path)}"/>
        <material name="marker_mat" texture="marker_tex" texrepeat="1 1" specular="0" shininess="0.02"/>
    </asset>
    <worldbody>
      <light pos="0 0 4" dir="0 0 -1" diffuse="1 1 1"/>
      <camera name="overhead" pos="0 0 5.4" xyaxes="1 0 0 0 1 0" fovy="58" resolution="{c.camera_width} {c.camera_height}"/>
      <geom name="ground" type="plane" pos="0 0 0" size="4 4 0.1" rgba="0.88 0.88 0.87 1"/>
      {walls}
      <body name="dock_marker" pos="{c.marker_xy[0]} {c.marker_xy[1]} 0.19">
        <geom name="marker_plate" type="box" size="0.006 0.13 0.13" rgba="1 1 1 1"/>
        <geom name="marker_face" type="box" pos="-0.007 0 0" size="0.0008 0.1188 0.1188" material="marker_mat" contype="0" conaffinity="0"/>
      </body>
      <body name="base" pos="{s.start.x} {s.start.y} 0.088" euler="0 0 {s.start.theta}">
        <freejoint name="base_freejoint"/>
        <geom name="chassis" type="box" pos="0 0 0.012" size="0.105 0.075 0.027" mass="0.78" rgba="0.13 0.22 0.25 1"/>
        <geom name="front_bumper" type="box" pos="0.108 0 0.001" size="0.012 0.072 0.025" mass="0.025" rgba="0.12 0.35 0.5 1"/>
        <body name="left_wheel" pos="0 0.080 -0.055">
            <joint name="left_joint" type="hinge" axis="0 1 0" damping="0.002"/>
            <geom name="left_wheel_geom" type="cylinder" quat="0.707107 -0.707107 0 0" size="0.033 0.010" mass="0.085" rgba="0.05 0.05 0.07 1" friction="1.8 0.002 0.0001"/>
        </body>
        <body name="right_wheel" pos="0 -0.080 -0.055">
            <joint name="right_joint" type="hinge" axis="0 1 0" damping="0.002"/>
            <geom name="right_wheel_geom" type="cylinder" quat="0.707107 -0.707107 0 0" size="0.033 0.010" mass="0.085" rgba="0.05 0.05 0.07 1" friction="1.8 0.002 0.0001"/>
        </body>
        <geom name="rear_castor" type="sphere" pos="-0.085 0 -0.065" size="0.020" mass="0.012" friction="0.03 0.002 0.0001" rgba="0.6 0.6 0.6 1"/>
        <site name="imu_site" pos="0 0 0.03" size="0.005"/>
        <camera name="front_camera" pos="{c.camera_offset_x_m} 0 {c.camera_height_m-0.088}" xyaxes="0 -1 0 0 0 1" fovy="{c.camera_fovy_deg}" resolution="{c.camera_width} {c.camera_height}"/>
        {lidar_sites}
      </body>
    </worldbody>
    <sensor>
        <jointpos name="enc_left" joint="left_joint"/>
        <jointpos name="enc_right" joint="right_joint"/>
        <gyro name="imu_gyro" site="imu_site"/>
        {lidar_sensors}
    </sensor>
    <actuator>
        <velocity name="motor_left" joint="left_joint" kv="0.15" ctrlrange="-15 15" ctrllimited="true"/>
        <velocity name="motor_right" joint="right_joint" kv="0.15" ctrlrange="-15 15" ctrllimited="true"/>
    </actuator>
</mujoco>'''
    return xml

def make_mujoco_uv_marker(source: Path, destination: Path) -> Path:
    """Compensate for the mirrored planar texture coordinates on marker_face.

    Only the MJCF texture is mirrored. DO NOT mirror the camera frame itself:
    doing so would reverse horizontal image coordinates and camera bearings.
    Keep the original marker image unchanged for standalone OpenCV tests.
    """
    image = cv2.imread(str(source), cv2.IMREAD_UNCHANGED)
    if image is None:
        raise FileNotFoundError(f"Could not read original ArUco image: {source}")
    if not cv2.imwrite(str(destination), cv2.flip(image, 1)):
        raise OSError(f"Could not write MJCF marker texture: {destination}")
    return destination


class MuJoCoBackend:
    def __init__(self,c:Config,s:Scenario,seed:int=7,record_camera:bool=True):
        try:
            import mujoco
        except ImportError as ex:
            raise RuntimeError('MuJoCo is required. Install with: pip install -e . ; check README') from ex
        self.mj=mujoco;self.c=c;self.s=s;self.rng=np.random.default_rng(seed)
        marker_path=Path(__file__).resolve().parent.parent/'assets'/'aruco_0.png'
        if not marker_path.exists():make_marker(str(marker_path))
        # MuJoCo's default box-face UVs mirror the supplied marker texture.
        # The camera probe detected the tag only after reflecting the image;
        # compensate at the *texture* so the camera image coordinates remain
        # physically correct for range/bearing observations and the EKF.
        with tempfile.TemporaryDirectory(prefix='turtlebot_mj_marker_') as tmpdir:
            mj_marker=make_mujoco_uv_marker(marker_path,Path(tmpdir)/'marker_uv.png')
            self.model=mujoco.MjModel.from_xml_string(scene_xml(c,s,mj_marker))
        self.data=mujoco.MjData(self.model)
        self.renderer=mujoco.Renderer(self.model,height=c.camera_height,width=c.camera_width) if record_camera else None
        self.tick=0;self.time=0.0;self.collision=False
        self.scan_angles=np.linspace(-math.pi,math.pi,c.lidar_beams,endpoint=False)
        self.left_sid=mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_SENSOR,'enc_left')
        self.right_sid=mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_SENSOR,'enc_right')
        self.gyro_sid=mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_SENSOR,'imu_gyro')
        self.ray_sids=[mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_SENSOR,f'range_{i:03d}') for i in range(c.lidar_beams)]
        self.body_id=mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_BODY,'base')
        mujoco.mj_forward(self.model,self.data)

    def get_truth(self):
        p=self.data.xpos[self.body_id]
        quat=self.data.xquat[self.body_id]
        # w,x,y,z -> yaw of body relative to world
        w,x,y,z=quat
        yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
        return np.array([p[0],p[1],yaw],float)

    def _sensor(self,sid):
        adr=self.model.sensor_adr[sid]
        return float(self.data.sensordata[adr])

    def render_rgb(self,camera='front_camera'):
        if self.renderer is None:return None
        self.renderer.update_scene(self.data,camera=camera)
        return self.renderer.render().copy()

    def observe(self):
        c=self.c
        d=np.array([self._sensor(i) for i in self.ray_sids],float)
        # Rays now originate at the body-centred LiDAR head, so their measured
        # distances are already base-centred: do NOT add the robot radius.
        d=np.where(d<0,c.lidar_max_range,d)
        d=np.clip(d+self.rng.normal(0,c.lidar_noise*self.s.noise_multiplier,len(d)),.03,c.lidar_max_range)
        camera=None
        if self.renderer is not None and self.tick%5==0 and self.rng.random()>(c.camera_dropout+self.s.camera_dropout_extra):
            frame=self.render_rgb()
            camera=marker_from_rgb(frame,c)
            if camera is not None:
                camera.distance+=float(self.rng.normal(0,c.camera_noise_range*.4*self.s.noise_multiplier))
                camera.bearing+=float(self.rng.normal(0,c.camera_noise_bearing*.4*self.s.noise_multiplier))
        # MuJoCo gyro is a legitimate 3-axis angular-velocity sensor expressed
        # in imu_site's local frame. Using component z is NOT reading true yaw.
        gyro_start=int(self.model.sensor_adr[self.gyro_sid])
        gyro_z=float(self.data.sensordata[gyro_start+2])
        gyro_z+=c.gyro_bias_rad_s
        gyro_z+=float(self.rng.normal(0,c.gyro_noise_rad_s*self.s.noise_multiplier))
        return SensorPacket(self._sensor(self.left_sid),self._sensor(self.right_sid),
                            Scan(d,self.scan_angles,c.lidar_max_range),camera,gyro_z)

    def step(self,v:float,w:float):
        c=self.c
        v=float(np.clip(v,-c.max_speed,c.max_speed));w=float(np.clip(w,-c.max_omega,c.max_omega))
        self.data.ctrl[0]=(v-w*c.wheel_separation/2)/c.wheel_radius
        self.data.ctrl[1]=(v+w*c.wheel_separation/2)/c.wheel_radius
        n=int(round(c.dt/self.model.opt.timestep))
        for _ in range(n):self.mj.mj_step(self.model,self.data)
        self.time=float(self.data.time);self.tick+=1
        # Only metrics/diagnostics consume ground truth here.
        p=self.get_truth()
        self.collision=self.collision or self.s.hit(float(p[0]),float(p[1]),c.robot_radius*.9)

    def close(self):
        if self.renderer is not None:self.renderer.close()
