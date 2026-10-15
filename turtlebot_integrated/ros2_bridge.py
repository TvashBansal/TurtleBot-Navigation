"""Physical TurtleBot3 ROS 2 adapter.

DO NOT run --enable-motion until physical topic names, wheel parameters, transforms,
marker location, failsafes and free space have been checked with the lab demonstrator.
Default: read-only dry run, publishes ZERO velocity.

Run: python -m turtlebot_integrated.ros2_bridge --config configs/default.yaml
"""
import argparse
import math
import time
from pathlib import Path
import numpy as np
from .camera import marker_from_rgb
from .config import Config
from .ekf import PoseEKF
from .hardware import SensorBridge,SafetyGate
from .navigation import Navigator


def start_bridge(c:Config,enable_motion:bool,left_name:str,right_name:str):
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import JointState,LaserScan,Image,CameraInfo,Imu
        from geometry_msgs.msg import Twist
        from cv_bridge import CvBridge
    except ImportError as ex:
        raise RuntimeError('ROS 2 runtime unavailable. Source /opt/ros/<distro>/setup.bash and install rclpy, cv_bridge, sensor_msgs, geometry_msgs.') from ex

    class Bridge(Node):
        def __init__(self):
            super().__init__('turtlebot_visual_docking')
            self.c=c
            self.sensors=SensorBridge(c)
            self.safety=SafetyGate(c)
            self.ekf=PoseEKF(c,np.array(c.start_pose))
            self.navigator=Navigator(c)
            self.prev=None
            self.cv=CvBridge()
            self.cam_matrix=None;self.dist_coeff=None
            self.last_info=None
            self.publish=self.create_publisher(Twist,'/cmd_vel',10)
            self.create_subscription(JointState,'/joint_states',self.on_encoder,qos_profile_sensor_data)
            self.create_subscription(LaserScan,'/scan',self.on_scan,qos_profile_sensor_data)
            self.create_subscription(Imu,'/imu',self.on_imu,qos_profile_sensor_data)
            self.create_subscription(Image,'/camera/image_raw',self.on_image,qos_profile_sensor_data)
            self.create_subscription(CameraInfo,'/camera/camera_info',self.on_info,qos_profile_sensor_data)
            self.t0=time.monotonic()
            self.create_timer(c.dt,self.tick)
            self.get_logger().warn('READ-ONLY / ZERO MOTION' if not enable_motion else 'PHYSICAL MOTION ENABLED: ensure supervised, clear laboratory space and emergency stop access')

        def on_encoder(self,msg):
            try:
                names=list(msg.name)
                i,j=names.index(left_name),names.index(right_name)
                self.sensors.encoder_update(msg.position[i],msg.position[j],time.monotonic())
            except (ValueError,IndexError):
                self.get_logger().warn(f'Expected wheels {left_name},{right_name}; /joint_states supplies {list(msg.name)}',throttle_duration_sec=5.0)

        def on_scan(self,msg):
            self.sensors.scan_update(msg.ranges,msg.angle_min,msg.angle_increment,msg.range_max,time.monotonic())

        def on_imu(self,msg):
            # Assumes IMU +Z corresponds to base_link yaw. Verify frame and sign
            # in the lab before enabling physical motion.
            self.sensors.imu_update(msg.angular_velocity.z,time.monotonic())

        def on_info(self,msg):
            if len(msg.k)==9:
                self.cam_matrix=np.asarray(msg.k,dtype=float).reshape(3,3)
                self.dist_coeff=np.asarray(msg.d,dtype=float)

        def on_image(self,msg):
            try:
                rgb=self.cv.imgmsg_to_cv2(msg,desired_encoding='rgb8')
                det=marker_from_rgb(rgb,c,camera_matrix_override=self.cam_matrix,distortion=self.dist_coeff)
                self.sensors.camera_update(det,time.monotonic())
            except Exception as ex:
                self.get_logger().warn(f'Camera decoding/detection failed: {ex}',throttle_duration_sec=5.0)

        def zero(self):
            msg=Twist()
            self.publish.publish(msg)

        def tick(self):
            now=time.monotonic()
            packet=self.sensors.packet()
            status=self.sensors.status(now)
            if packet is None or not(status.encoder_ready and status.scan_ready):
                self.zero();return
            if self.prev is not None:
                gyro_delta=(packet.gyro_z_rad_s*c.dt if packet.gyro_z_rad_s is not None else None)
                self.ekf.predict(packet.encoder_left-self.prev[0],packet.encoder_right-self.prev[1],
                                 gyro_delta_yaw=gyro_delta)
            self.prev=(packet.encoder_left,packet.encoder_right)
            self.ekf.update_camera(packet.camera,c.marker_xy)
            (v,w),target=self.navigator.tick(now-self.t0,self.ekf.x.copy(),packet)
            v,w,reason=self.safety.apply(v,w,status,self.ekf.x,packet.camera is not None,enable_motion)
            out=Twist()
            out.linear.x=float(v);out.angular.z=float(w)
            self.publish.publish(out)
            if self.navigator.done:self.zero()
            if int((now-self.t0)*10)%50==0:
                self.get_logger().info(f'Pose estimate {np.round(self.ekf.x,2).tolist()} target={target} gate={reason}')

        def destroy_node(self):
            for _ in range(3):self.zero()
            return super().destroy_node()

    rclpy.init()
    node=Bridge()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser=argparse.ArgumentParser(description='ROS 2 TurtleBot3 integration: SAFE DRY-RUN by default')
    parser.add_argument('--config',default='configs/default.yaml')
    parser.add_argument('--enable-motion',action='store_true')
    parser.add_argument('--left-joint',default='wheel_left_joint')
    parser.add_argument('--right-joint',default='wheel_right_joint')
    args=parser.parse_args()
    if args.enable_motion:
        print('WARNING: physical robot will move. Ensure hardware E-stop/supervisor and validate all frame and topic settings.')
    start_bridge(Config.load(args.config),args.enable_motion,args.left_joint,args.right_joint)

if __name__=='__main__':main()
