"""ArUco image detection, calibrated pinhole projection, camera-to-base translation."""
import math
import numpy as np
import cv2
from .config import Config
from .geometry import wrap
from .types import CameraDetection

def camera_intrinsics(c:Config):
    fy=c.camera_height/(2*math.tan(math.radians(c.camera_fovy_deg)/2))
    return np.array([[fy,0,c.camera_width/2],[0,fy,c.camera_height/2],[0,0,1]],float)

# OpenCV camera is x right, y down, z forward. MuJoCo rendered frames are flipped vertically
# if necessary by the calling backend. We use the ArUco marker's four known-size corners.
def marker_from_rgb(rgb:np.ndarray,c:Config,marker_id:int=0,
                    camera_matrix_override:np.ndarray | None=None,distortion:np.ndarray | None=None):
    if not hasattr(cv2,'aruco'):
        raise RuntimeError('OpenCV ArUco not available; install opencv-contrib-python')
    gray=cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY)
    dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    detector=cv2.aruco.ArucoDetector(dictionary,cv2.aruco.DetectorParameters())
    corners,ids,_=detector.detectMarkers(gray)
    if ids is None:return None
    selected=np.where(ids.flatten()==marker_id)[0]
    if len(selected)==0:return None
    image_corners=np.asarray(corners[int(selected[0])][0],dtype=np.float32)
    half=c.camera_marker_size*.5
    object_points=np.array([[-half,half,0],[half,half,0],[half,-half,0],[-half,-half,0]],np.float32)
    ok,rvec,tvec=cv2.solvePnP(object_points,image_corners,
                                  camera_intrinsics(c) if camera_matrix_override is None else camera_matrix_override,
                                  np.zeros(5) if distortion is None else distortion,flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:return None
    # Robot x forward, y left. Camera +z forward, +x right = robot -y.
    forward=float(tvec[2,0]);lateral=float(-tvec[0,0])
    # Camera extrinsic translation from optical center to base_link.
    forward+=c.camera_offset_x_m
    return CameraDetection(math.hypot(forward,lateral),wrap(math.atan2(lateral,forward)),image_corners)

def make_marker(path:str, marker_id:int=0):
    dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    marker=cv2.aruco.generateImageMarker(dictionary,marker_id,400)
    # Thick white margin around tag helps visual segmentation and supports UV mapping.
    marker=cv2.copyMakeBorder(marker,16,16,16,16,cv2.BORDER_CONSTANT,value=255)
    if not cv2.imwrite(str(path),marker):raise OSError(f'Cannot write {path}')
