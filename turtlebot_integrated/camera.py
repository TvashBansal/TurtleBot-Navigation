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

# Optical coordinates: x right, y down, z forward. Keep native rendered
# orientation so bearing signs are physical; v0.3 corrected the marker UVs.
def _detect_marker_corners(gray: np.ndarray, marker_id: int, mode: str):
    """Return corners in original pixel coordinates, or None.

    Upscaling and contrast enhancement operate on the *same real camera frame*,
    not on synthetic ground-truth bearings or marker poses. A detected ArUco ID
    is still required; the PnP geometry is unchanged.
    """
    if mode not in ("standard", "multiscale"):
        raise ValueError("camera_detection_mode must be standard or multiscale")
    dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    params=cv2.aruco.DetectorParameters()
    detector=cv2.aruco.ArucoDetector(dictionary,params)
    attempts=[(gray,1.0)]
    if mode=="multiscale":
        # Upsampling can help tags that occupy too few source pixels; it cannot
        # recover a marker hidden behind a wall or entirely outside the image.
        clahe=cv2.createCLAHE(clipLimit=2.0,tileGridSize=(8,8)).apply(gray)
        attempts.extend([
            (clahe,1.0),
            (cv2.resize(gray,None,fx=2,fy=2,interpolation=cv2.INTER_CUBIC),2.0),
            (cv2.resize(clahe,None,fx=2,fy=2,interpolation=cv2.INTER_CUBIC),2.0),
        ])
    for view,scale in attempts:
        corners,ids,_=detector.detectMarkers(view)
        if ids is None:continue
        selected=np.where(ids.flatten()==marker_id)[0]
        if len(selected):
            return np.asarray(corners[int(selected[0])][0],dtype=np.float32)/scale
    return None

def marker_from_rgb(rgb:np.ndarray,c:Config,marker_id:int=0,
                    camera_matrix_override:np.ndarray | None=None,distortion:np.ndarray | None=None):
    if not hasattr(cv2,'aruco'):
        raise RuntimeError('OpenCV ArUco not available; install opencv-contrib-python')
    if rgb is None or rgb.ndim!=3 or rgb.shape[-1]!=3:
        raise ValueError('Expected RGB image with dimensions H x W x 3')
    gray=cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY)
    image_corners=_detect_marker_corners(gray,marker_id,c.camera_detection_mode)
    if image_corners is None:return None
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
