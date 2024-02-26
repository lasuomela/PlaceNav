
# ROS
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
import cv_bridge
bridge = cv_bridge.CvBridge()

# pytorch
import torch
import torch.nn as nn
from torchvision import transforms

import numpy as np
import cv2
from typing import List, Tuple
import yaml

# models
from placenav.models.wp_models.gnm.gnm import GNM


def load_model(
    model_path: str,
    model_params: dict,
    device: torch.device = torch.device("cpu"),
) -> nn.Module:
    """Load a model from a checkpoint file (works with models trained on multiple GPUs)"""
    checkpoint = torch.load(model_path, map_location=device)
    loaded_model = checkpoint["model"]

    if model_params["model_type"] == "gnm":
        model = GNM(
            model_params["context_size"],
            model_params["len_traj_pred"],
            model_params["learn_angle"],
            model_params["obs_encoding_size"],
            model_params["goal_encoding_size"],
        )
    else:
        raise ValueError(f"Invalid model type: {model_params['model_type']}")
    try:
        state_dict = loaded_model.module.state_dict()
        model.load_state_dict(state_dict)
    except AttributeError as e:
        state_dict = loaded_model.state_dict()
        model.load_state_dict(state_dict)
    model.to(device)
    return model


def to_numpy(tensor):
    return tensor.cpu().detach().numpy()

def read_image(path):
    mode = cv2.IMREAD_COLOR
    image = cv2.imread(str(path), mode)
    if image is None:
        raise ValueError(f"Cannot read image {path}.")
    if len(image.shape) == 3:
        image = image[:, :, ::-1]  # BGR to RGB
    return image

def get_image_transform(image_size: List[int]) -> transforms.Compose:
    image_size = image_size[::-1] # torchvision's transforms.Resize expects [height, width]
    return transforms.Compose(
        [
            transforms.ToTensor(),
            transforms.Resize(image_size, antialias=True),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
        ]
    )

def odometry_msg_to_numpy(msg: Odometry):
    """Converts a ROS Odometry message to a numpy array of the form [x, y, yaw]"""

    w, x, y, z = msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z
    yaw = np.arctan2(2.0 * (w * z + x * y), w * w + x * x - y * y - z * z)
    return np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, yaw]).astype(float)

def yaw_rotmat(yaw: float) -> np.ndarray:
    return np.array(
        [
            [np.cos(yaw), -np.sin(yaw), 0.0],
            [np.sin(yaw), np.cos(yaw), 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=object,  # get rid of warning
    )

def calculate_sin_cos(waypoints: torch.Tensor) -> torch.Tensor:
    """
    Calculate sin and cos of the angle

    Args:
        waypoints (torch.Tensor): waypoints
    Returns:
        torch.Tensor: waypoints with sin and cos of the angle
    """

    assert waypoints.shape[1] == 3
    angle_repr = torch.zeros_like(waypoints[:, :2])
    angle_repr[:, 0] = torch.cos(waypoints[:, 2])
    angle_repr[:, 1] = torch.sin(waypoints[:, 2])
    return torch.concat((waypoints[:, :2], angle_repr), axis=1)

def to_local_coords(
    positions: np.ndarray, curr_pos: np.ndarray, curr_yaw: float
) -> np.ndarray:
    """
    Convert positions to local coordinates

    Args:
        positions (np.ndarray): positions to convert
        curr_pos (np.ndarray): current position
        curr_yaw (float): current yaw
    Returns:
        np.ndarray: positions in local coordinates
    """
    rotmat = yaw_rotmat(curr_yaw)
    if positions.shape[-1] == 2:
        rotmat = rotmat[:2, :2]
    elif positions.shape[-1] == 3:
        pass
    else:
        raise ValueError

    return (positions - curr_pos).dot(rotmat)

def waypoints_to_ros_msg(waypoints: np.ndarray) -> List[Point]:
    """Converts a numpy array of waypoints to a list of ROS Point messages"""
    return [Point(x=x, y=y, z=0) for x, y, _, _ in waypoints]

def project_waypoint(wp: Point, camera_height: float, K: np.ndarray, D: np.ndarray) -> Tuple[int, int]:
    '''
    Project waypoint to image plane using camera intrinsics and distortion coefficients
    '''
    # Add the height of camera from ground and flip axes to opencv convention (x right, y down, z forward from image plane)
    wp_3d = np.expand_dims(np.array([-wp.y, camera_height, wp.x], dtype=np.float64), axis=0)
    t = np.array([0,0,0], dtype=np.float64)
    R = t
    wp_image, J = cv2.projectPoints(wp_3d, R, t, K, D)
    wp_image = wp_image.squeeze().astype(int)
    return wp_image[0], wp_image[1]