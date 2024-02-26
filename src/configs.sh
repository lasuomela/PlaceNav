#!/bin/bash

# Get the path of the parent directory of this script
TOP_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Get the robot name from environment variable set by docker container
robot=$ROBOT
robot_config_path="${TOP_DIR}/config/robots.yaml"
joy_config_path="${TOP_DIR}/config/ps4_joy.yaml"
camera_config_path="${TOP_DIR}/config/camera.yaml"
camera_calibration_path="${TOP_DIR}/config/calibration.yaml"

topomap_images_base_dir="${TOP_DIR}/topomaps/images"
model_weight_dir="${TOP_DIR}/model_weights"
model_config_path="${TOP_DIR}/config/models.yaml"

