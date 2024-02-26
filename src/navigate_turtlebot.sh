#!/bin/bash

# Please provide the following command line arguments:
#
# --wp-model        (choice: gnm_large)
# --pr-model        (choice: cosplace)
# --subgoal-mode    (choice: place_recognition | temporal_distance}])
# --filter-mode     (choice: bayesian | sliding_window)
# --topomap-dir     (name of the directory containing the topomap images)
#
# E.g. --wp-model gnm_large --pr-model cosplace --subgoal-mode place_recognition --filter-mode bayesian --topomap-dir test_dir


# Get default values and the paths to config files
source configs.sh

# Create a new tmux session
session_name="placenav_turtlebot$(date +%s)"
tmux new-session -d -s $session_name

# Split the window into five panes
tmux selectp -t 0    # select the first (0) pane
tmux splitw -h -p 50 # split it into two halves
tmux selectp -t 0    # select the first (0) pane
tmux splitw -v -p 50 # split it into two halves
#
tmux selectp -t 2    # select the new, second (2) pane
tmux splitw -v -p 50 # split it into two halves
#
tmux selectp -t 3    # select the new, third (3) pane
tmux splitw -v -p 50 # split it into two halves
tmux selectp -t 0    # go back to the first pane


# Run the roslaunch command in the first pane
tmux select-pane -t 0
tmux send-keys "roslaunch launch/turtlebot.launch camera_config:=$camera_config_path joystick_config:=$joy_config_path" Enter

# Run the navigate.py script with command line args in the second pane
tmux select-pane -t 1
tmux send-keys "python3 placenav/navigate.py \
    --robot $robot \
    --robot-config-path $robot_config_path \
    --topomap-base-dir $topomap_images_base_dir \
    --model-weight-dir $model_weight_dir \
    --model-config-path $model_config_path \
    $@ \
    " Enter

# Run the teleop.py script in the third pane
tmux select-pane -t 2
tmux send-keys "python3 placenav/joy_teleop.py \
    --robot $robot \
    --robot-config-path $robot_config_path \
    --joy-config-path $joy_config_path" \
    Enter

# Run the pd_controller.py script in the fourth pane
tmux select-pane -t 3
tmux send-keys "python3 placenav/pd_controller.py --robot $robot --robot-config-path $robot_config_path" Enter

# Run the visualization node in the fifth pane
tmux select-pane -t 4
tmux send-keys "python3 placenav/visualization_node.py \
    --robot $robot \
    --robot-config-path $robot_config_path \
    --topomap-base-dir $topomap_images_base_dir \
    --cam-cal-path $camera_calibration_path \
    --camera-config-path $camera_config_path \
    --display-waypoints \
    --save \
    --show \
    $@ \
    " Enter

# Attach to the tmux session
tmux -2 attach-session -t $session_name
