#!/bin/bash

# Populate the config path variables
source configs.sh

# Create a new tmux session
session_name="record_bag_$(date +%s)"
tmux new-session -d -s $session_name

# Get the camera and odometry topics from the robot config file
camera_topic=$(yq -r .${robot}.camera_topic $robot_config_path)
odometry_topic=$(yq -r .${robot}.odom_topic $robot_config_path)

# Split the window into three panes
tmux selectp -t 0    # select the first (0) pane
tmux splitw -v -p 50 # split it into two halves
tmux selectp -t 0    # go back to the first pane
tmux splitw -h -p 50 # split it into two halves

# Run the roslaunch command in the first pane
tmux select-pane -t 0
tmux send-keys "roslaunch launch/turtlebot.launch camera_config:=$camera_config_path joystick_config:=$joy_config_path" Enter

# Run the teleop.py script in the second pane
tmux select-pane -t 1
tmux send-keys "python3 placenav/joy_teleop.py \
    --robot $robot --robot-config-path $robot_config_path --joy-config-path $joy_config_path" Enter

# Change the directory to ../topomaps/bags and run the rosbag record command in the third pane
tmux select-pane -t 2
tmux send-keys "cd topomaps/bags" Enter
tmux send-keys "rosbag record $camera_topic $odometry_topic -o $1"

# Attach to the tmux session
tmux -2 attach-session -t $session_name
