#!/bin/bash

# Get the paths to config files
source configs.sh

# Create a new tmux session
session_name="topomap_creation_$(date +%s)"
tmux new-session -d -s $session_name

# Split the window into three panes
tmux selectp -t 0    # select the first (0) pane
tmux splitw -v -p 50 # split it into two halves
tmux selectp -t 0    # go back to the first pane
tmux splitw -h -p 50 # split it into two halves

# Run roscore in the first pane
tmux select-pane -t 0
tmux send-keys "roscore" Enter

# Run the create_topomap.py script with command line args in the second pane
tmux select-pane -t 1
tmux send-keys "python3 placenav/create_topomap.py --dt 1 --route_name $1 --robot ${robot} --robot_config_path $robot_config_path --topomap_directory $topomap_images_base_dir" Enter

# Change the directory to ../topomaps/bags and run the rosbag play command in the third pane
tmux select-pane -t 2
tmux send-keys "cd topomaps/bags" Enter
tmux send-keys "rosbag play -r 5 $2" # feel free to change the playback rate to change the edge length in the graph

# Attach to the tmux session
tmux -2 attach-session -t $session_name
