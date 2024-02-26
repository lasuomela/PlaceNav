#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/noetic/setup.bash" 
source "/opt/turtlebot_ws/devel/setup.bash"
source "/opt/placenav/src/placenav_viz_msgs/devel/setup.bash"
exec "$@"
