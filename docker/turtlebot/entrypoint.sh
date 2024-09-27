#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/noetic/setup.bash" 
source "/opt/turtlebot_ws/devel/setup.bash"

# Check if the viz_msgs package has been built. If not, build it.
if [ ! -d "/opt/placenav/src/placenav_viz_msgs/devel" ]; then
    echo "Building placenav_viz_msgs package..."
    cd /opt/placenav/src/placenav_viz_msgs
    catkin_make
fi
source "/opt/placenav/src/placenav_viz_msgs/devel/setup.bash"
exec "$@"
