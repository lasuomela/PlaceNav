#!/bin/bash
usage() { echo "Usage: $0 [-r <dockerhub_repo>] [-t <tag>] [-i <image>]" 1>&2; exit 1; }

# Defaults
REPO="lasuomela/"
DOCKER_IMAGE_NAME="placenav_turtlebot_noetic"
TAG="0.0.1"
ROBOT="turtlebot"

# Parse arguments
while getopts ":h:r:t:i:" opt; do
  case $opt in
    h)
      usage
      exit
      ;;
    r)
      REPO=$OPTARG
      ;;
    t)
      TAG=$OPTARG
      ;;
    i)
      DOCKER_IMAGE_NAME=$OPTARG
      ;;
    \?)
      echo "Invalid option: -$OPTARG" >&2
      exit 1
      ;;
    :)
      echo "Option -$OPTARG requires an argument." >&2
      exit 1
      ;;
  esac
done
shift $((OPTIND-1))

DOCKER_IMAGE_ID="${REPO}${DOCKER_IMAGE_NAME}:${TAG}"
echo "Using $DOCKER_IMAGE_ID"

SCRIPT=$(readlink -f "$0")
CWD=$(dirname "$SCRIPT")

xhost + local:
docker run \
    -it --rm \
    --privileged \
    --net=host \
    -e ROBOT="${ROBOT}" \
    -e SDL_VIDEODRIVER='x11' \
    -e DISPLAY=$DISPLAY \
    --mount "type=bind,src=$CWD/../../,dst=/opt/placenav" \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /etc/localtime:/etc/localtime \
    -v /dev/input/:/dev/input/ \
    -v /dev/bus/usb:/dev/bus/usb \
    -v /dev/kobuki:/dev/kobuki \
    -v /dev:/dev \
    --device-cgroup-rule='c 189:* rmw' \
    --shm-size=8gb \
    --gpus 'all,"capabilities=graphics,utility,display,video,compute"' \
    "$DOCKER_IMAGE_ID" "$@" 
xhost - local: 
