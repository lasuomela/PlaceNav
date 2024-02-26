#!/bin/sh
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

# Get the current script directory
SCRIPT=$(readlink -f "$0")
SCRIPT_DIR=$(dirname "$SCRIPT")

# Check if the udev rules are installed
if [ ! -f /etc/udev/rules.d/60-kobuki.rules ]; then
    echo 'Installing the udev rules for the kobuki base'
    wget -P /etc/udev/rules.d https://raw.githubusercontent.com/kobuki-base/kobuki_ftdi/devel/60-kobuki.rules
    echo 'If building for the first time, please reboot to update udev rules'
    exit 1
fi

# Attempt to pull the image first from Docker Hub
docker pull $DOCKER_IMAGE_ID

# Check if the image was pulled successfully
if [ $? -eq 0 ]; then
    echo "Image $DOCKER_IMAGE_ID found on Docker Hub."
else
    echo "Image $DOCKER_IMAGE_ID not found on Docker Hub. Building locally..."
    # Build the image locally
    DOCKER_BUILDKIT=1 docker build \
        -t $DOCKER_IMAGE_ID \
        -f Dockerfile ${SCRIPT_DIR}/../..
fi
