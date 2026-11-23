#!/bin/bash

# Based on the ETH Robotics Summer school docker:
# https://github.com/ETHZ-RobotX/smb_docker/

# If not working, first do: sudo rm -rf /tmp/.docker.xauth
# It still not working, try running the script as root.

# Default options
DOCKER=coin_bievr_ros2
DOCKERFILE=Dockerfile_ros2
NAME=COIN-BIEVR-ROS2
BUILD=false
NO_CACHE=""

# The image is built from the repository root, so it contains the checked-out code.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"

help()
{
    echo "Usage: run_docker_ros2.sh [ -b | --build ] [ -c | --no-cache ] [ -n | --name <docker name> ]
               [ -h | --help  ]
  -b, --build     (Re)build the image from the current checkout before running.
  -c, --no-cache  With -b: rebuild every layer, e.g. to pick up new upstream
                  versions of the dependencies (Ceres, Livox)."
    exit 2
}

SHORT=b,c,n:,h
LONG=build,no-cache,name:,help
OPTS=$(getopt -a -n run_docker --options $SHORT --longoptions $LONG -- "$@")
echo $OPTS

eval set -- "$OPTS"

while :
do
  case "$1" in
    -b | --build )
      BUILD="true"
      shift
      ;;
    -c | --no-cache )
      NO_CACHE="--no-cache"
      shift
      ;;
    -n | --name )
      NAME="$2"
      shift 2
      ;;
    -h | --help)
      help
      ;;
    --)
      shift;
      break
      ;;
    *)
      echo "Unexpected option: $1"
      help
      ;;
  esac
done

if [ "$BUILD" = true ]; then
    echo "Building docker: $DOCKERFILE as $DOCKER"
    docker build $NO_CACHE --network=host --progress=plain \
        --build-arg USER_ID=$(id -u) --build-arg GROUP_ID=$(id -g) \
        -f "$SCRIPT_DIR/$DOCKERFILE" -t $DOCKER "$REPO_ROOT" || exit 1
fi

XAUTH=/tmp/.docker.xauth

echo "Preparing Xauthority data..."
xauth_list=$(xauth nlist :0 | tail -n 1 | sed -e 's/^..../ffff/')
if [ ! -f $XAUTH ]; then
    if [ -n "$xauth_list" ]; then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

echo "Done."
echo ""
echo "Verifying file contents:"
file $XAUTH
echo "--> It should say \"X11 Xauthority data\"."
echo ""
echo "Permissions:"
ls -FAlh $XAUTH
echo ""
echo "Running docker..."

docker run -it --rm \
    --env="DISPLAY=$DISPLAY" \
    --volume=/home/$USER/data:/home/coin_bievr/data \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --net=host \
    --privileged \
    --name=$NAME \
    ${DOCKER} \
    bash

echo "Done."