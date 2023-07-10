#!/bin/bash

# Name and base and options
IMAGE_NAME=epfl-lasa/dual_arm_control_iam                 # Chose any name for your image (but make sure to report it in start_docker)
ROS_DISTRO=melodic                                        # Possible: noetic, melodic

# Help
HELP_MESSAGE="Usage: ./build.sh [-r, --rebuild] [-v, --verbose] [-i, --image-name] [-d, --distro] [--smid]
Build the '${IMAGE_NAME}' image.
Options:
  -r, --rebuild            Rebuild with --no-cache option.
  -v, --verbose            Display additional info upon build.
  -i, --image-name         Defines the name given to the image beeing built. Default: ${IMAGE_NAME}
  -d, --distro             Can be \"noetic\" or \"melodic\", default: ${ROS_DISTRO}
  -h, --help               Show this help message."

# Parse build flags
BUILD_FLAGS=()
while [ "$#" -gt 0 ]; do
  case "$1" in
  -r | --rebuild)
    BUILD_FLAGS+=(--no-cache)
    shift 1
    ;;
  -v | --verbose)
    BUILD_FLAGS+=(--progress=plain)
    shift 1
    ;;
  -i | --image-name)
    IMAGE_NAME=$2
    shift 2
    ;;
  -d | --distro)
    ROS_DISTRO=$2
    if [[ "$ROS_DISTRO" != "noetic" && "$ROS_DISTRO" != "melodic" ]] ; then
      echo -e "\033[31mERROR: Distro \"$ROS_DISTRO\" is not supported"; \
      exit 1;
    fi
    shift 2
    ;;
  -h | --help)
    echo "${HELP_MESSAGE}"
    exit 0
    ;;
  *)
    echo "Unknown option: $1" >&2
    exit 1
    ;;
  esac
done


# Setup build flags
BUILD_FLAGS+=(--build-arg ROS_DISTRO="${ROS_DISTRO}")
BUILD_FLAGS+=(-t "${IMAGE_NAME}:${ROS_DISTRO}")
BUILD_FLAGS+=(--build-arg HOST_GID=$(id -g))   # Pass the correct GID to avoid issues with mounted volumes
BUILD_FLAGS+=(--ssh default="${SSH_AUTH_SOCK}")
BUILD_FLAGS+=(--build-arg GIT_NAME=$(git config user.name))    # Pass git user info to be able to pull
BUILD_FLAGS+=(--build-arg GIT_EMAIL=$(git config user.email))

DOCKER_BUILDKIT=1 docker build "${BUILD_FLAGS[@]}" -f ./docker/Dockerfile .