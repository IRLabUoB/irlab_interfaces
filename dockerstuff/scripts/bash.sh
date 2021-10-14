#!/bin/bash

# Description: starts container and opens an interactive bash shell using image tag passed as parameter
# Usage: ./bash.sh <docker-image-tag>
# Example: ./bash.sh dev:melodic-cuda

DOCKER_IMAGE=$1
WORK_DIR="${HOME}/catkin_ws/"
ROOT_DIR="$(cd $( dirname ${BASH_SOURCE[0]} ) && pwd)"
echo "ROOT_DIR: ${ROOT_DIR}"

if [ -z "$DOCKER_IMAGE" ]
then
      echo "usage: ./bash.sh <docker-image-tag>"
      echo "example: ./bash.sh dev:melodic-cuda"
      echo "to list built docker images run: docker images"
      exit 1
fi

shopt -s expand_aliases
source $HOME/.bashrc
source ${ROOT_DIR}/irli_aliases.sh

xdocker run -it \
       --user=$(id -u) \
       --env="DISPLAY" \
       --network="host" \
       --ulimit rtprio=99 \
       --cap-add=sys_nice \
       --privileged \
       --env="QT_X11_NO_MITSHM=1" \
       --workdir="/home/$USER/Projects/irlab_ws" \
       --volume="${ROOT_DIR}/avahi-configs:/etc/avahi" \
       --volume="/home/$USER:/home/$USER" \
       --volume="/etc/group:/etc/group:ro" \
       --volume="/etc/passwd:/etc/passwd:ro" \
       --volume="/etc/shadow:/etc/shadow:ro" \
       --volume="/etc/sudoers.d:/etc/sudoers.d:rw" \
       --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
       --volume="${WORK_DIR}:/home/Projects" ${extra_params} \
       $DOCKER_IMAGE \
       bash 
