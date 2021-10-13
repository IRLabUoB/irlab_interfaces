#!/bin/bash

DOCKER_IMAGE=$1
WORK_DIR="${HOME}/Projects/"

if [ -z "$DOCKER_IMAGE" ]
then
      echo "usage: ./bash.sh <docker-image-tag>"
      echo "example: ./bash.sh dev:melodic-cuda"
      echo "to list built docker images run: docker images"
      exit 1
fi

# Running container and giving access to X11 in a safer way
docker run -it \
       --user=$(id -u) \
       --env="DISPLAY" \
       --network="host" \
       --env="QT_X11_NO_MITSHM=1" \
       --workdir="/home/$USER/Projects" \
       --volume="/home/$USER:/home/$USER" \
       --volume="/etc/group:/etc/group:ro" \
       --volume="/etc/passwd:/etc/passwd:ro" \
       --volume="/etc/shadow:/etc/shadow:ro" \
       --volume="/etc/sudoers.d:/etc/sudoers.d:ro" \
       --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
       --volume="${WORK_DIR}:/home/Projects" \
       $DOCKER_IMAGE \
       bash -c "cd irlab_ws/src/irlab_interfaces && source build_ws.sh && source src/irlab_interfaces/configure_env.sh"

echo "If build succeeded, make sure to edit the 'your_ip' variable in the files 'franka.sh', 'intera.sh', and 'baxter.sh' to be able to connect to the robots via the network. These files can be found in the root of the newly created catkin workspace."
