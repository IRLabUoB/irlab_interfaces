#!/bin/bash


DOCKER_FILE_PATH="../dockerfiles/$1"

if [ ! -d "$DOCKER_FILE_PATH" ]
then
      echo "usage: ./docker_build.sh <path-to-docker-file>"
      echo "example: ./docker_build.sh melodic-cuda"
      exit 1
fi

# echo $DOCKER_FILE_PATH

# if [[ -z ${http_proxy} ]]; then
#   HTTP_PROXY="127.0.0.1"
# else
#   HTTP_PROXY=${http_proxy}
# fi

# if [[ -z ${https_proxy} ]]; then
#   HTTPS_PROXY="127.0.0.1"
# else
#   HTTPS_PROXY=${https_proxy}
# fi

#cp -r avahi-configs ${DOCKER_FILE_PATH}
docker build ${DOCKER_FILE_PATH} --network="host" -t irli:${DOCKER_FILE_PATH##*/} 