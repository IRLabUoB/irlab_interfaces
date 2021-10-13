#!/bin/bash

ROOT_DIR="$(cd $( dirname ${BASH_SOURCE[0]} ) && pwd)"

if [ $# -ne 1 ]; then
	echo "usage: ./exec_container.sh <container_id>"
    echo "example: ./exec_container.sh fa11998579c"
    echo "to get current container id, run ./get_containerId.sh script"
  exit 1
fi

shopt -s expand_aliases
source $HOME/.bashrc
source ${ROOT_DIR}/irli_aliases.sh


echo 'Entering container:' $1
xdocker exec -it $1 bash
