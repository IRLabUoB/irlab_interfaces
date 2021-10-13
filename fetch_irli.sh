#!/bin/bash

ROOT_DIR="$(cd $( dirname ${BASH_SOURCE[0]} ) && pwd)"
CATKIN_WS_PATH=$1
IRLI_BRANCH=$2
SHALLOW_CLONE='n'

if [ -z "$CATKIN_WS_PATH" ]
then
    
    echo "usage: ./fetch_irli.sh [<optional-path-to-catkin-workspace>]"
    CATKIN_WS_PATH=${HOME}/Projects/irlab_ws/
	echo "This script will create a default catkin workpace at ${CATKIN_WS_PATH}, proceed? (y/n)"
	
	read answer
	if echo "$answer" | grep -iq "^y" ;then
		echo "Fetching IRLI..."
	else
	    echo "Exiting script. Operation cancelled. Provide argument for specifying custom workspace location."
	    exit 1
	fi

fi


# Backing up existing irli directory, if it exists
rm -rf irlab_interfaces.bkp > /dev/null 2>&1
mv -f -b ${CATKIN_WS_PATH}/src/irlab_interfaces ${ROOT_DIR}/irlab_interfaces.bkp > /dev/null 2>&1
mkdir -p ${CATKIN_WS_PATH}/src

CATKIN_WS_ABS_PATH="$( cd "$( dirname "${CATKIN_WS_PATH}/." )" && pwd )"
IRLI_ABS_PATH="${CATKIN_WS_ABS_PATH}/src/irlab_interfaces"
echo ${IRLI_ABS_PATH}

CLONE_DEPTH=""
if echo "$SHALLOW_CLONE" | grep -iq "^y" ;then
	CLONE_DEPTH="--depth 1"
fi

git clone ${CLONE_DEPTH} -b ${IRLI_BRANCH} https://github.com/IRLabUoB/irlab_interfaces.git ${IRLI_ABS_PATH}

echo "git clone ${CLONE_DEPTH} -b ${IRLI_BRANCH} https://github.com/IRLabUoB/irlab_interfaces.git ${IRLI_ABS_PATH}"
echo ${IRLI_ABS_PATH} > .irli_path