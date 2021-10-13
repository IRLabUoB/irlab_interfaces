#!/bin/bash

# INSTALL_TYPE=
INSTALL_FROM_HOST=false

# checking if this script is being run from the curl-based install script or locally from host machine
# parameter base is different if run locally
key=$0

if [ "$key" == "./install.sh" ]
then
	INSTALL_FROM_HOST=true
else
	# compatibility with curl-based install
	args=$@
	vals=( "$0" ${args[@]} )
	set -- "${vals[@]}"
fi


BRANCH="main"
WORKSPACE_PATH="$HOME/Projects/irlab_ws/"

while [[ $# -gt 0 ]]; do

	key="$1"
	value="$2"

    case $key in
        -w|--workspace)
        WORKSPACE_PATH="$value"
        shift # past argument
        shift # past value
        ;;
        -b|--irli_branch)
        BRANCH="$value"
        shift # past argument
        shift # past value
        ;;   
        *)    # unknown option
        INSTALL_TYPE+=("$key") # save it in an array for later
        shift # past argument
        ;;
    esac
done

echo "INSTALL_TYPE: $INSTALL_TYPE"
echo "WORKSPACE_PATH: $WORKSPACE_PATH"
echo "GIT BRANCH: $BRANCH"


if [ "$INSTALL_FROM_HOST" == "true" ] && [ -z $WORKSPACE_PATH ]
then
	echo "WORKSPACE_PATH is empty: will prompt to use default, i.e. $WORKSPACE_PATH"
fi


if [ "$INSTALL_FROM_HOST" == "true" ]
then
	echo "Local install..."
	ROOT_DIR="$(cd $( dirname ${BASH_SOURCE[0]} ) && pwd)"
	${ROOT_DIR}/install_on_host.sh $BRANCH $WORKSPACE_PATH
else
	if [ -z "$INSTALL_TYPE" ] || [ "$INSTALL_TYPE" == "bash" ]
	then 
	    echo "usage: ./install.sh <install-options> <image-type>"
	    echo "example: ./install.sh --workspace $HOME/Projects/ melodic-dev"
	    echo "install options: --branch: IRLab Interfaces git branch"
	    echo "                 --workspace: absolute path to create workspace (Default: $HOME/Projects/aml_ws)"
		exit 1
	fi
	echo "Curl-based install..."
	rm -rf /tmp/install_irli
	git clone --depth 1 -b install https://github.com/IRLabUoB/irlab_interfaces.git /tmp/install_irli
	cd /tmp/install_irli
	./install_docker.sh $INSTALL_TYPE $BRANCH $WORKSPACE_PATH
fi

