#!/bin/bash

shopt -s expand_aliases
source $HOME/.bashrc

ROOT_DIR="$(cd $( dirname ${BASH_SOURCE[0]} ) && pwd)"
cd ${ROOT_DIR}

IRLI_PATH=$(cd ../ && pwd)


if [ -z "${IRLI_DIR}" ]
then
	  echo "Setting environment variable IRLI_DIR=${IRLI_PATH}"
	  echo ' ' >> ${HOME}/.bashrc
      echo "export IRLI_DIR=${IRLI_PATH}" >> ${HOME}/.bashrc
else
	  echo "IRLI environment variable already exists and set to IRLI_DIR=${IRLI_PATH}"
fi
