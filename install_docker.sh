#!/bin/bash

IMAGETYPE=$1

ROOT_DIR="$(cd $( dirname ${BASH_SOURCE[0]} ) && pwd)"


${ROOT_DIR}/fetch_irli.sh $3 $2
IRLI_PATH=$(cat $ROOT_DIR/.irli_path)

cd ${IRLI_PATH} && git checkout $2
cd dockerstuff/scripts


./docker_build.sh ${IMAGETYPE}
./build_irli.sh irli:${IMAGETYPE}
