#!/usr/bin/env bash

TAG=$1

if [ -z $TAG ]; then
    echo "Usage: ./docker/build_docker.sh TAG"
    exit -1
fi

docker build -f docker/Dockerfile.vdo_slam --ssh default -t $TAG .
