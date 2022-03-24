#!/usr/bin/env bash

CONTAINER_NAME=docker_vdo_slam
CONTAINER_IMAGE_NAME=docker_vdo_slam

docker create --privileged \
            --net=host \
            --pid=host \
            --name=$CONTAINER_NAME \
            -v /var/run/docker.sock:/var/run/docker.sock \
            -it \
            $CONTAINER_IMAGE_NAME \
            bash