#!/usr/bin/env bash

CONTAINER_NAME=docker_vdo_slam
CONTAINER_IMAGE_NAME=docker_vdo_slam

### EDIT THIS TO WHEREVER YOU'RE STORING YOU DATA ###
# folder should exist before you mount it
LOCAL_DATA_FOLDER=/media/jesse/T7/datasets


CONTAINER_DATA_FOLDER=/root/data

echo "Mounting data folder (local) $LOCAL_DATA_FOLDER -> (container) $CONTAINER_DATA_FOLDER"


docker create --privileged \
            --net=host \
            --pid=host \
            --name=$CONTAINER_NAME \
            -v /var/run/docker.sock:/var/run/docker.sock \
            -v $LOCAL_DATA_FOLDER:$CONTAINER_DATA_FOLDER \
            -it \
            $CONTAINER_IMAGE_NAME \
            bash