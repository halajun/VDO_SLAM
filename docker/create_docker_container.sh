#!/usr/bin/env bash

CONTAINER_NAME=docker_vdo_slam
CONTAINER_IMAGE_NAME=docker_vdo_slam

### EDIT THIS TO WHEREVER YOU'RE STORING YOU DATA ###
# folder should exist before you mount it
LOCAL_DATA_FOLDER=/media/jesse/T73/datasets
LOCAL_SSH_KEY_FOLDER=~/.ssh


CONTAINER_DATA_FOLDER=/root/data
CONTAINER_SSH_FOLDER=/root/.ssh

echo "Mounting data folder (local) $LOCAL_DATA_FOLDER -> (container) $CONTAINER_DATA_FOLDER"

docker create --privileged \
            --net=host \
            --pid=host \
            --name=$CONTAINER_NAME \
            -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 \
            -v /tmp/.X11-unix:/tmp/.X11-unix \
            -v /var/run/docker.sock:/var/run/docker.sock \
            -v $LOCAL_DATA_FOLDER:$CONTAINER_DATA_FOLDER \
            -v $LOCAL_SSH_KEY_FOLDER:$CONTAINER_SSH_FOLDER \
            -it \
            $CONTAINER_IMAGE_NAME \
            bash

sudo xhost +local:`sudo docker inspect --format='{{ .Config.Hostname }}' $CONTAINER_NAME`
