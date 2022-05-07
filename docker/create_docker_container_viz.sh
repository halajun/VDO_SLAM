#!/usr/bin/env bash

CONTAINER_NAME=docker_vdo_slam
CONTAINER_IMAGE_NAME=docker_vdo_slam

### EDIT THIS TO WHEREVER YOU'RE STORING YOU DATA ###
# folder should exist before you mount it
LOCAL_DATA_FOLDER=/media/jesse/T73/datasets
LOCAL_SSH_KEY_FOLDER=~/.ssh


CONTAINER_DATA_FOLDER=/root/data
CONTAINER_SSH_FOLDER=/root/.ssh


xhost +local:root

USE_NVIDIA=false

# If we are running in a docker-in-docker scenario NVIDIA_SOS will be populated
# and we dont need to check for gpu or roscore
if [ -z "$NVIDIA_SOS" ]; then
    # Is there an nvidia card in the system?
    if [[ $(lshw -C display 2> /dev/null | grep -i NVIDIA) ]]; then
        echo "Nvidia GPU in the system, trying to use it."
        # If we have nvidia-container-runtime...
        if nvidia-container-runtime -v > /dev/null; then
            # Check if we have an nvidia driver
            NVIDIA_DRIVER_VERSION=`nvidia-smi --query-gpu=driver_version --format=csv,noheader | awk '{ print substr($0, 0, 4) }'`
            if [ -z "$NVIDIA_DRIVER_VERSION" ]; then
                echo "No Nvidia driver version found, but nvidia-container-runtime is installed... can you run 'nvidia-smi?"
                echo "You may want to install a driver to take advantage of it."
            else
                USE_NVIDIA=true
            fi
        else
            echo "Warning:"
            echo "The system has a Nvidia GPU but doesn't have nvidia-container-runtime installed"
            echo "You may want to install it to take advantage of it, e.g.:"
            echo "sudo apt-get install nvidia-container-runtime"
        fi
        if ! "$USE_NVIDIA"; then
            echo "Using non-nvidia alternative."
        fi
    fi
else
    USE_NVIDIA=true
    DOCKER_IN_DOCKER=true
fi


XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth

echo "Mounting data folder (local) $LOCAL_DATA_FOLDER -> (container) $CONTAINER_DATA_FOLDER"


if [ -z "$NVIDIA_SOS" ]; then
    # Compose the long list of .so files from Nvidia to make X work with seawolf
    # This provides a list of all .so files installed from packages from 'nvidia-''
    NVIDIA_SOS=$(dpkg -l | grep nvidia- | awk '{print $2}' | xargs dpkg-query -L | grep lib | grep .so)
    # This generates --volume /path/to/.so:/path/to/.so lines for every .so
fi
for so in $NVIDIA_SOS; do DOCKER_NVIDIA_SO_VOLUMES+="--volume $so:$so "; done

if "$USE_NVIDIA"; then
    # If executing this script from an unmanned shell (like from seawolf ShellCmd)
    # We can't use "-i"
    if [ -t 1 ]; then
        TERMINAL_FLAGS='-it'
    else
        TERMINAL_FLAGS='-t'
    fi
    # Create the container based on the launchfile it's launching (if any)
    # removes '.launch' from the last argument
    echo "Container name will be: $CONTAINER_NAME"
    docker run $DOCKER_NVIDIA_SO_VOLUMES \
        --privileged \
        -i -d --gpus all \
        --volume $XSOCK:$XSOCK:rw \
        -v $LOCAL_DATA_FOLDER:$CONTAINER_DATA_FOLDER \
        -v $LOCAL_SSH_KEY_FOLDER:$CONTAINER_SSH_FOLDER \
        -v /var/run/docker.sock:/var/run/docker.sock \
        --env DISPLAY=$DISPLAY \
        --env XAUTHORITY=$XAUTH \
        --env QT_X11_NO_MITSHM=0 \
        --env QT_X11_NO_XRENDER=0 \
        --volume $XAUTH:$XAUTH:rw \
        --net host \
        --pid host \
        -it \
        --name=$CONTAINER_NAME \
        $CONTAINER_IMAGE_NAME "$@"
fi

xhost +local:`docker inspect --format='{{ .Config.Hostname }}' $CONTAINER_NAME`
# xhost -local:root
