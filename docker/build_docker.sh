#!/usr/bin/env bash

CHECKOUT_HASH=$1

if [ -z $CHECKOUT_HASH ]; then
    CHECKOUT_HASH=master
fi

docker build --build-arg CHECKOUT_HASH=$CHECKOUT_HASH -f Dockerfile.vdo_slam -t docker_vdo_slam .