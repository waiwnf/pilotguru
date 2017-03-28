#!/bin/bash

if [ -z "$1" ]; then
  echo "Need docker config name, main or dev, as an argument."
else
  docker run --privileged --rm -ti \
      --network=host \
      -e DISPLAY=$DISPLAY \
      -v /tmp/.X11-unix:/tmp/.X11-unix \
      -v ${HOME}/.Xauthority:/root/.Xauthority \
      -v `pwd`:/opt/pilotguru \
      -w /opt/pilotguru \
      $USER/pilotguru-$1
fi
