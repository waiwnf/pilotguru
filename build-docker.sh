#!/bin/bash

if [ -z "$1" ]; then
  echo "Need docker config name, main or dev, as an argument."
else
  docker build -t $USER/pilotguru-$1 docker/$1
fi
