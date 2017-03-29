#!/bin/bash

mkdir -p data/orb-vocabulary
cd data/orb-vocabulary
wget https://github.com/raulmur/ORB_SLAM2/raw/master/Vocabulary/ORBvoc.txt.tar.gz
tar -xzf ORBvoc.txt.tar.gz
cd ..
