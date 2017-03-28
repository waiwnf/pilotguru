FROM ubuntu:xenial

RUN apt-get update

RUN apt-get -y install \
    git \
    cmake \
    wget \
    libgoogle-glog-dev \
    protobuf-compiler \
    libprotobuf-dev \
    libglew-dev \
    libeigen3-dev \
    libopenblas-dev \
    liblapack-dev \
    libopencv-dev \
    libav-tools

RUN mkdir /opt/pangolin && \
    cd /opt/pangolin && \
    wget https://codeload.github.com/stevenlovegrove/Pangolin/tar.gz/v0.5 && \
    tar -xzf v0.5 && \
    mkdir Pangolin-0.5/build && \
    cd Pangolin-0.5/build && \
    cmake .. && \
    make -j4 && \
    make install

RUN cd /usr/include/ && wget https://github.com/nlohmann/json/releases/download/v2.1.1/json.hpp
