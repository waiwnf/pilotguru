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

RUN apt-get -y install curl
RUN mkdir /opt/install && \
    cd /opt/install && \
    curl https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > microsoft.gpg && \
    mv microsoft.gpg /etc/apt/trusted.gpg.d/microsoft.gpg
RUN echo "deb [arch=amd64] http://packages.microsoft.com/repos/vscode stable main" > /etc/apt/sources.list.d/vscode.list
RUN apt-get update

RUN apt-get -y install \
    gdb \
    code \
    libxss1 \
    xterm \
    sudo \
    libgconf-2-4 \
    valgrind \
    clang-format 
