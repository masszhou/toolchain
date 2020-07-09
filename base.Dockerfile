# Build:
#     docker build -t masszhou/builder-base:0.1 -f Dockerfile.base .
#
# Inspection:
#     docker run -t -i masszhou/builder-base:0.1 /bin/bash 

ARG cuda_version=10.2
ARG cudnn_version=7
ARG ubuntu=18.04
FROM nvidia/cuda:${cuda_version}-cudnn${cudnn_version}-devel-ubuntu${ubuntu}
LABEL maintainer="Zhiliang Zhou <zhouzhiliang@gmail.com>"

USER root
WORKDIR /root

RUN apt-get -y update && apt-get -y upgrade
RUN apt-get -y install \
    apt-utils\
    ssh \
    openssh-server\
    build-essential \
    g++ \
    gcc \
    clang \
    gdb \
    gdbserver\
    git \
    wget \
    rsync \
    python3 \
    python3-pip \
    python3-setuptools \
    pkg-config \
    vim

# Install CMake 3.14

ARG CMAKE=cmake-3.14.1.tar.gz
RUN wget https://github.com/Kitware/CMake/releases/download/v3.14.1/${CMAKE}
RUN tar xvzf ${CMAKE} \
    && cd cmake* \
    && ./bootstrap --parallel=$(nproc)
RUN cd cmake* \
    && make -j$(nproc) \
    && make install
RUN cd \
    && rm -r cmake-3.14.1 \
    && rm ${CMAKE}


