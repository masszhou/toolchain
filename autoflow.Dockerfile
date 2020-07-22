# info of base image:
#     nvidia/cuda, 10.2-cudnn7-devel-ubuntu18.04
#
# Build:
#     docker build -t masszhou/toolchains:dev-autoflow -f autoflow.Dockerfile .
#
# Inspection:
#     docker run -t -i masszhou/toolchains:dev-autoflow /bin/bash
#
# Notes:
#    1. ros-melodic-desktop has opencv 3.2 included
#

FROM masszhou/toolchains:dev-ros-melodic-gpu
LABEL maintainer="Zhiliang Zhou <zhouzhiliang@gmail.com>"
ENV ROS_WS=/home/catkin_ws
ENV ROS_DISTRO=melodic
WORKDIR $ROS_WS

# install libtorch dependencies
RUN apt-get -y update && apt-get -y upgrade
RUN apt-get -y install \
    libgflags-dev \
    libgoogle-glog-dev\
    libiomp-dev \
    libopenmpi-dev \
    protobuf-compiler \
    python3-yaml

COPY --from=masszhou/toolchains:dev-torch-1.5.1  /root/libtorch /opt/torch/libtorch