# info of base image:
#     ubuntu:18.04
#
# Build:
#     docker build -t masszhou/toolchains:dev-ros-melodic-cpu -f ros_cpu.Dockerfile .
#
# Inspection:
#     docker run -t -i masszhou/toolchains:dev-ros-melodic-cpu /bin/bash
#
# Notes:
#    1. ros-melodic-desktop has opencv 3.2 included
#
# ROS master:
#     docker run -it --net=host masszhou/dev-fsd:2020-02-cpu /bin/bash -c "roscore"
#
# ROS app:
#     docker run -v /host/directory:/container/directory -it masszhou/dev-fsd:2020-02-cpu /bin/bash -c "rosbag play -l /media/klettwitz.bag"
#     docker run -v /media/zzhou/data-passat/:/media -it --net=host masszhou/dev-fsd:2020-02-cpu /bin/bash -c "rosbag play -l /media/klettwitz.bag"

FROM ubuntu:18.04
LABEL maintainer="Zhiliang Zhou <zhouzhiliang@gmail.com>"

ENV ROS_WS=/home/catkin_ws
ENV ROS_DISTRO=melodic
WORKDIR $ROS_WS

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get -y update \
    && apt-get -y install \
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
    cmake

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros-latest.list' \
    && apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
    && apt-get update \
    && apt-get install -y \
    ros-melodic-desktop \
    ros-melodic-socketcan-interface \
    ros-melodic-socketcan-bridge \
    ros-melodic-gps-common \
    ros-melodic-pcl-ros \
    freeglut3-dev \
    libpcap-dev \
    libyaml-cpp-dev \
    python-pip

# delete all the apt list files since they're big and get stale quickly
RUN rm -rf /var/lib/apt/lists/*

# install catkin tools
RUN pip install catkin_tools

# setup entrypoint, remeber chmod +x ros_entrypoint.sh on Host
COPY ./ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]