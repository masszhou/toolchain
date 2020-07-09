# info of base image:
#     nvidia/cuda, 10.2-cudnn7-devel-ubuntu18.04
#
# Build:
#     docker build -t masszhou/toolchains:dev-ros-melodic-gpu -f ros_gpu.Dockerfile .
#
# Inspection:
#     docker run -t -i masszhou/toolchains:dev-ros-melodic-gpu /bin/bash
#
# Notes:
#    1. ros-melodic-desktop has opencv 3.2 included
#
# ROS master:
#     docker run -it --net=host masszhou/dev-fsd:2020-02-cpu /bin/bash -c "roscore"
#
# ROS app:
#     docker run -v /host/directory:/container/directory -it --gpus all masszhou/toolchains:dev-ros-melodic-gpu /bin/bash -c "rosrun <app>"
#     docker run -v /media/zzhou/data-passat/:/media -it --net=host masszhou/toolchains:dev-ros-melodic-gpu -c "rosbag play -l /media/klettwitz.bag"

FROM masszhou/toolchains:builder-base-0.1
LABEL maintainer="Zhiliang Zhou <zhouzhiliang@gmail.com>"
ENV ROS_WS=/home/catkin_ws
ENV ROS_DISTRO=melodic
WORKDIR $ROS_WS

ENV DEBIAN_FRONTEND=noninteractive
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

RUN pip install catkin_tools

# setup entrypoint, remeber chmod +x ros_entrypoint.sh on Host
COPY ./ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]