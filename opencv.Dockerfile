# reference:
#     https://www.pyimagesearch.com/2018/05/28/ubuntu-18-04-how-to-install-opencv/
#
# Build:
#     docker build -t masszhou/builder-opencv:4.2.0 -f Dockerfile.opencv .
#
# Inspection:
#     docker run -t -i masszhou/builder-opencv:4.2.0 /bin/bash

FROM masszhou/builder-base:0.1
LABEL maintainer="Zhiliang Zhou <zhouzhiliang@gmail.com>"

RUN apt-get update

RUN apt-get install -y \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libv4l-dev \
    libxvidcore-dev \
    libx264-dev \
    libgtk-3-dev \
    libtbb2 \
    libtbb-dev \
    libeigen3-dev \
    libatlas-base-dev \
    gfortran

RUN rm -rf /var/lib/apt/lists/*

ARG OPENCV_VERSION=4.2.0
RUN wget https://github.com/Itseez/opencv/archive/${OPENCV_VERSION}.tar.gz \ 
    && tar -xvf ${OPENCV_VERSION}.tar.gz \
    && rm ${OPENCV_VERSION}.tar.gz \
    && mkdir -p opencv-${OPENCV_VERSION}/build

WORKDIR /root/opencv-${OPENCV_VERSION}/build

RUN cmake -DCMAKE_BUILD_TYPE=RELEASE \
    -DWITH_OPENEXR=OFF \
    -DWITH_WEBP=OFF \
    -DWITH_OPENCL=OFF \
    -DWITH_CUDA=OFF \
    -DCMAKE_INSTALL_PREFIX=/usr/local/opencv \
    .. \
    && make -j$(nproc) && make install \
    && cd /root && rm -rf opencv-${OPENCV_VERSION}

CMD [ "/bin/bash", "-" ]