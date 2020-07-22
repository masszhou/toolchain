# original repo: 
#     github:freemanix/torchbuilder
#
# changes:
#     1. use v1.5.1
#     2. fix path error -> cp -r pytorch/build/build/lib libtorch
#
# Build:
#     docker build -t masszhou/toolchains:dev-torch-1.5.1 -f torch.Dockerfile .
#
# Inspection:
#     docker run -t -i masszhou/toolchains:dev-torch-1.5.1 /bin/bash
#
# Export to local:
#     VERSION=1.0.1
#     docker run --rm masszhou/toolchains:dev-torch-1.5.1 tar czf - libtorch >libtorch-${VERSION}.tgz

FROM masszhou/toolchains:dev-cuda-10.2
LABEL maintainer="Zhiliang Zhou <zhouzhiliang@gmail.com>"

USER root
WORKDIR /root

RUN apt-get -y update && apt-get -y upgrade
RUN apt-get -y install \
    libgflags-dev \
    libgoogle-glog-dev\
    libiomp-dev \
    libopenmpi-dev \
    protobuf-compiler \
    python3-yaml

# Intel MKL installation

RUN wget https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS-2019.PUB
RUN apt-key add GPG-PUB-KEY-INTEL-SW-PRODUCTS-2019.PUB && rm GPG-PUB*
RUN sh -c 'echo deb https://apt.repos.intel.com/mkl all main > /etc/apt/sources.list.d/intel-mkl.list'
RUN apt-get update && apt-get -y install intel-mkl-64bit-2019.1-053
RUN rm /opt/intel/mkl/lib/intel64/*.so

# Download and build libtorch with MKL support

ARG TORCH_VERSION=v1.5.1
ENV TORCH_CUDA_ARCH_LIST="5.2 6.0 6.1 7.0 7.5"
ENV TORCH_NVCC_FLAGS="-Xfatbin -compress-all"
RUN git clone --recurse-submodules -j8 https://github.com/pytorch/pytorch.git --branch ${TORCH_VERSION} --single-branch
RUN cd pytorch && mkdir build && cd build && BUILD_TEST=OFF USE_NCCL=OFF python3 ../tools/build_libtorch.py

# Prepare built package

RUN cd && mkdir -p libtorch/include && mkdir -p libtorch/share
RUN cp -r pytorch/build/build/lib libtorch
RUN cp -r pytorch/torch/share/cmake libtorch/share/cmake
RUN for dir in ATen c10 caffe2 torch; do cp -r pytorch/torch/include/$dir libtorch/include; done

CMD [ "/bin/bash", "-" ]
