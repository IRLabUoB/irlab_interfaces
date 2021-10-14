# FROM osrf/ros:melodic-desktop-full as base

ARG UBUNTU_VERSION=18.04

ARG ARCH=
ARG CUDA=10.1

FROM nvidia/cuda${ARCH:+-$ARCH}:${CUDA}-base-ubuntu${UBUNTU_VERSION}
ENV DEBIAN_FRONTEND noninteractive
# ARCH and CUDA are specified again because the FROM directive resets ARGs
# (but their default value is retained if set previously)
ARG ARCH
ARG CUDA
ARG CUDNN=7.6.4.38-1
ARG CUDNN_MAJOR_VERSION=7
ARG LIB_DIR_PREFIX=x86_64
ARG LIBNVINFER=6.0.1-1
ARG LIBNVINFER_MAJOR_VERSION=6

# Needed for string substitution
SHELL ["/bin/bash", "-c"]

# RUN echo "Acquire::http { Proxy \"$http_proxy\"; };" >> /etc/apt/apt.conf

# Pick up some TF dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        cuda-command-line-tools-${CUDA/./-} \
        # There appears to be a regression in libcublas10=10.2.2.89-1 which
        # prevents cublas from initializing in TF. See
        # https://github.com/tensorflow/tensorflow/issues/9489#issuecomment-562394257
        libcublas10=10.2.1.243-1 \ 
        cuda-nvrtc-${CUDA/./-} \
        cuda-cufft-${CUDA/./-} \
        cuda-curand-${CUDA/./-} \
        cuda-cusolver-${CUDA/./-} \
        cuda-cusparse-${CUDA/./-} \
        curl \
        libcudnn7=${CUDNN}+cuda${CUDA} \
        libfreetype6-dev \
        libhdf5-serial-dev \
        libzmq3-dev \
        pkg-config \
        software-properties-common \
        unzip

# Install TensorRT if not building for PowerPC
RUN [[ "${ARCH}" = "ppc64le" ]] || { apt-get update && \
        apt-get install -y --no-install-recommends libnvinfer${LIBNVINFER_MAJOR_VERSION}=${LIBNVINFER}+cuda${CUDA} \
        libnvinfer-plugin${LIBNVINFER_MAJOR_VERSION}=${LIBNVINFER}+cuda${CUDA} \
        && apt-get clean \
        && rm -rf /var/lib/apt/lists/*; }

# For CUDA profiling, TensorFlow requires CUPTI.
ENV LD_LIBRARY_PATH /usr/local/cuda/extras/CUPTI/lib64:/usr/local/cuda/lib64:$LD_LIBRARY_PATH

# Link the libcuda stub to the location where tensorflow is searching for it and reconfigure
# dynamic linker run-time bindings
RUN ln -s /usr/local/cuda/lib64/stubs/libcuda.so /usr/local/cuda/lib64/stubs/libcuda.so.1 \
    && echo "/usr/local/cuda/lib64/stubs" > /etc/ld.so.conf.d/z-cuda-stubs.conf \
    && ldconfig

# TODO(angerson) Completely remove Python 2 support
ARG _PY_SUFFIX=
ARG PYTHON=python${_PY_SUFFIX}
ARG PIP=pip

# See http://bugs.python.org/issue19846
ENV LANG C.UTF-8

RUN apt-get update && apt-get install -y \
    ${PYTHON} \
    ${PYTHON}-pip

RUN ${PIP} --no-cache-dir install --upgrade \
    pip \
    setuptools

# Some TF tools expect a "python" binary
RUN ln -s $(which ${PYTHON}) /usr/local/bin/python

# Options:
#   tensorflow
#   tensorflow-gpu
#   tf-nightly
#   tf-nightly-gpu
# Set --build-arg TF_PACKAGE_VERSION=1.11.0rc0 to install a specific version.
# Installs the latest version by default.
ARG TF_PACKAGE=tensorflow
ARG TF_PACKAGE_VERSION=
RUN ${PIP} install --no-cache-dir ${TF_PACKAGE}${TF_PACKAGE_VERSION:+==${TF_PACKAGE_VERSION}}


# install packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN curl -sSL \
'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' \
| apt-key add -

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    && rm -rf /var/lib/apt/lists/*

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# bootstrap rosdep
RUN rosdep init \
    && rosdep update

# install ros packages
ENV ROS_DISTRO melodic
RUN apt-get update && apt-get install -y \
    ros-melodic-ros-core \
&& rm -rf /var/lib/apt/lists/*


# install ros packages
RUN apt-get update && apt-get install -y \
    ros-melodic-ros-base \
&& rm -rf /var/lib/apt/lists/*

# install ros desktop full packages
RUN apt-get update && apt-get install -y \
    ros-melodic-desktop-full \
    && rm -rf /var/lib/apt/lists/*

# install packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

#installs avahi for local network discovery (required for using the real robot)
RUN apt-get update && apt-get install -y \
    avahi-daemon avahi-utils


# Dependencies for baxter, sawyer and franka

RUN apt-get update && \ 
    apt-get -y install sudo nano python-pip python-scipy libprotobuf-dev protobuf-compiler libboost-all-dev \
                       ros-melodic-convex-decomposition ros-melodic-ivcon \
                       git-core python-argparse python-wstool python-vcstools python-rosdep ros-melodic-control-msgs \
                       ros-melodic-joystick-drivers ros-melodic-xacro ros-melodic-tf2-ros ros-melodic-rviz ros-melodic-cv-bridge \
                       ros-melodic-actionlib ros-melodic-actionlib-msgs ros-melodic-dynamic-reconfigure \
                       ros-melodic-trajectory-msgs ros-melodic-moveit \
                       ros-melodic-octomap-rviz-plugins \
                       ros-melodic-control-toolbox  ros-melodic-catkin python-catkin-tools \
                       ros-melodic-rospy-message-converter\
                       ros-melodic-moveit-visual-tools \
                       ros-melodic-realtime-tools ros-melodic-ros-controllers \
                       ros-melodic-tf-conversions ros-melodic-kdl-parser\ 
                       ros-melodic-ros-control ros-melodic-ros-controllers ros-melodic-gazebo-ros-control \
                       ros-melodic-libfranka ros-melodic-franka-ros \
                       build-essential python-dev swig python-pygame && \
    rm -rf /var/lib/apt/lists/*
 #ros-melodic-sns-ik-lib \

RUN pip install --upgrade pip && \
    pip install protobuf && \
    pip install pandas

#six PySide
RUN pip install numpy numpy-quaternion==2020.5.19.15.27.24 pygame decorator ipython jupyter matplotlib Pillow scipy scikit-learn 
    # https://storage.googleapis.com/tensorflow/linux/gpu/tensorflow_gpu-1.4.1-cp27-none-linux_x86_64.whl
# TENSOR FLOW *GPU*

# Install pybullet
RUN pip install pybullet


# ===========================================
# ========= FOR CUDA AND TENSORFLOW =========
# ===========================================


# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

RUN if [[ $HTTP_PROXY != '' ]]; then git config --global http.proxy $HTTP_PROXY; fi

RUN apt-get update && apt-get -y install python-catkin-tools ros-melodic-ros-pytest \ 
    ros-melodic-joy-teleop libcppunit-dev ros-melodic-moveit-resources python-future \
    ros-melodic-rosparam-shortcuts 
    
RUN apt-get update && apt-get -y upgrade

# disable sound card warnings in gazebo
COPY asound.conf /etc/asound.conf

# setup entrypoint, need entrypoint.sh in the same folder with Dockerfile
COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["bash"]