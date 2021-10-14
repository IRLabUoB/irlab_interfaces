FROM ubuntu:bionic

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && apt-get install -q -y tzdata && rm -rf /var/lib/apt/lists/*

#RUN apt-get update && apt-get -y upgrade

# install packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*


RUN apt-get update && apt-get install -y curl

# setup keys
RUN curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | apt-key add -

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros1-latest.list

# Completely remove Python 2 support
ARG _PY_SUFFIX=3
ARG PYTHON=python${_PY_SUFFIX}
ARG PIP=pip${_PY_SUFFIX}

RUN apt-get update && apt-get install -y \
    ${PYTHON} \
    ${PYTHON}-pip

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    ${PYTHON}-rosdep \
    ${PYTHON}-rosinstall \
    ${PYTHON}-vcstools \
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
#ARG http_proxy
#ARG https_proxy

#ENV http_proxy=$http_proxy
#ENV https_proxy=$https_proxy

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
    apt-get -y install sudo nano libprotobuf-dev protobuf-compiler libboost-all-dev \
                       ros-melodic-convex-decomposition ros-melodic-ivcon \
                       ${PYTHON}-all-dev ${PYTHON}-rospkg \
                       git-core ${PYTHON}-wstool ${PYTHON}-vcstools ros-melodic-control-msgs \
                       ros-melodic-joystick-drivers ros-melodic-xacro ros-melodic-tf2-ros ros-melodic-rviz ros-melodic-cv-bridge \
                       ros-melodic-actionlib ros-melodic-actionlib-msgs ros-melodic-dynamic-reconfigure \
                       ros-melodic-trajectory-msgs ros-melodic-moveit \
                       ros-melodic-octomap-rviz-plugins \
                       ros-melodic-control-toolbox \
                       ros-melodic-rospy-message-converter \
                       ros-melodic-moveit-visual-tools \
                       ros-melodic-realtime-tools ros-melodic-ros-controllers \
                       ros-melodic-tf-conversions ros-melodic-kdl-parser\ 
                       ros-melodic-ros-control ros-melodic-ros-controllers ros-melodic-gazebo-ros-control \
                       ros-melodic-libfranka ros-melodic-franka-ros \
                       build-essential ${PYTHON}-dev swig ${PYTHON}-empy && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y ros-melodic-desktop-full --fix-missing

RUN ${PIP} --no-cache-dir install --upgrade \
    pip \
    setuptools catkin-tools

RUN ${PIP} --no-cache-dir install protobuf pandas 
# rospy-message-converter # some error in pip3 installation

#six PySide
RUN ${PIP} install numpy numpy-quaternion pygame decorator ipython jupyter matplotlib Pillow scipy scikit-learn && \
    ${PIP} install --upgrade tensorflow
# TENSOR FLOW *GPU*

# Install pybullet
RUN ${PIP} install pybullet

RUN if [[ $HTTP_PROXY != '' ]]; then git config --global http.proxy $HTTP_PROXY; fi

RUN apt-get update && apt-get -y upgrade

# Some TF tools expect a "python" binary
RUN ln -s $(which ${PYTHON}) /usr/local/bin/python

# disable sound card warnings in gazebo
COPY asound.conf /etc/asound.conf
