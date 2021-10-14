FROM osrf/ros:melodic-desktop-full
ENV DEBIAN_FRONTEND noninteractive

#ARG http_proxy
#ARG https_proxy

#ENV http_proxy=$http_proxy
#ENV https_proxy=$https_proxy

SHELL ["/bin/bash", "-c"]

# setup keys
RUN curl -sSL \
'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' \
| apt-key add -

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
                       ros-melodic-control-toolbox ros-melodic-catkin python-catkin-tools \
                       ros-melodic-moveit-visual-tools \
                       ros-melodic-rospy-message-converter \
                       ros-melodic-realtime-tools ros-melodic-ros-controllers \
                       ros-melodic-tf-conversions ros-melodic-kdl-parser \ 
                       ros-melodic-ros-control ros-melodic-ros-controllers ros-melodic-gazebo-ros-control \
                       ros-melodic-libfranka ros-melodic-franka-ros \
                       build-essential python-dev swig python-pygame && \
    rm -rf /var/lib/apt/lists/*
 #ros-melodic-sns-ik-lib \

RUN pip install --upgrade pip && \
    pip install protobuf && \
    pip install pandas 


#six PySide
RUN pip install numpy numpy-quaternion==2020.5.19.15.27.24 pygame decorator ipython jupyter matplotlib Pillow scipy scikit-learn && \
    pip install --upgrade tensorflow
# TENSOR FLOW *GPU*

# Install pybullet
RUN pip install pybullet



# Install pybullet
# RUN pip install pybullet

RUN if [[ $HTTP_PROXY != '' ]]; then git config --global http.proxy $HTTP_PROXY; fi

RUN apt-get update && apt-get -y install python-catkin-tools
RUN apt-get update && apt-get -y upgrade

# disable sound card warnings in gazebo
COPY asound.conf /etc/asound.conf

# setup entrypoint, need entrypoint.sh in the same folder with Dockerfile
COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["bash"]