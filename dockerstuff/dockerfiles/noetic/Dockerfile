# FROM osrf/ros:kinetic-desktop-full-xenial
FROM osrf/ros:noetic-desktop-full

ENV ROS_DISTRO=${ROS_DISTRO}

RUN apt-get update && apt-get install -q -y \
    build-essential git swig sudo python3-future libcppunit-dev python3-pip

#installs avahi for local network discovery (required for using the sawyer robot)
RUN apt-get update && apt-get install -y \
    avahi-daemon avahi-utils


RUN apt-get update && \ 
    apt-get -y install sudo nano python3-pip python3-scipy libprotobuf-dev protobuf-compiler libboost-all-dev \
                       ros-noetic-convex-decomposition ros-noetic-ivcon \
                       git-core python3-argparse python3-wstool python3-vcstools python3-rosdep ros-noetic-control-msgs \
                       ros-noetic-joystick-drivers ros-noetic-xacro ros-noetic-tf2-ros ros-noetic-rviz ros-noetic-cv-bridge \
                       ros-noetic-actionlib ros-noetic-actionlib-msgs ros-noetic-dynamic-reconfigure \
                       ros-noetic-trajectory-msgs ros-noetic-moveit \
                       ros-noetic-octomap-rviz-plugins \
                       ros-noetic-control-toolbox  ros-noetic-catkin python3-catkin-tools \
                       ros-noetic-rospy-message-converter\
                       ros-noetic-moveit-visual-tools \
                       ros-noetic-realtime-tools ros-noetic-ros-controllers \
                       ros-noetic-tf-conversions ros-noetic-kdl-parser\ 
                       ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-gazebo-ros-control \
                       ros-noetic-libfranka ros-noetic-franka-ros \
                       build-essential python3-dev swig python3-pygame && \
    rm -rf /var/lib/apt/lists/*
 #ros-noetic-sns-ik-lib \

RUN pip3 install --upgrade pip && \
    pip3 install protobuf && \
    pip3 install pandas

#six PySide
RUN pip3 install numpy numpy-quaternion==2020.5.19.15.27.24 pygame decorator ipython jupyter matplotlib Pillow scipy scikit-learn 
    # https://storage.googleapis.com/tensorflow/linux/gpu/tensorflow_gpu-1.4.1-cp27-none-linux_x86_64.whl
# TENSOR FLOW *GPU*

# Install pybullet
RUN pip3 install pybullet


# ===========================================
# ========= FOR CUDA AND TENSORFLOW =========
# ===========================================


# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

RUN if [[ $HTTP_PROXY != '' ]]; then git config --global http.proxy $HTTP_PROXY; fi


RUN apt-get update && apt-get -y install python3-catkin-tools ros-noetic-ros-pytest \ 
    ros-noetic-joy-teleop libcppunit-dev ros-noetic-moveit-resources python3-future \
    ros-noetic-rosparam-shortcuts 
    
RUN apt-get update && apt-get -y upgrade

# disable sound card warnings in gazebo
COPY asound.conf /etc/asound.conf

# setup entrypoint, need entrypoint.sh in the same folder with Dockerfile
COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["bash"]