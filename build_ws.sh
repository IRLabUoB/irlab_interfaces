#! /bin/bash

source /opt/ros/$ROS_DISTRO/setup.bash
cd ..
wstool init
wstool merge irlab_interfaces/"$ROS_DISTRO"_dependencies.rosinstall
wstool up

cd .. && rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO --skip-keys python-sip

source /opt/ros/$ROS_DISTRO/setup.bash
catkin build

cp src/irlab_interfaces/package_scripts/franka.sh ./
cp src/irlab_interfaces/package_scripts/intera.sh ./
