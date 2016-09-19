#!/bin/bash

set -e

# http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Kinetic%20on%20the%20Raspberry%20Pi 

echo "=== setup: setup ROS repositories"
#sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
#sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116

echo "=== setup: install bootstrap dependencies"
#sudo apt-get update
#sudo apt-get -y upgrade
sudo apt-get install -y python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential cmake

echo "=== setup: initializing rosdep"
sudo rosdep init
rosdep update

echo "=== setup: create a catkin workspace"
mkdir -p ~/ros_catkin_ws
pushd ~/ros_catkin_ws
rosinstall_generator ros_comm common_msgs tf --rosdistro kinetic --deps --wet-only --tar > kinetic-ros_comm-wet.rosinstall
wstool init src kinetic-ros_comm-wet.rosinstall

echo "=== setup: resolving dependencies"
rosdep install -y --from-paths src --ignore-src --rosdistro kinetic -r --os=debian:jessie

echo "=== setup: building the catkin workspace"
sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/kinetic -j2

popd


echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
