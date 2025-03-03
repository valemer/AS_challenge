#!/bin/bash

set -e

# install ros noetic
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop-full
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential python3-catkin-tools

source /opt/ros/noetic/setup.bash

# install required pkgs
sudo apt install -y ros-noetic-depth-image-proc ros-noetic-octomap ros-noetic-octomap-server \
  ros-noetic-pcl-conversions git-lfs unzip python3-pip
pip install pynput
pip install scipy

# build ros ws
cd ./catkin_ws
catkin build
cd ../

sudo chmod +x ./Simulation.zip 

# Check if Simulation.x86_64 already exists
git lfs pull
if [ ! -f ./catkin_ws/devel/lib/simulation/Simulation.x86_64 ]; then
  echo "Simulation.x86_64 not found. Unzipping simulation files..."

  # Unzip simulation files
  mkdir -p ./tmp/extracted
  cp ./Simulation.zip ./tmp/
  cd tmp/
  unzip Simulation.zip -d extracted
  cd ../

  # Place simulation files in devel folder
  mv -f ./tmp/extracted/* ./catkin_ws/devel/lib/simulation/
  chmod +x ./catkin_ws/devel/lib/simulation/Simulation.x86_64
  rm -r ./tmp
else
  echo "Simulation.x86_64 already exists. Skipping unzip."
fi
