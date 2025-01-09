#!/bin/bash

set -e

# install required pkgs
sudo apt update
sudo apt install -y ros-noetic-octomap ros-noetic-octomap-server ros-noetic-pcl-conversions

# build ros ws
cd ./catkin_ws
catkin build
cd ../

# Check if Simulation.x86_64 already exists
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
