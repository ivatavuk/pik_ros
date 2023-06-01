#!/bin/bash
set -e

#Build and install osqp
cd
git clone --recursive --branch release-0.6.3 https://github.com/osqp/osqp
cd osqp
mkdir build
cd build
cmake -G "Unix Makefiles" ..
cmake --build .
sudo cmake --build . --target install

#Install Eigen3
sudo apt install libeigen3-dev

#Build and install osqp-eigen
cd
git clone https://github.com/robotology/osqp-eigen.git
cd osqp-eigen
mkdir build
cd build
cmake ..
make
sudo make install

#Build and install ptsc_eigen
cd 
git clone https://github.com/ivatavuk/ptsc_eigen.git
cd ptsc_eigen
mkdir build
cd build
cmake ..
make
sudo make install

#Install moveit
sudo apt-get update
sudo apt-get install -y ros-${ROS_DISTRO}-moveit