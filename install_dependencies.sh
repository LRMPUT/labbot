#!/usr/bin/env bash
LABBOT_WS_PATH="$(pwd)"

# Dependencies installation
sudo apt install libgflags-dev ros-$ROS_DISTRO-image-geometry ros-$ROS_DISTRO-camera-info-manager ros-$ROS_DISTRO-image-transport ros-$ROS_DISTRO-image-publisher libgoogle-glog-dev libusb-1.0-0-dev libeigen3-dev

# Install libuvc library (required for camera)
cd libuvc
mkdir build && cd build
cmake .. && make -j4
sudo make install
sudo ldconfig

# # Install libusb rules
sudo bash "$LABBOT_WS_PATH"/src/ros2_astra_camera/astra_camera/scripts/install.sh
sudo udevadm control --reload-rules && sudo udevadm trigger