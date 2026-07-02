#!/bin/bash
# ============================================================
# WSL2 Ubuntu 22.04 Setup for Drone A-to-B Waypoint Nav System
# PX4 SITL + ROS 2 Humble + MAVSDK Python + Micro XRCE-DDS
# ============================================================
# Run this INSIDE your Ubuntu-22.04 WSL terminal, not PowerShell.
# Execute section by section — some steps need a new shell session.

set -e

echo "=== STAGE 0: Confirm you're on Ubuntu 22.04 ==="
lsb_release -a
# Expect: Ubuntu 22.04.x LTS (jammy)

# ============================================================
# STAGE 1: Base system update
# ============================================================
sudo apt update && sudo apt upgrade -y
sudo apt install -y git curl wget build-essential python3-pip python3-venv \
    software-properties-common gnupg lsb-release

# ============================================================
# STAGE 2: ROS 2 Humble
# ============================================================
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install -y software-properties-common
sudo add-apt-repository universe -y

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg 2>/dev/null || \
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | \
    sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
    sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-humble-desktop ros-humble-ros-base python3-argcomplete
sudo apt install -y ros-dev-tools

# Source ROS 2 automatically in every new shell
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source /opt/ros/humble/setup.bash

# ============================================================
# STAGE 3: PX4 Autopilot (SITL) + simulation toolchain
# ============================================================
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot

# This installs Gazebo (Harmonic on newer PX4 main branches), NuttX toolchain, etc.
bash ./Tools/setup/ubuntu.sh

# First build (also downloads/builds simulation targets) — takes a while
make px4_sitl

# ============================================================
# STAGE 4: Micro XRCE-DDS Agent (PX4 <-> ROS 2 bridge)
# ============================================================
cd ~
git clone -b v2.4.2 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
sudo ldconfig /usr/local/lib/

# ============================================================
# STAGE 5: PX4 ROS 2 workspace (drone_ws)
# ============================================================
mkdir -p ~/drone_ws/src
cd ~/drone_ws/src
git clone https://github.com/PX4/px4_msgs.git
git clone https://github.com/PX4/px4_ros_com.git

cd ~/drone_ws
source /opt/ros/humble/setup.bash
sudo apt install -y python3-colcon-common-extensions
colcon build
echo "source ~/drone_ws/install/setup.bash" >> ~/.bashrc

# ============================================================
# STAGE 6: MAVSDK Python
# ============================================================
python3 -m venv ~/drone_venv
source ~/drone_venv/bin/activate
pip install --upgrade pip
pip install mavsdk pymavlink

# ============================================================
# STAGE 7: Your project repo
# ============================================================
cd ~
git clone https://github.com/aadhi4200/Drone-A-to-B-waypoint-Nav-system.git
cd Drone-A-to-B-waypoint-Nav-system
# pip install -r requirements.txt   # uncomment once repo has one

# ============================================================
# STAGE 8: QGroundControl (optional, GUI via WSLg)
# ============================================================
sudo usermod -aG dialout $USER
sudo apt install -y libfuse2 libxcb-xinerama0 libxkbcommon-x11-0 \
    libxcb-cursor0 libxcb-icccm4 libxcb-image0 libxcb-keysyms1 \
    libxcb-randr0 libxcb-render-util0 libxcb-shape0
cd ~
wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl-x86_64.AppImage
chmod +x QGroundControl-x86_64.AppImage
# Launch later with: ./QGroundControl-x86_64.AppImage

echo ""
echo "============================================================"
echo "DONE. Close and reopen your WSL terminal, then verify:"
echo "  ros2 --version"
echo "  cd ~/PX4-Autopilot && make px4_sitl"
echo "  MicroXRCEAgent udp4 -p 8888"
echo "============================================================"
