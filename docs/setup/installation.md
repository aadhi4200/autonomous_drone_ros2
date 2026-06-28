# Installation Guide

## 1. Create workspace
```bash
mkdir -p ~/drone_ws/src
cd ~/drone_ws/src
git clone https://github.com/aadhi4200/autonomous_drone_ros2.git
```

## 2. Install dependencies
```bash
sudo apt install ros-humble-mavros ros-humble-mavros-extras -y
sudo apt install ros-humble-cv-bridge python3-opencv -y
sudo apt install ros-humble-ros-gz-bridge -y
sudo apt install python3-colcon-common-extensions -y
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash install_geographiclib_datasets.sh
```

## 3. Build
```bash
cd ~/drone_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 4. Setup Gazebo
```bash
python3 ~/drone_ws/src/autonomous_drone_ros2/scripts/generate_aruco.py
bash ~/drone_ws/src/autonomous_drone_ros2/scripts/setup_gazebo.sh
```

## 5. Add to .bashrc
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/drone_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
