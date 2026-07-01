# Installation Guide

## 1. Create workspace
\```bash
mkdir -p ~/drone_ws/src
cd ~/drone_ws/src
git clone https://github.com/aadhi4200/autonomous_drone_ros2.git
\```

## 2. Install dependencies
\```bash
sudo apt install ros-humble-mavros ros-humble-mavros-extras -y
sudo apt install ros-humble-cv-bridge python3-opencv -y
sudo apt install ros-humble-ros-gz-bridge -y
sudo apt install python3-colcon-common-extensions -y
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash install_geographiclib_datasets.sh
\```

## 3. Verify package location
\```bash
find ~/drone_ws/src -name "package.xml" | sort
\```
Confirm all 9 packages appear under:
`~/drone_ws/src/autonomous_drone_ros2/src/autonomous_drone_ros2/<package>/package.xml`

## 4. Build (no --symlink-install, with explicit base-paths)
\```bash
cd ~/drone_ws
source /opt/ros/humble/setup.bash
colcon build --base-paths src/autonomous_drone_ros2/src/autonomous_drone_ros2
source install/setup.bash
\```

## 5. Verify executables installed correctly
\```bash
ros2 pkg list | grep drone
ls install/drone_camera/lib/drone_camera/
\```
If `camera_node` is missing from `lib/drone_camera/` (check with `find install/drone_camera -type f`),
run the symlink fix:
\```bash
bash ~/drone_ws/src/autonomous_drone_ros2/scripts/fix_symlinks.sh
\```

## 6. Setup Gazebo
\```bash
python3 ~/drone_ws/src/autonomous_drone_ros2/src/autonomous_drone_ros2/../../scripts/generate_aruco.py
bash ~/drone_ws/src/autonomous_drone_ros2/src/autonomous_drone_ros2/../../scripts/setup_gazebo.sh
\```

## 7. Add to .bashrc (use the build_drone alias too)
\```bash
cat >> ~/.bashrc << 'EOF'
source /opt/ros/humble/setup.bash
source ~/drone_ws/install/setup.bash
alias build_drone='cd ~/drone_ws && rm -rf build install log && source /opt/ros/humble/setup.bash && colcon build --base-paths src/autonomous_drone_ros2/src/autonomous_drone_ros2 && bash ~/drone_ws/src/autonomous_drone_ros2/scripts/fix_symlinks.sh && source install/setup.bash'
EOF
source ~/.bashrc
\```