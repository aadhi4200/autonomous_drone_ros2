# autonomous_drone_ros2

Autonomous delivery drone — **ROS 2 Humble + PX4 SITL + Gazebo Harmonic + MAVROS**

## Mission
```
Takeoff from A
    ↓
B → ArUco Precision Land → Payload Drop
    ↓
C → ArUco Precision Land → Payload Drop
    ↓
D → ArUco Precision Land → Payload Drop
    ↓
Return to A → Land at base
```

## Workspace Structure
```
drone_ws/
└── src/
    └── autonomous_drone_ros2/
        ├── drone_base/           # Arm, disarm, takeoff, land
        ├── drone_camera/         # Gazebo camera bridge
        ├── drone_vision/         # ArUco ID=17 detection
        ├── drone_controller/     # WaypointNavigator + ArucoLandingNode
        ├── drone_mission/        # MissionManager (full orchestration)
        ├── drone_navigation/     # Phase 5+ Nav2 + SLAM placeholder
        ├── drone_interfaces/     # Shared constants and utilities
        ├── drone_msgs/           # Custom messages (future)
        ├── drone_bringup/        # Launch files
        ├── simulation/           # Gazebo worlds + ArUco models
        ├── docs/                 # Architecture, setup, roadmap
        ├── scripts/              # Setup and build scripts
        └── docker/               # Container for Jetson deployment
```

## Quick Start
```bash
# 1. Clone
cd ~/drone_ws/src
git clone https://github.com/aadhi4200/autonomous_drone_ros2.git

# 2. Build
cd ~/drone_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# 3. Setup Gazebo
bash src/autonomous_drone_ros2/scripts/setup_gazebo.sh

# 4. Launch (5 terminals)
# T1: cd ~/PX4-Autopilot && PX4_GZ_WORLD=aruco_landing make px4_sitl gz_x500_lidar_cam_down
# T2: ros2 launch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14580"
# T3: ros2 run ros_gz_bridge parameter_bridge '/world/default/model/x500_lidar_cam_down_0/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image@gz.msgs.Image'
# T4: ros2 launch drone_bringup full_mission.launch.py
# T5: ros2 topic pub /mission/command std_msgs/msg/String "data: 'START'" --once
```

## Hardware (Planned)
| Component | Part |
|---|---|
| Frame | TBS 500 / Holybro S500 V2 |
| Flight Controller | Pixhawk 2.4.8 |
| Companion Computer | Raspberry Pi 5 |
| Camera | IMX219 (downward facing) |
| LiDAR | TF02-Pro |
| GPS | M8N |
| Depth Camera | OAK-D Lite (Phase 5+) |

## Roadmap
- [x] Phase 1 — ROS2 + Camera + Vision
- [x] Phase 2 — PX4 SITL + MAVROS + Offboard
- [x] Phase 3 — ArUco precision landing (simulation)
- [ ] Phase 4 — Real hardware (Pixhawk + RPi 5)
- [ ] Phase 5 — OAK-D Lite obstacle avoidance + Nav2
- [ ] Phase 6 — Jetson Orin Nano + TensorRT YOLOv8
