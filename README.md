# autonomous_drone_ros2

Autonomous delivery drone — **ROS 2 Humble + PX4 SITL + Gazebo Harmonic + MAVROS**

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![PX4](https://img.shields.io/badge/PX4-SITL-orange)](https://px4.io/)
[![License](https://img.shields.io/badge/License-MIT-green)](LICENSE)

---

## Mission Flow

```
Takeoff from A (Home)
        ↓
Fly to B → Search ArUco → Align → Precision Land → Payload Drop (5s)
        ↓
Takeoff → Fly to C → Search ArUco → Align → Precision Land → Payload Drop (5s)
        ↓
Takeoff → Fly to D → Search ArUco → Align → Precision Land → Payload Drop (5s)
        ↓
Takeoff → Return to A → Land at base
```

---

## Workspace Structure

```
drone_ws/
└── src/
    └── autonomous_drone_ros2/
        ├── src/
        │   ├── autonomous_drone_ros2/
        │   │   ├── drone_base/           # Arm, disarm, takeoff, land
        │   │   ├── drone_camera/         # Gazebo camera bridge
        │   │   ├── drone_vision/         # ArUco ID=17 detection
        │   │   ├── drone_controller/     # WaypointNavigator + ArucoLandingNode
        │   │   ├── drone_mission/        # MissionManager (full orchestration)
        │   │   ├── drone_navigation/     # Phase 5+ Nav2 + SLAM placeholder
        │   │   ├── drone_interfaces/     # Shared constants and topic names
        │   │   ├── drone_msgs/           # Custom messages (future)
        │   │   └── drone_bringup/        # Launch files
        │   ├── drone_dashboard/          # React frontend (Phase 4)
        │   └── drone_backend/            # FastAPI backend (Phase 4)
        ├── simulation/
        │   ├── gazebo/
        │   │   ├── worlds/               # aruco_landing.sdf
        │   │   └── models/aruco_17/      # ArUco marker model + PNG texture
        │   └── px4/
        ├── docs/
        │   ├── architecture/             # Node architecture diagrams
        │   ├── setup/                    # Installation guide
        │   ├── milestones/               # Roadmap
        │   └── api/                      # API docs (Phase 4)
        ├── scripts/
        │   ├── generate_aruco.py         # Generate ArUco ID=17 PNG
        │   ├── setup_gazebo.sh           # Copy files into PX4-Autopilot
        │   └── build.sh                  # Build workspace
        └── docker/
            └── Dockerfile                # Jetson deployment (Phase 6)
```

---

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
| AI Computer | Jetson Orin Nano (Phase 6+) |

---

## Node Architecture

```
                    ┌─────────────────────┐
                    │   MissionManager    │
                    │  /mission/command   │
                    └──────┬──────────────┘
              ┌────────────┼────────────┐
              ↓            ↓            ↓
     /drone_base/cmd  /waypoint_nav/ /aruco_landing/
              ↓        command ↓      command ↓
       DroneBaseNode  WaypointNav   ArucoLanding
              ↓            ↓            ↑
           MAVROS        MAVROS    /vision/landing_target
              ↓            ↓            ↑
             PX4          PX4      VisionNode
                                        ↑
                                   CameraNode
                                        ↑
                                  Gazebo Camera
```

---

## Quick Setup

### 1. Prerequisites

```bash
# ROS 2 Humble must be installed
# PX4-Autopilot must be cloned at ~/PX4-Autopilot

sudo apt install ros-humble-mavros ros-humble-mavros-extras -y
sudo apt install ros-humble-cv-bridge python3-opencv -y
sudo apt install ros-humble-ros-gz-bridge -y
sudo apt install python3-colcon-common-extensions -y

wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash install_geographiclib_datasets.sh
```

### 2. Clone and Build

```bash
mkdir -p ~/drone_ws/src
cd ~/drone_ws/src
git clone https://github.com/aadhi4200/autonomous_drone_ros2.git

cd ~/drone_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 3. Add to .bashrc (auto-source every terminal)

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/drone_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 4. Setup Gazebo World

```bash
# Generate ArUco ID=17 PNG texture
python3 ~/drone_ws/src/autonomous_drone_ros2/scripts/generate_aruco.py

# Copy world + ArUco model into PX4-Autopilot
bash ~/drone_ws/src/autonomous_drone_ros2/scripts/setup_gazebo.sh
```

---

## Running SITL Simulation

Open **5 terminals** (recommended: Terminator with split panes).

### Terminal 1 — PX4 SITL + Gazebo

```bash
cd ~/PX4-Autopilot
PX4_GZ_WORLD=aruco_landing make px4_sitl gz_x500_lidar_cam_down
```

Wait for:
```
INFO  [px4] Startup script returned successfully
pxh> 
```

### Terminal 2 — MAVROS (ROS2 ↔ PX4 bridge)

```bash
source /opt/ros/humble/setup.bash
ros2 launch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14580"
```

Wait for:
```
INFO  [mavros]: CON: Got HEARTBEAT, connected. FCU: PX4 Autopilot
```

### Terminal 3 — Camera Bridge (Gazebo → ROS2)

```bash
source /opt/ros/humble/setup.bash
ros2 run ros_gz_bridge parameter_bridge \
  '/world/default/model/x500_lidar_cam_down_0/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image@gz.msgs.Image'
```

Wait for:
```
Creating GZ->ROS bridge for topic [/world/default/...]
```

### Terminal 4 — All ROS2 Drone Nodes

```bash
source /opt/ros/humble/setup.bash
source ~/drone_ws/install/setup.bash
ros2 launch drone_bringup full_mission.launch.py
```

Wait for all 6 nodes to report ready:
```
[camera_node]      CameraNode started.
[vision_node]      VisionNode ready — detecting ArUco ID=17
[drone_base]       DroneBaseNode started.
[waypoint_nav]     WaypointNavigator ready.
[aruco_landing]    ArucoLandingNode ready.
[mission_manager]  MissionManager ready. Publish 'START' to /mission/command
```

### Terminal 5 — Start the Mission

```bash
source ~/drone_ws/install/setup.bash
ros2 topic pub /mission/command std_msgs/msg/String "data: 'START'" --once
```

---

## Monitor the Mission

Run these in additional terminals to watch live status:

```bash
# Overall mission state
ros2 topic echo /mission/status

# Individual node states
ros2 topic echo /drone_base/status
ros2 topic echo /waypoint_nav/status
ros2 topic echo /aruco_landing/status

# ArUco marker detection (pixel offset from center)
ros2 topic echo /vision/landing_target

# Live altitude and position
ros2 topic echo /mavros/local_position/odom

# Check all active topics
ros2 topic list

# Check all running nodes
ros2 node list
```

---

## Abort Mission

```bash
ros2 topic pub /mission/command std_msgs/msg/String "data: 'ABORT'" --once
```

The drone will immediately stop navigation, abort ArUco landing, and trigger AUTO.LAND.

---

## Terminator Layout (Recommended)

```
┌──────────────────────┬──────────────────────┐
│   Terminal 1         │   Terminal 2          │
│   PX4 SITL           │   MAVROS              │
│                      │                       │
├──────────────────────┼──────────────────────┤
│   Terminal 3         │   Terminal 4          │
│   Camera Bridge      │   All ROS2 Nodes      │
│                      │   (launch file)       │
├──────────────────────┴──────────────────────┤
│   Terminal 5 — START mission                 │
│   ros2 topic pub /mission/command ...        │
└─────────────────────────────────────────────┘
```

Right-click Terminator → Split Horizontally / Vertically to create this layout.

---

## Waypoint Configuration

Edit GPS coordinates in:
`src/autonomous_drone_ros2/drone_controller/drone_controller/waypoint_navigator.py`

```python
self.waypoints = {
    "A": Waypoint(10.850500, 76.271000, label="HOME"),    # Home base
    "B": Waypoint(10.850600, 76.271200, label="POINT_B"), # Delivery 1
    "C": Waypoint(10.850700, 76.271400, label="POINT_C"), # Delivery 2
    "D": Waypoint(10.850800, 76.271600, label="POINT_D"), # Delivery 3
}
```

After editing, rebuild:
```bash
cd ~/drone_ws
colcon build --symlink-install --packages-select drone_controller
```

---

## Verification Checklist

```
□ Terminal 1 — PX4 SITL starts, Gazebo opens with aruco_landing world
□ Terminal 2 — MAVROS shows "connected. FCU: PX4 Autopilot"
□ Terminal 3 — Camera bridge shows topic created
□ Terminal 4 — All 6 nodes start without errors
□ Terminal 5 — Mission START accepted
□ Gazebo    — Drone arms, takes off to 5m
□ Gazebo    — Drone flies north toward Point B
□ Gazebo    — Drone descends over ArUco marker
□ Gazebo    — Drone lands precisely on marker
□ Gazebo    — Drone takes off, flies to C, D
□ Gazebo    — Drone returns to home and lands
```

Full SITL test

# Terminal 1 — PX4 SITL
cd ~/PX4-Autopilot
PX4_GZ_WORLD=aruco_landing make px4_sitl gz_x500_lidar_cam_down

# Terminal 2 — MAVROS
source /opt/ros/humble/setup.bash
ros2 launch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14580"

# Terminal 3 — Camera Bridge
source /opt/ros/humble/setup.bash
ros2 run ros_gz_bridge parameter_bridge \
'/world/default/model/x500_lidar_cam_down_0/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image@gz.msgs.Image'

# Terminal 4 — All nodes
source ~/drone_ws/install/setup.bash
ros2 launch drone_bringup full_mission.launch.py

# Terminal 5 — Start mission
source ~/drone_ws/install/setup.bash
ros2 topic pub /mission/command std_msgs/msg/String "data: 'START'" --once

---

## Common Errors and Fixes

| Error | Fix |
|---|---|
| `No such file: aruco_landing.sdf` | Run `setup_gazebo.sh` |
| `ArUco PNG not found` | Run `generate_aruco.py` |
| `MAVROS not connected` | Check PX4 SITL is running first |
| `EKF2 heading failure` | Use `gz_x500_lidar_cam_down` model (has magnetometer) |
| `OFFBOARD rejected` | Setpoints must stream 2s before mode switch |
| `Camera topic not found` | Run camera bridge (Terminal 3) |
| `colcon build error` | Run `source /opt/ros/humble/setup.bash` first |
| `QoS mismatch warning` | All subscribers use BEST_EFFORT QoS — already set |

---

## Roadmap

| Phase | Status | Description |
|---|---|---|
| Phase 1 | ✅ Done | ROS2 + Camera + ArUco Vision |
| Phase 2 | ✅ Done | PX4 SITL + MAVROS + Offboard control |
| Phase 3 | ✅ Done | Full mission A→B→C→D→Home in SITL |
| Phase 4 | 🔄 Next | Real hardware — Pixhawk + Raspberry Pi 5 |
| Phase 5 | 📋 Planned | OAK-D Lite + Nav2 + obstacle avoidance |
| Phase 6 | 📋 Planned | Jetson Orin Nano + TensorRT YOLOv8 |

---

## License

MIT License © 2024 [aadhi4200](https://github.com/aadhi4200)
