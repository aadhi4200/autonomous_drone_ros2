# Node Architecture

## Topic Map
```
MissionManager
  ├── publishes → /drone_base/command
  ├── publishes → /waypoint_nav/command
  ├── publishes → /aruco_landing/command
  ├── subscribes ← /drone_base/status
  ├── subscribes ← /waypoint_nav/status
  └── subscribes ← /aruco_landing/status

CameraNode → /camera/image_raw → VisionNode → /vision/landing_target → ArucoLandingNode
DroneBaseNode → /mavros/setpoint_position/local → MAVROS → PX4
ArucoLandingNode → /mavros/setpoint_velocity/cmd_vel_unstamped → MAVROS → PX4
```

## State Machine
```
IDLE → PREFLIGHT → TAKEOFF → GOTO_WAYPOINT → ARUCO_LAND
                                                   ↓
                                          WAIT_ON_GROUND
                                                   ↓
                                          INTER_TAKEOFF
                                         ↙            ↘
                               GOTO_WAYPOINT      RETURN_HOME
                                                       ↓
                                                  HOME_LAND
                                                       ↓
                                             MISSION_COMPLETE
```
