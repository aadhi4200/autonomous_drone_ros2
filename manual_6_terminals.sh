#!/usr/bin/env bash
#
# manual_6_terminals.sh
# Reference commands for running the SITL stack as 6 separate terminals
# instead of launch_full_sim.sh. Not meant to be executed as one script --
# copy each numbered block into its own terminal, in order, waiting for
# each stage to finish starting before moving to the next.
#
# PX4_HOME_LAT/LON below are read from ~/drone_ws2/.last_synced_home at the
# time this file was generated. Re-sync location from the website and
# update these two values (or `cat ~/drone_ws2/.last_synced_home`) before
# Terminal 1 if you've synced since. Syncing AFTER Terminal 1 is already
# running has no effect until PX4 is restarted with the new values.

# ── Terminal 1 — PX4 SITL ──────────────────────────────────────────────────
cd ~/PX4-Autopilot
export PX4_HOME_LAT=8.5359441
export PX4_HOME_LON=76.9297242
export PX4_HOME_ALT=0
PX4_GZ_WORLD=aruco_landing make px4_sitl gz_x500_lidar_cam_down
# Wait for "Startup script returned successfully" before continuing.
# NOTE: if this is backgrounded/non-interactive (no real tty) rather than
# run in a live terminal, PX4's pxh> shell can busy-loop on stdin EOF and
# fill its log to tens of MB/sec. `< /dev/null` did not stop this when it
# was tested backgrounded here; holding stdin open on a FIFO did:
#   mkfifo /tmp/px4_stdin.fifo; exec 3<> /tmp/px4_stdin.fifo
#   setsid make px4_sitl gz_x500_lidar_cam_down <&3 > px4.log 2>&1 &
# Not needed if you're just typing this directly into a real terminal.

# ── Terminal 2 — MAVROS ────────────────────────────────────────────────────
source /opt/ros/humble/setup.bash
ros2 launch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14580"
# Wait for "Got HEARTBEAT ... connected" before continuing.

# ── Terminal 3 — Camera Bridge ─────────────────────────────────────────────
source /opt/ros/humble/setup.bash
ros2 run ros_gz_bridge parameter_bridge \
'/world/default/model/x500_lidar_cam_down_0/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image@gz.msgs.Image'

# ── Terminal 4 — All mission nodes (drone_ws2, NOT drone_ws) ───────────────
source /opt/ros/humble/setup.bash
source ~/drone_ws2/install/setup.bash
ros2 launch drone_bringup full_mission.launch.py
# Wait for /mission/status topic / "MissionManager ready" before continuing.

# ── Terminal 5 — Website backend (FastAPI + rclpy bridge) ─────────────────
source /opt/ros/humble/setup.bash
source ~/drone_ws2/install/setup.bash
cd ~/drone_ws2/src/frontend_bridge/backend
python3 main.py
# Wait for "BridgeNode ready" — serves on http://localhost:8000

# ── Terminal 6 — Website frontend ──────────────────────────────────────────
cd ~/drone_ws2/src/frontend_bridge
npm run dev
# Vite defaults to port 3000, but auto-bumps to 3001+ if 3000 is taken --
# check this terminal's own "Local:" line for the actual URL instead of
# assuming a fixed port.
