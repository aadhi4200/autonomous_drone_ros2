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
#
# Corrected 2026-07-10: `make px4_sitl gz_<model>` (Terminal 1 below, old
# version) does NOT work on this PX4 checkout (v1.18.0-alpha1+) — the
# per-model ninja target it needs was removed upstream. This was verified
# broken repeatedly, not a fluke. Terminal 1 is now the two-process
# replacement, verified working end-to-end (including markers) against the
# real aruco_landing world. Terminal 3's camera bridge topic was ALSO wrong
# (camera lives in a nested "mono_cam" sub-model, not directly on
# camera_link) — but even with a corrected path, the camera sensor doesn't
# publish at all in this environment (likely a WSL GPU/offscreen-rendering
# limitation) — deprioritized, not fixed. Terminals 2/4/5/6 are unchanged
# and confirmed working, including all running together simultaneously.

# ── Terminal 1 — PX4 SITL + Gazebo, as two processes ───────────────────────
# Build once (safe to rerun elsewhere, no-op if already built):
#   cd ~/PX4-Autopilot && make px4_sitl_default
#
# Terminal 1a — Gazebo:
export GZ_SIM_RESOURCE_PATH=~/PX4-gazebo-models/models:~/drone_ws2/src/autonomous_drone_ros2/simulation/gazebo/models:~/PX4-Autopilot/Tools/simulation/gz/models
export GZ_SIM_SERVER_CONFIG_PATH=~/PX4-gazebo-models/server.config
gz sim -r ~/drone_ws2/src/autonomous_drone_ros2/simulation/gazebo/worlds/aruco_landing.sdf
#
# Terminal 1b — PX4, in a SECOND terminal, once Gazebo's window is open:
cd ~/PX4-Autopilot/build/px4_sitl_default/rootfs
export PX4_HOME_LAT=8.5359441
export PX4_HOME_LON=76.9297242
export PX4_HOME_ALT=0
export PX4_SYS_AUTOSTART=4018   # airframe ID for x500_lidar_cam_down
export PX4_GZ_WORLD=aruco_landing
~/PX4-Autopilot/build/px4_sitl_default/bin/px4 -d
# Wait for "Startup script returned successfully" before continuing.
# `-d` (daemon mode) is the actual fix for the pxh> shell's stdin-EOF
# busy-loop — confirmed live: `< /dev/null` alone did NOT stop it when
# tested against the raw binary (filled a log to 800+MB in under a minute).
#
# Or just run both of the above as one command:
#   ros2 launch drone_bringup px4_sitl.launch.py gz_world:=aruco_landing

# ── Terminal 2 — MAVROS ────────────────────────────────────────────────────
source /opt/ros/humble/setup.bash
ros2 launch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14580"
# Wait for "Got HEARTBEAT ... connected" before continuing.

# ── Terminal 3 — Camera Bridge (DEPRIORITIZED — camera doesn't publish in
# this environment at all right now, likely a WSL GPU/rendering limitation,
# not just a topic-name issue; skip this terminal for now) ─────────────────
source /opt/ros/humble/setup.bash
ros2 run ros_gz_bridge parameter_bridge \
'/world/aruco_landing/model/x500_0/model/mono_cam/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image@gz.msgs.Image'

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
