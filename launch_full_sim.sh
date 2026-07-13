#!/usr/bin/env bash
#
# launch_full_sim.sh
# Sequenced launch of the full SITL stack: PX4 SITL -> MAVROS -> camera bridge
# -> all mission nodes -> backend bridge (so the website can drive it).
#
# This does NOT just fire everything in parallel. Each stage waits for a real
# readiness signal from the previous one before starting, because e.g. MAVROS
# started before PX4 is actually listening will silently fail to connect --
# no error, it just never links, matching the QoS silent-failure pattern
# already documented for this project.
#
# The old manual "Terminal 5 — publish /mission/command START by hand" step is
# gone by default: once the backend bridge (backend/main.py) is up, the
# website's own START button publishes that exact same message. Use
# --cli-start only if you're testing the ROS2 side without the website open.
#
# Usage:
#   chmod +x launch_full_sim.sh
#   ./launch_full_sim.sh              # Gazebo GUI on, sim + website backend up
#   ./launch_full_sim.sh --headless   # no GUI window, lighter/faster, use if
#                                      # MAVROS timesync issues reappear under load
#   ./launch_full_sim.sh --no-backend # sim only, no website bridge
#   ./launch_full_sim.sh --cli-start  # also fire START via CLI (no website needed)
#
# Ctrl+C at any point kills everything this script started (PX4/Gazebo, MAVROS,
# bridge, node launch, backend) via the trap below.

set -uo pipefail

# ── Paths — edit these if your setup differs ─────────────────────────────
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
DRONE_WS="${DRONE_WS:-$HOME/drone_ws2}"
BACKEND_DIR="${BACKEND_DIR:-$HOME/drone_ws2/src/frontend_bridge/backend}"
LOG_DIR="${LOG_DIR:-$HOME/drone_ws2/logs/$(date +%Y%m%d_%H%M%S)}"
# Airframe ID for this project's vehicle (ROMFS/px4fmu_common/init.d-posix/
# airframes/4018_gz_x500_lidar_cam_down) — replaces the old GZ_MODEL/
# `make px4_sitl gz_<model>` target name, which no longer exists on PX4
# v1.18.0-alpha1+ (verified 2026-07-10: that command fails immediately with
# `ninja: error: unknown target`, on this exact checkout, freshly cloned,
# never pulled since — this was never actually working here, not a
# regression from something that changed after this script was written).
PX4_SYS_AUTOSTART="${PX4_SYS_AUTOSTART:-4018}"
# Base Gazebo model/world assets (ground plane, sun, x500 vehicle family).
# Defaults to a pre-populated PX4-gazebo-models clone if present, else the
# stock simulation-gazebo script's own auto-download location.
GZ_MODEL_STORE="${GZ_MODEL_STORE:-$([ -d "$HOME/PX4-gazebo-models" ] && echo "$HOME/PX4-gazebo-models" || echo "$HOME/.simulation-gazebo")}"
# This project's own world/models (aruco_landing.sdf, aruco_<id> markers
# Feature 1 spawns at runtime) — merged onto GZ_SIM_RESOURCE_PATH alongside
# GZ_MODEL_STORE below, not copied into it.
PROJECT_SIM_DIR="$DRONE_WS/src/autonomous_drone_ros2/simulation/gazebo"
GZ_WORLD="${GZ_WORLD:-aruco_landing}"
# NOTE (found 2026-07-10, not yet fixed — separate from the boot-mechanism
# fix above): the actual spawned vehicle model name under PX4_SYS_AUTOSTART
# is "x500_0", not "x500_lidar_cam_down_0" — confirmed via a live boot's own
# gz_bridge log ("world: aruco_landing, model: x500_0"). CAMERA_TOPIC below
# is very likely wrong as a result and the camera bridge in stage 3 may not
# actually find its topic. Verify before relying on the camera feed.
CAMERA_MODEL_NAME="${CAMERA_MODEL_NAME:-x500_lidar_cam_down_0}"
FCU_URL="${FCU_URL:-udp://:14540@127.0.0.1:14580}"
# Written by the website (POST /system/set-home) whenever the operator's
# browser syncs its laptop geolocation. Spawning SITL's home anywhere else
# guarantees the backend's home_position_match gate fails permanently, since
# it compares PX4's actual home against this exact file.
SYNCED_HOME_FILE="${SYNCED_HOME_FILE:-$HOME/drone_ws2/.last_synced_home}"
PX4_HOME_ALT_DEFAULT="${PX4_HOME_ALT_DEFAULT:-0}"
# If the GUI shows a black/broken viewport (known issue on weaker/older GPUs
# like the MX250), export this before running: PX4_GZ_SIM_RENDER_ENGINE=ogre
PX4_GZ_SIM_RENDER_ENGINE="${PX4_GZ_SIM_RENDER_ENGINE:-}"

HEADLESS=0     # GUI on by default — you need to see the drone move.
               # Use --headless when you actually hit MAVROS timesync/RTT
               # issues under load (per project history) or want a faster,
               # lighter-weight run without watching it.
WITH_BACKEND=1 # launch backend/main.py (FastAPI+rclpy bridge) so the website can drive it
CLI_START=0    # default OFF: press START on the website instead of publishing via CLI

for arg in "$@"; do
  case "$arg" in
    --headless)     HEADLESS=1 ;;
    --no-backend)   WITH_BACKEND=0 ;;
    --cli-start)    CLI_START=1 ;;   # for testing without the website up
    *) echo "Unknown option: $arg" ; exit 1 ;;
  esac
done

mkdir -p "$LOG_DIR"
PIDS=()

# ── Cleanup on Ctrl+C / exit ──────────────────────────────────────────────
cleanup() {
  echo ""
  echo "🛑 Shutting down — killing $(( ${#PIDS[@]} )) launched process(es)..."
  for pid in "${PIDS[@]:-}"; do
    if [ -n "${pid:-}" ] && kill -0 "$pid" 2>/dev/null; then
      kill "$pid" 2>/dev/null
    fi
  done
  sleep 1
  echo "Done. If Gazebo/PX4 processes linger, check: pgrep -fl 'px4|gz sim'"
}
trap cleanup INT TERM EXIT

wait_for_log() {
  # wait_for_log <logfile> <pattern> <timeout_seconds> <stage_name>
  local logfile="$1" pattern="$2" timeout="$3" name="$4"
  local waited=0
  echo "⏳ Waiting for $name (up to ${timeout}s)..."
  while ! grep -qE "$pattern" "$logfile" 2>/dev/null; do
    sleep 1
    waited=$((waited + 1))
    if [ "$waited" -ge "$timeout" ]; then
      echo "❌ Timed out waiting for $name. Last 20 lines of $logfile:"
      tail -20 "$logfile" 2>/dev/null
      exit 1
    fi
  done
  echo "✅ $name ready (${waited}s)"
}

wait_for_topic() {
  # wait_for_topic <topic_substring> <timeout_seconds> <stage_name>
  local pattern="$1" timeout="$2" name="$3"
  local waited=0
  echo "⏳ Waiting for $name topic (up to ${timeout}s)..."
  while ! ros2 topic list 2>/dev/null | grep -q "$pattern"; do
    sleep 1
    waited=$((waited + 1))
    if [ "$waited" -ge "$timeout" ]; then
      echo "❌ Timed out waiting for $name topic."
      exit 1
    fi
  done
  echo "✅ $name topic present (${waited}s)"
}

wait_for_topic_soft() {
  # Same as wait_for_topic, but returns 1 on timeout instead of exiting --
  # for stages whose failure shouldn't be fatal to the rest of the stack.
  local pattern="$1" timeout="$2" name="$3"
  local waited=0
  echo "⏳ Waiting for $name topic (up to ${timeout}s)..."
  while ! ros2 topic list 2>/dev/null | grep -q "$pattern"; do
    sleep 1
    waited=$((waited + 1))
    if [ "$waited" -ge "$timeout" ]; then
      return 1
    fi
  done
  echo "✅ $name topic present (${waited}s)"
  return 0
}

# ── Stage 1 — PX4 SITL + Gazebo ───────────────────────────────────────────
echo "── Stage 1/5: PX4 SITL + Gazebo ──"
if [ -f "$SYNCED_HOME_FILE" ]; then
  SYNCED_HOME_RAW="$(cat "$SYNCED_HOME_FILE")"
  PX4_HOME_LAT="${SYNCED_HOME_RAW%%,*}"
  PX4_HOME_LON="${SYNCED_HOME_RAW##*,}"
  PX4_HOME_ALT="$PX4_HOME_ALT_DEFAULT"
  echo "ℹ️  Spawning SITL home at synced location $PX4_HOME_LAT,$PX4_HOME_LON (from $SYNCED_HOME_FILE)"
else
  echo "⚠️  No synced home file at $SYNCED_HOME_FILE yet — SITL will spawn at the"
  echo "   world's default origin, and the website's home_position_match gate"
  echo "   will stay red until you sync location from the website and rerun."
fi
# Build once (safe to rerun — ninja no-ops if already built).
echo "ℹ️  Ensuring px4_sitl_default is built (first run only, ~10-20min)..."
(cd "$PX4_DIR" && make px4_sitl_default) > "$LOG_DIR/01a_px4_build.log" 2>&1
if [ $? -ne 0 ]; then
  echo "❌ px4_sitl_default build failed. See $LOG_DIR/01a_px4_build.log"
  exit 1
fi

# Gazebo, as its own process — PX4 waits for it internally ("Waiting for
# Gazebo world..." / "Gazebo world is ready" in its own log), so no separate
# readiness wait is needed here; the PX4 wait_for_log below covers it.
#
# The simulation SERVER always runs headless (-s): a combined server+GUI
# process corrupts IMU/GPS timestamps under CPU load (confirmed live
# 2026-07-10 — caused auto preflight disarms and garbage telemetry flung
# across the dashboard map). When the GUI is wanted, it runs as a separate
# viewer client (gz sim -g) attached to the headless server, which is the
# verified-stable configuration.
(
  # PX4-Autopilot ships the x500_lidar_cam_down model (and its mono_cam /
  # LW20 parts) in its own tree, NOT in PX4-gazebo-models -- without this
  # entry gz cannot resolve the model and PX4 silently spawns the plain
  # x500 with NO camera at all (found 2026-07-13: world had x500_0 with
  # zero image topics).
  export GZ_SIM_RESOURCE_PATH="$HOME/PX4-Autopilot/Tools/simulation/gz/models:$GZ_MODEL_STORE/models:$PROJECT_SIM_DIR/models"
  export GZ_SIM_SERVER_CONFIG_PATH="$GZ_MODEL_STORE/server.config"
  [ -n "$PX4_GZ_SIM_RENDER_ENGINE" ] && export PX4_GZ_SIM_RENDER_ENGINE
  # --headless-rendering: camera/lidar sensors need a render engine even
  # on the display-less server (EGL offscreen); without it the camera
  # sensor never creates its image topic.
  gz sim -r -s --headless-rendering "$PROJECT_SIM_DIR/worlds/$GZ_WORLD.sdf"
) > "$LOG_DIR/01b_gazebo.log" 2>&1 &
PIDS+=("$!")

if [ "$HEADLESS" -eq 0 ]; then
  (
    [ -n "$PX4_GZ_SIM_RENDER_ENGINE" ] && export PX4_GZ_SIM_RENDER_ENGINE
    sleep 3   # let the server open its transport endpoints first
    gz sim -g
  ) > "$LOG_DIR/01b_gazebo_gui.log" 2>&1 &
  PIDS+=("$!")
fi

# PX4 SITL, connecting to the Gazebo process above. `-d` (daemon mode,
# "don't start pxh shell") is the actual fix for the pxh> shell's stdin-EOF
# busy-loop — confirmed live: `< /dev/null` alone did NOT stop it when
# tested against the raw binary (filled a log to 800+MB in under a minute).
# Both are kept together for safety.
(
  cd "$PX4_DIR/build/px4_sitl_default/rootfs" || exit 1
  export PX4_SYS_AUTOSTART
  # Where PX4 resolves the model SDF it spawns (px4-rc.gzsim builds a
  # file://.../<model>/model.sdf URI from this). Never set anywhere before,
  # so the URI was file:///x500/... and the camera model could not load.
  export PX4_GZ_MODELS="/home/aadhi/PX4-Autopilot/Tools/simulation/gz/models"
  export PX4_GZ_WORLD="$GZ_WORLD"
  if [ -n "${PX4_HOME_LAT:-}" ]; then
    export PX4_HOME_LAT PX4_HOME_LON PX4_HOME_ALT
  fi
  "$PX4_DIR/build/px4_sitl_default/bin/px4" -d
) < /dev/null > "$LOG_DIR/01c_px4_sitl.log" 2>&1 &
PIDS+=("$!")
wait_for_log "$LOG_DIR/01c_px4_sitl.log" "Startup script returned successfully" 90 "PX4 SITL"

# ── Stage 2 — MAVROS ───────────────────────────────────────────────────────
echo "── Stage 2/5: MAVROS ──"
# ROS2's setup.bash references variables (e.g. AMENT_TRACE_SETUP_FILES) it
# never guarantees are set, which trips `set -u` above and kills this whole
# script the instant it's sourced. Relax -u only around the source itself.
set +u
source /opt/ros/humble/setup.bash
set -u
(
  ros2 launch mavros px4.launch fcu_url:="$FCU_URL"
) > "$LOG_DIR/02_mavros.log" 2>&1 &
PIDS+=("$!")
wait_for_log "$LOG_DIR/02_mavros.log" "Got HEARTBEAT.*connected" 60 "MAVROS"

# ── Stage 3 — Camera bridge (Gazebo -> ROS2) ──────────────────────────────
# Non-fatal: a stuck/missing camera topic (known WSL GPU-rendering limitation
# on this project, plus the CAMERA_MODEL_NAME/"x500_0" mismatch noted above)
# must not take down the rest of an already-healthy stack. wait_for_topic's
# own exit 1 on timeout would otherwise fire this script's cleanup trap and
# kill PX4/Gazebo/MAVROS too -- confirmed live 2026-07-11, this exact failure
# mode silently nuked a fully-booted stack right after MAVROS connected.
echo "── Stage 3/5: Camera bridge ──"
CAMERA_TOPIC="/world/${GZ_WORLD}/model/${CAMERA_MODEL_NAME}/link/camera_link/sensor/camera/image"
(
  ros2 run ros_gz_bridge parameter_bridge \
    "${CAMERA_TOPIC}@sensor_msgs/msg/Image@gz.msgs.Image" \
    --ros-args -r "${CAMERA_TOPIC}:=/camera/image_raw"
) > "$LOG_DIR/03_camera_bridge.log" 2>&1 &
PIDS+=("$!")
if ! wait_for_topic_soft "/camera/image_raw" 30 "camera image"; then
  echo "⚠️  Camera bridge did not come up (known WSL rendering limitation) — continuing without it."
fi

# ── Stage 4 — All mission nodes ───────────────────────────────────────────
echo "── Stage 4/5: Mission nodes (drone_bringup) ──"
set +u
source "$DRONE_WS/install/setup.bash"
set -u
(
  ros2 launch drone_bringup full_mission.launch.py
) > "$LOG_DIR/04_full_mission.log" 2>&1 &
PIDS+=("$!")
# No single "all ready" log line exists across the 6 nodes today — polling
# /mission/status is the closest real signal that mission_manager is up.
# Soft wait: confirmed live 2026-07-11 that mission_manager was already up
# and healthily logging its state well inside this window while the topic
# still hadn't shown up in `ros2 topic list` yet (DDS discovery lag under
# the CPU load of everything else already running) -- a hard exit here was
# killing an already-working node stack over a slow-but-fine discovery, the
# same false-failure shape as the camera bridge fix above.
if ! wait_for_topic_soft "/mission/status" 30 "mission_manager"; then
  echo "⚠️  /mission/status not seen in ros2 topic list yet (DDS discovery lag) — check $LOG_DIR/04_full_mission.log; nodes may still be fine."
fi
echo "ℹ️  Nodes launched — check $LOG_DIR/04_full_mission.log if any node failed silently."
sleep 2   # small settle margin for the remaining 5 nodes to finish init

# ── Stage 5 — Backend bridge (FastAPI + rclpy) so the website can drive it ─
if [ "$WITH_BACKEND" -eq 1 ]; then
  echo "── Stage 5/5: Backend bridge (backend/main.py) ──"
  (
    # Supervisor: if the backend dies mid-flight, restart it after 2s so the
    # dashboard comes back on its own (the drone flies on regardless — this
    # only restores monitoring/control; main.py re-adopts the open mission
    # row via db.get_open_mission()). The trap keeps Ctrl+C on the main
    # script working: without it, killing this subshell would orphan the
    # running python3 instead of stopping it.
    cd "$BACKEND_DIR" || exit 1
    child=""
    trap '[ -n "$child" ] && kill "$child" 2>/dev/null; exit 0' INT TERM
    while true; do
      python3 main.py &
      child=$!
      wait "$child"
      echo "⚠️  backend exited — auto-restarting in 2s (Ctrl+C the main script to stop for real)"
      sleep 2
    done
  ) > "$LOG_DIR/05_backend_bridge.log" 2>&1 &
  PIDS+=("$!")
  # main.py logs "BridgeNode ready" once its ROS2 node + uvicorn thread are up
  wait_for_log "$LOG_DIR/05_backend_bridge.log" "BridgeNode ready" 20 "backend bridge"
  echo "🌐 Website can now reach the drone at http://localhost:8000"
  echo "   Use the website's START button — it publishes the same /mission/command"
  echo "   message the old manual CLI pub did, so you no longer need to run that by hand."
else
  echo "── Stage 5/5 skipped (--no-backend) — website will not be able to reach the drone. ──"
fi

# ── Optional: publish START via CLI (only for testing without the website) ─
if [ "$CLI_START" -eq 1 ]; then
  echo "── CLI START requested (--cli-start) ──"
  set +u; source "$DRONE_WS/install/setup.bash" 2>/dev/null || true; set -u
  ros2 topic pub /mission/command std_msgs/msg/String "data: 'START'" --once
  echo "🚀 Mission started via CLI."
fi

echo ""
echo "All stages up. Logs: $LOG_DIR"
if [ "$WITH_BACKEND" -eq 1 ] && [ "$CLI_START" -eq 0 ]; then
  echo "Now open the website and press START — no CLI command needed."
fi

# Run a one-shot health check so you see real status, not just "stages started"
STATUS_SCRIPT="$(dirname "$(readlink -f "$0")")/check_status.sh"
if [ -f "$STATUS_SCRIPT" ]; then
  sleep 2
  echo ""
  bash "$STATUS_SCRIPT"
else
  echo "(check_status.sh not found alongside this script — run it separately to verify health)"
fi

echo ""
echo "Press Ctrl+C to stop the whole stack."

# Keep the script alive so the trap can clean up on Ctrl+C
wait
