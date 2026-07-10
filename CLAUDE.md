# drone_ws2 — project notes

ROS2 Humble + PX4 SITL (Gazebo Harmonic) + MAVROS + a FastAPI/React
"frontend_bridge" website that gates mission controls behind a live
readiness check (MAVROS connection, GPS lock, battery, home-position sync,
geofence).

Two ROS2 packages live under `src/`:
- `autonomous_drone_ros2/` — the ROS2 side (drone_base, drone_bringup,
  drone_camera, drone_controller, drone_interfaces, mission nodes)
- `frontend_bridge/` — `backend/main.py` (FastAPI + rclpy bridge, port
  8000) and a Vite/React frontend (`npm run dev`, port 3000 by default —
  **Vite auto-bumps to 3001+ if 3000 is taken**, check the terminal's own
  "Local:" line rather than assuming the port)

## Target machine for this checkout: WSL2 on Windows

Ryzen 5 5600 (6c/12t), RX6600, 32GB RAM. This replaces an earlier
7.6GB-RAM laptop where Gazebo's Ruby-based `gz` CLI launcher segfaulted
in `libgcc_s.so.1` every 10-40 minutes under memory/swap pressure (system
was down to <400MB free, swap 90% full) — killing the sim server out
from under `make px4_sitl` and cascading into a `ninja: build stopped:
subcommand failed` exit. 32GB should make that failure mode go away, but
if you ever see that exact segfault signature again
(`journalctl -k | grep -i segfault`), it's memory pressure, not a code
bug — check `free -h` first.

### WSL2 setup checklist
1. **WSL2, not WSL1** — required for GPU passthrough and the systemd/networking
   behavior ROS2 and Gazebo expect. Ubuntu 22.04 (matches ROS2 Humble).
2. **WSLg** for the Gazebo GUI — built into Windows 11 (and backported to
   Windows 10 21H2+). Install the latest AMD Adrenalin driver *on the
   Windows host* with WSL support; WSLg passes rendering through via
   D3D12, so the RX6600 needs a current driver, not anything installed
   inside WSL itself.
3. **`.wslconfig`** (in `%UserProfile%\.wslconfig` on the Windows side) —
   default WSL2 caps memory at ~50% of host RAM, which would land around
   16GB here; be explicit instead so Gazebo + PX4 + ROS2 + browser all
   have headroom:
   ```ini
   [wsl2]
   memory=24GB
   processors=12
   swap=8GB
   ```
4. **Clone inside the WSL filesystem**, not `/mnt/c/...` — e.g.
   `~/PX4-Autopilot` and `~/drone_ws2` under the Linux home directory.
   `/mnt/c` I/O is slow enough to make colcon/PX4 builds painful.
5. **Verify GPU acceleration** once Gazebo's up:
   `glxinfo | grep "OpenGL renderer"` should show the RX6600 via Mesa/D3D12,
   not `llvmpipe` (software rendering) — if it's software-rendering,
   Gazebo will be slow and the `PX4_GZ_SIM_RENDER_ENGINE=ogre` fallback
   mentioned in `launch_full_sim.sh` (for weak GPUs) shouldn't be needed
   here.
6. Install: ROS2 Humble, `ros-humble-ros-gzharmonic*` (Gazebo Harmonic
   ROS2 bridge packages), MAVROS, Node.js/npm, `pip install fastapi
   uvicorn opencv-python`, and run
   `autonomous_drone_ros2/install_geographiclib_datasets.sh`.

## Running the stack

Two ways to launch, both documented in `src/autonomous_drone_ros2/`:
- `launch_full_sim.sh` — automated, staged (PX4 → MAVROS → camera bridge
  → mission nodes → backend), reads `~/drone_ws2/.last_synced_home` and
  exports `PX4_HOME_LAT/LON/ALT` before starting PX4. Use `--headless`
  for a lighter/faster run without the GUI.
- `manual_6_terminals.sh` — reference commands for running the same 6
  stages as separate terminals instead of the script above.

**Home-sync gotcha (the whole reason the readiness gate has a
`home_position_match` check):** the website's "sync location" button
writes the browser's geolocation to `~/drone_ws2/.last_synced_home`, but
PX4 only reads that file *once*, at `make px4_sitl` startup, to set its
spawn location. Syncing from the website *after* PX4 is already running
has no effect on the live sim — you must sync first, then (re)launch
PX4. `POST /system/set-home` returns `relaunch_needed: true` when it
detects this situation; the frontend surfaces it as a log-line warning.

**Workspace gotcha:** this laptop also has a separate, older `~/drone_ws`
(no "2") checkout with a different `full_mission.launch.py` — always
source `~/drone_ws2/install/setup.bash`, not `~/drone_ws`.

**Backgrounding PX4 non-interactively:** `make px4_sitl` starts an
interactive `pxh>` shell. If you background it with stdin redirected
from `/dev/null` in a non-interactive context (no real controlling tty),
it can busy-loop on instant EOF and fill its log at tens of MB/sec
instead of treating `/dev/null` as "non-interactive and quiet" — holding
stdin open on a FIFO instead avoids this:
```bash
mkfifo /tmp/px4_stdin.fifo; exec 3<> /tmp/px4_stdin.fifo
setsid make px4_sitl gz_x500_lidar_cam_down <&3 > px4.log 2>&1 &
```
Not an issue when you just type the command into a real terminal.

The `frontend_bridge/README.md` `cd` path (`~/drone_ws/src/autonomous_drone_ros2`)
is stale — the actual backend lives at
`~/drone_ws2/src/frontend_bridge/backend/main.py`, run directly with
`python3 main.py` (no colcon build needed, it's a plain script — only
`drone_interfaces`/`mavros_msgs` need `~/drone_ws2/install/setup.bash`
sourced first for the imports to resolve).
