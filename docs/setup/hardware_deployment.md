# Real-Hardware Deployment — Pixhawk + Raspberry Pi 5 / Jetson + Website

How to run the exact stack you verified in Gazebo/SITL on a **real drone**: a Pixhawk
flight controller, a companion computer (Raspberry Pi 5 or Jetson) carrying MAVROS +
all ROS 2 nodes + the FastAPI backend, an OAK-D Lite as the down-camera, and the
operator's laptop/phone loading the website over WiFi.

Written 2026-07-14, after the SITL A→B round trip was verified 3× consecutively
(commit `98981db` and the fixes around it). Everything here was cross-checked against
the actual repo — file paths and launch arguments are real, not aspirational.

---

## 1. Architecture — what runs where

```
┌────────────────────────────  THE DRONE  ────────────────────────────┐
│                                                                     │
│  ┌──────────────┐  serial (USB or TELEM2)  ┌──────────────────────┐ │
│  │   PIXHAWK    │◄────────────────────────►│  COMPANION COMPUTER  │ │
│  │  (PX4)       │      MAVLink @ 57600-    │  (Pi 5 / Jetson)     │ │
│  │              │      921600 baud         │                      │ │
│  │ • GPS, IMU   │                          │ • MAVROS             │ │
│  │ • motors/ESC │                          │ • drone_base         │ │
│  │ • failsafes  │                          │ • waypoint_navigator │ │
│  │ • owns GPS   │                          │ • aruco_landing      │ │
│  │   navigation │                          │ • vision_node        │ │
│  │   (same as   │                          │ • mission_manager    │ │
│  │    SITL)     │                          │ • failsafe_monitor   │ │
│  └──────────────┘                          │ • OAK-D driver ──────┼─── publishes
│                                            │ • FastAPI backend    │    /camera/image_raw
│                       USB3 ┌───────────┐   │   (0.0.0.0:8000)     │
│                            │ OAK-D Lite│───┤ • frontend static    │
│                            └───────────┘   │   server (:3000)     │
│                                            └──────────┬───────────┘ │
└───────────────────────────────────────────────────────┼─────────────┘
                                                  WiFi   │
                                            ┌────────────▼───────────┐
                                            │  OPERATOR LAPTOP/PHONE │
                                            │  browser only:         │
                                            │  http://<pi-ip>:3000   │
                                            │  API+WS → :8000        │
                                            └────────────────────────┘
```

**What stays identical to sim:** every ROS topic name, all mission logic (including the
2026-07-14 ground-relative altitude fixes — they are correct on real hardware too),
the whole website, the WebSocket health gate, the SQLite travel log.

**What disappears:** Gazebo, the PX4 SITL binary, runtime ArUco marker *spawning*
(markers get printed instead), and laptop-location home syncing (real GPS sets home).

**Principle unchanged:** the dashboard is monitoring/командование only — losing WiFi to
the laptop must never crash the mission. The failsafe_monitor watching the *onboard*
links (MAVROS↔PX4, node heartbeats) rides on the companion computer and keeps working
with no ground link at all.

---

## 2. Hardware & wiring

### 2.1 Pixhawk ↔ companion serial link

**Option A — USB (do this first).** Pixhawk micro-USB → companion USB port.
Appears as `/dev/ttyACM0`. Zero wiring, works today:

```bash
fcu_url:=/dev/ttyACM0:57600
```

Add your user to the serial group once: `sudo usermod -aG dialout $USER` (relogin).
Caveats: the USB port is physically fragile for flight and PX4 pauses USB MAVLink
while the safety switch is unpressed on some boards — fine for bench, not the
permanent build.

**Option B — TELEM2 UART (the permanent build).**

| Pixhawk TELEM2 pin | → | Pi 5 GPIO header | Jetson (40-pin) |
|---|---|---|---|
| TX  | → | RX = GPIO15 (pin 10) | UART1 RX (pin 10) |
| RX  | → | TX = GPIO14 (pin 8)  | UART1 TX (pin 8)  |
| GND | → | GND (pin 6)          | GND (pin 6)       |
| VCC | — | **do not connect** — power the Pi separately | same |

PX4 params (QGroundControl once): `MAV_1_CONFIG = TELEM2`, `MAV_1_MODE = Onboard`,
`SER_TEL2_BAUD = 921600`.

Pi 5: enable the UART (`dtparam=uart0=on` in `/boot/firmware/config.txt`, disable the
serial console in `raspi-config`), then:

```bash
fcu_url:=/dev/ttyAMA0:921600     # Pi 5 GPIO UART (or /dev/serial0)
```

A USB-FTDI adapter into TELEM2 is an equally good middle path (`/dev/ttyUSB0:921600`)
and avoids GPIO console headaches.

### 2.2 Camera & power

- **OAK-D Lite** → companion **USB3** port (blue). It is powered over USB — budget for it.
- **Power:** Pi 5 wants 5 V/5 A; Jetson 5 V/4 A+. Use a dedicated 5 V BEC from the main
  battery rated ≥5 A. **Never** power the companion or camera from the Pixhawk's rails.
- All-up weight goes into the website's hardware-profile page so the range estimator
  gates missions correctly.

---

## 3. Companion computer software install

Works the same on Pi 5 and Jetson; differences called out in the table.

|  | Raspberry Pi 5 | Jetson (Orin Nano / Nano) |
|---|---|---|
| OS | Ubuntu **22.04** Server 64-bit (Raspberry Pi Imager) | JetPack (L4T). Orin JetPack 6 is Ubuntu 22.04-based ✔. Original Nano is stuck on 18.04 → runs ROS 2 Humble only via Docker — prefer Orin Nano (the repo roadmap already targets it) |
| OpenCV | `sudo apt install python3-opencv` | Use NVIDIA's bundled OpenCV; don't pip-install over it |
| Extra | — | Fan/power mode: `sudo nvpmodel -m 0` for max perf |

ROS 2 Humble needs Ubuntu 22.04 — same distro as the WSL workspace, so the packages
build unchanged.

```bash
# 1. ROS 2 Humble + MAVROS
sudo apt install ros-humble-ros-base ros-humble-mavros ros-humble-mavros-extras \
                 ros-humble-cv-bridge python3-colcon-common-extensions
# MAVROS needs geographiclib datasets once:
sudo bash /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh

# 2. The workspace — same layout & commands as the WSL install (CLAUDE.md §1)
mkdir -p ~/drone_ws2/src && cd ~/drone_ws2/src
git clone https://github.com/aadhi4200/autonomous_drone_ros2.git
git clone https://github.com/aadhi4200/Drone-A-to-B-waypoint-Nav-system.git frontend_bridge
cd ~/drone_ws2
source /opt/ros/humble/setup.bash
colcon build --base-paths src/autonomous_drone_ros2/src/autonomous_drone_ros2
# (no --symlink-install — same setup.cfg gotcha as documented for WSL)

# 3. Backend python deps (rclpy/cv_bridge come from ROS, NOT pip)
pip3 install fastapi uvicorn pydantic

# 4. OAK-D Lite driver
sudo apt install ros-humble-depthai-ros
# udev rule so it enumerates without sudo:
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | \
  sudo tee /etc/udev/rules.d/80-movidius.rules && sudo udevadm control --reload-rules
```

---

## 4. Bring-up sequence — bench test, PROPS OFF

Do these in order; each step has an observable pass condition. Source the env in every
terminal: `source /opt/ros/humble/setup.bash && source ~/drone_ws2/install/setup.bash`.

**Step 1 — MAVROS talks to the Pixhawk:**
```bash
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:57600
```
PASS: `Got HEARTBEAT ... connected`. (Same readiness string the SITL launcher watches —
it is transport-agnostic.)

**Step 2 — real GPS:**
```bash
ros2 topic echo /mavros/state          # connected: true
ros2 topic echo /mavros/global_position/global   # your real lat/lon (needs sky view)
```
Home comes from GPS lock automatically — **no location sync, no `.last_synced_home`**.
(waypoint_navigator still reads that file as a pre-GPS fallback; harmless — the real
MAVROS home overrides it the moment it arrives.)

**Step 3 — camera publishes the topic the whole stack expects:**
```bash
ros2 launch depthai_ros_driver camera.launch.py \
    --ros-args -r /oak/rgb/image_raw:=/camera/image_raw
ros2 topic hz /camera/image_raw        # PASS: steady rate
```
(Exact source topic name depends on the depthai-ros version — check `ros2 topic list`.
Anything publishing `sensor_msgs/Image` on `/camera/image_raw` makes the entire vision
chain, backend stream, and website camera panel work with **zero code changes** —
`vision.launch.py` hardware mode is designed around exactly this seam.)

**Step 4 — mission nodes, hardware mode (no Gazebo anything):**
```bash
ros2 launch drone_bringup full_mission.launch.py mode:=hardware
```
⚠️ Do **not** use `full_stack.launch.py` on hardware — it unconditionally starts PX4
SITL + Gazebo (known gap, §8).

**Step 5 — backend:**
```bash
cd ~/drone_ws2/src/frontend_bridge/backend && python3 main.py
# then lock in hardware mode (server-side blocks sim-only marker generation):
curl -X POST http://localhost:8000/system/mode -H 'Content-Type: application/json' \
     -d '{"mode":"hardware"}'
```
The backend already binds `0.0.0.0:8000` — reachable over WiFi as-is.

**Step 6 — website.** One code edit is required (the only one): the backend's CORS
list (`backend/main.py` ~line 56) allows only localhost origins. Add your Pi's origin:

```python
allow_origins=[..., "http://<pi-ip>:3000"],   # or "*" for field use
```

Then build the frontend against the Pi's address and serve the static build:

```bash
cd ~/drone_ws2/src/frontend_bridge
VITE_API_URL=http://<pi-ip>:8000 npm run build     # baked at build time!
npx vite preview --host 0.0.0.0 --port 3000        # serves dist/
```

`VITE_API_URL` drives **both** the REST base and the WebSocket URL (`src/api.ts` derives
`ws://` from it), so one env var covers everything.

**Step 7 — the gate goes green.** Open `http://<pi-ip>:3000` on the laptop/phone. The
same `/ws/system-status` preflight panel from sim must go all-clear on real hardware:
MAVROS connected, all node heartbeats fresh, GPS lock, battery. START stays blocked
until it does — this is your bench-test scoreboard.

---

## 5. Networking in the field

**Recommended: the companion computer is its own WiFi hotspot** — no router needed at
the field, fixed IP, nothing else on the network:

```bash
sudo nmcli device wifi hotspot ssid droneAP password <pw> ifname wlan0
# companion is then always 10.42.0.1 →  website at http://10.42.0.1:3000
```

Build the frontend once with `VITE_API_URL=http://10.42.0.1:8000` and it works at every
field session. Ports used: 3000 (website), 8000 (API + WebSocket + camera stream).

Losing this WiFi mid-flight is **not** an emergency: everything flight-critical is
onboard; failsafe RTH triggers only on *onboard* link loss (MAVROS↔PX4, node
heartbeats), exactly like the SITL behavior you verified.

---

## 6. PX4 parameters — sim settings that must NOT carry over

The 2026-07-13/14 SITL debugging set three params in the simulator. They live in the
SITL eeprom, not your real Pixhawk — but be explicit about what applies where:

| Param | SITL value we set | Real Pixhawk |
|---|---|---|
| `SIM_BAT_DRAIN` | 1800 | Doesn't exist on hardware — real battery is real |
| `COM_RCL_EXCEPT` | 4 (ignore RC loss in Offboard) | **Leave at 0.** This was a sim-only decision because SITL has no RC. On a real drone RC-loss failsafe is a safety feature — keep it, fly with an RC transmitter as backup |
| `COM_OF_LOSS_T` | 5.0 s | Start at default (1 s). The Pi runs lean (no Gazebo eating CPU), so setpoint-stream stalls are far less likely. Only raise it if you *measure* stream gaps |
| `NAV_DLL_ACT` | backend sets 0 at connect | Reconsider: with no QGC connected this avoids GCS-loss failsafe; acceptable since failsafe_monitor covers link loss — but know that's the trade |

Do configure on the real Pixhawk: battery failsafe voltage thresholds
(`BAT1_V_EMPTY`, `COM_LOW_BAT_ACT`), geofence (the website's QGC-style fence push
works on hardware unchanged), and standard PX4 sensor/RC calibration in QGC first.

---

## 7. The landing marker & first flights

**Marker:** print it instead of spawning it. Generate the PNG with the same function
the sim uses (`drone_interfaces/aruco_marker.py: generate_marker_png(marker_id, path)`
— DICT_6X6_250). Print **at least 40–60 cm square** on matte paper/board (the sim pads
are 2 m; detection height scales with size — bigger = acquired from higher). Keep a
white border around the black marker. Landing is pixel-offset based (no pose
estimation), so exact print size is not calibration-critical — visibility is.
Place it at the mapped B coordinates; the website's hardware mode already tells the
operator the marker ID to print instead of offering spawn.

**Flight test ladder — one new variable per flight:**
1. **Bench, props off:** full §4 bring-up; watch website telemetry, arm/disarm from the site.
2. **Props off, mission dry-run:** START a mission; verify OFFBOARD engages, motors
   respond, state machine walks TAKEOFF→GOTO (it will believe it's flying; that's fine).
3. **Manual hover on RC** (no autonomy) — airworthiness check.
4. **Website mission, tiny:** B = 5 m away, alt 2.5 m, open field, RC in hand ready to
   take over (mode switch to Position kills OFFBOARD instantly).
5. **Full A→B with marker landing**, then multi-stop.

---

## 8. Known gaps = Phase-4 code backlog

Found during exploration; the guide works around all of them today:

1. **CORS hardcoded to localhost** (`backend/main.py:56-62`) — the one-line edit in §4
   step 6. Proper fix: config-driven origins or same-origin serving (see 4).
2. **`full_stack.launch.py` always starts PX4 SITL + Gazebo** — needs a `mode:=hardware`
   branch that skips both and points `fcu_url` at serial; until then use
   `mavros px4.launch` + `full_mission.launch.py mode:=hardware` (§4).
3. **No OAK-D driver wrapper in-repo** — depthai-ros + remap covers it; a thin
   `drone_camera` hardware node could own exposure/fps defaults later.
4. **No production frontend serving** — `vite preview` works; the clean fix is FastAPI
   `StaticFiles` mounting `dist/` so website + API share one origin (also kills gap 1).
5. **systemd units** — for a flight-ready drone, MAVROS, mission nodes, camera driver,
   and backend should be `systemd` services that start on boot, so a field power cycle
   needs no SSH session. (The backend already survives restarts via its supervisor +
   mission-adoption logic from 2026-07-12.)

---

## Appendix A — Full install command reference (Pi 5 / Jetson)

Copy-paste sequence expanding §3. Run stages in order.

### A.0 Operating system

**Raspberry Pi 5:** flash **Ubuntu Server 22.04 LTS 64-bit** with Raspberry Pi Imager
(⚠️ not Raspberry Pi OS — ROS 2 Humble requires Ubuntu 22.04). Set user/WiFi in the
imager, then:

```bash
sudo apt update && sudo apt full-upgrade -y
sudo apt install -y git curl nano htop
```

**Jetson Orin Nano:** flash **JetPack 6** (Ubuntu 22.04-based) from NVIDIA. Then:

```bash
sudo apt update && sudo apt full-upgrade -y
sudo nvpmodel -m 0 && sudo jetson_clocks     # max performance mode
```

⚠️ Original (non-Orin) Jetson Nano is stuck on Ubuntu 18.04 — Humble won't install
natively (Docker only). Prefer Pi 5 or Orin Nano (the roadmap targets Orin).

### A.1 ROS 2 Humble (identical on both)

```bash
sudo apt install -y software-properties-common
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
     -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
     | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-humble-ros-base ros-dev-tools python3-colcon-common-extensions
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && source ~/.bashrc
```

(`ros-base` = no GUI tools — right for a headless companion computer.)

### A.2 MAVROS + OAK-D + Python deps

```bash
sudo apt install -y ros-humble-mavros ros-humble-mavros-extras
sudo bash /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh   # required once

sudo apt install -y ros-humble-depthai-ros
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | \
  sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger

sudo apt install -y ros-humble-cv-bridge python3-opencv python3-pip
#  Jetson: if `python3 -c "import cv2"` already works (NVIDIA's build), SKIP
#  python3-opencv and never pip-install opencv-python over it.
pip3 install fastapi uvicorn pydantic

sudo usermod -aG dialout $USER      # Pixhawk serial access; relogin afterwards
```

### A.3 Workspace (same commands as the WSL install)

```bash
mkdir -p ~/drone_ws2/src && cd ~/drone_ws2/src
git clone https://github.com/aadhi4200/autonomous_drone_ros2.git
git clone https://github.com/aadhi4200/Drone-A-to-B-waypoint-Nav-system.git frontend_bridge
cd ~/drone_ws2
source /opt/ros/humble/setup.bash
colcon build --base-paths src/autonomous_drone_ros2/src/autonomous_drone_ros2
#  (never --symlink-install — setup.cfg gotcha, same as WSL)
echo "source ~/drone_ws2/install/setup.bash" >> ~/.bashrc && source ~/.bashrc
```

### A.4 Frontend

Recommended: build on the laptop/WSL (Node is slow on a Pi), copy `dist/` over:

```bash
# laptop/WSL:
cd ~/drone_ws2/src/frontend_bridge
VITE_API_URL=http://10.42.0.1:8000 npm run build     # 10.42.0.1 = Pi hotspot IP
scp -r dist/ <user>@<pi-ip>:~/drone_ws2/src/frontend_bridge/
```

Or build on the companion:

```bash
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.7/install.sh | bash
source ~/.bashrc && nvm install 20
cd ~/drone_ws2/src/frontend_bridge && npm install
VITE_API_URL=http://10.42.0.1:8000 npm run build
```

Serve: `npx vite preview --host 0.0.0.0 --port 3000`
(or `cd dist && python3 -m http.server 3000`).

### A.5 Pi-only: enable GPIO UART (TELEM2 wiring only; USB needs none of this)

```bash
sudo sed -i '$ a enable_uart=1' /boot/firmware/config.txt
sudo sed -i 's/console=serial0,115200 //' /boot/firmware/cmdline.txt
sudo reboot
# Pixhawk TELEM2 link is then /dev/ttyAMA0 (or /dev/serial0)
```

### A.6 Field WiFi hotspot

```bash
sudo nmcli device wifi hotspot ssid droneAP password YourPassword ifname wlan0
nmcli connection modify Hotspot connection.autoconnect yes
# companion is always 10.42.0.1 → website at http://10.42.0.1:3000
```

Reminder: the one required code edit — add `"http://10.42.0.1:3000"` to
`allow_origins` in `backend/main.py` (~line 56) — is described in §4 step 6.
