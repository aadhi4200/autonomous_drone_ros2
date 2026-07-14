"""
constants.py | Package: drone_interfaces
Shared constants across all drone packages.
"""

# ── Altitudes ────────────────────────────────────────
TAKEOFF_ALTITUDE  = 5.0   # metres
CRUISE_ALTITUDE   = 5.0   # metres
RTH_ALTITUDE      = 7.0   # metres

# ── Navigation ───────────────────────────────────────
WP_TOLERANCE_SITL = 0.4   # metres (simulation)
WP_TOLERANCE_REAL = 1.0   # metres (real hardware)
WP_TIMEOUT        = 60.0  # seconds

# ── ArUco landing ────────────────────────────────────
ARUCO_MARKER_ID     = 17
ARUCO_ID_AUTO_START = 18   # runtime-generated marker IDs start here (17 is reserved/tested)
ALIGN_THRESHOLD   = 20.0  # pixels
DESCEND_RATE      = 0.15  # m/s
LAND_ALTITUDE     = 0.3   # metres

# ── Mission ───────────────────────────────────────────
WAIT_ON_GROUND    = 5.0   # seconds (payload drop)
SEARCH_TIMEOUT    = 30.0  # seconds

# ── Connectivity / heartbeat ──────────────────────────
NODE_HEARTBEAT_STALE_S = 6.0   # a node is considered dead if silent this long (3x slowest 1 Hz heartbeat; 2.0 false-fired under WSL CPU-load timer slips, seen 5x on 2026-07-13)
BATTERY_HEARTBEAT_STALE_S = 5.0  # /mavros/battery publishes ~1 Hz — looser than node heartbeats
MAVROS_STATE_STALE_S = 3.0       # /mavros/state publishes at 1 Hz — dead if silent this long

# ── Topics ────────────────────────────────────────────
TOPIC_BASE_STATUS   = "/drone_base/status"
TOPIC_BASE_CMD      = "/drone_base/command"
TOPIC_NAV_STATUS    = "/waypoint_nav/status"
TOPIC_NAV_CMD       = "/waypoint_nav/command"
TOPIC_ARUCO_STATUS  = "/aruco_landing/status"
TOPIC_ARUCO_CMD     = "/aruco_landing/command"
TOPIC_MISSION_CMD   = "/mission/command"
TOPIC_MISSION_STATUS= "/mission/status"
TOPIC_MISSION_WAYPOINTS = "/mission/waypoints"
TOPIC_MISSION_SAFETY_EVENT = "/mission/safety_event"
TOPIC_CAMERA        = "/camera/image_raw"
TOPIC_LANDING_TARGET= "/vision/landing_target"
TOPIC_VISION_TARGET_ID = "/vision/target_marker_id"
TOPIC_VISION_HEARTBEAT = "/vision_node/heartbeat"
TOPIC_CAMERA_HEARTBEAT = "/camera_node/heartbeat"
