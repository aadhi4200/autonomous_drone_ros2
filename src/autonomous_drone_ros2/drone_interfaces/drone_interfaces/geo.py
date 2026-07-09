"""
geo.py | Package: drone_interfaces
Shared GPS <-> local-frame conversion. Single source of truth so every node
that needs "how far is this GPS point from home" (waypoint_navigator,
the marker-spawn endpoint, the range estimator) uses the same approximation.
"""
import math

EARTH_RADIUS_M = 6371000


def gps_to_local(home_lat: float, home_lon: float, lat: float, lon: float):
    """Equirectangular approximation, valid for the short (<~few km) distances
    a single delivery mission covers. Returns (north, east) in metres."""
    north = math.radians(lat - home_lat) * EARTH_RADIUS_M
    east = math.radians(lon - home_lon) * EARTH_RADIUS_M * math.cos(math.radians(home_lat))
    return north, east


def gps_distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    north, east = gps_to_local(lat1, lon1, lat2, lon2)
    return math.hypot(north, east)
