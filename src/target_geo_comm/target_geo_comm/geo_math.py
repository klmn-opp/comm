from __future__ import annotations

import math

EARTH_RADIUS_M = 6378137.0


def ned_offset_to_geodetic(
    origin_lat_deg: float,
    origin_lon_deg: float,
    origin_alt_m: float,
    north_m: float,
    east_m: float,
    down_m: float,
) -> tuple[float, float, float]:
    lat_rad = math.radians(origin_lat_deg)
    d_lat = north_m / EARTH_RADIUS_M
    cos_lat = max(math.cos(lat_rad), 1e-9)
    d_lon = east_m / (EARTH_RADIUS_M * cos_lat)

    target_lat = origin_lat_deg + math.degrees(d_lat)
    target_lon = origin_lon_deg + math.degrees(d_lon)
    target_alt = origin_alt_m - down_m
    return target_lat, target_lon, target_alt

