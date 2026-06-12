from __future__ import annotations

import math

import numpy as np


def rot_x(angle_rad: float) -> np.ndarray:
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]], dtype=np.float64)


def rot_y(angle_rad: float) -> np.ndarray:
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=np.float64)


def rot_z(angle_rad: float) -> np.ndarray:
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)


def euler_zyx_to_matrix(roll_rad: float, pitch_rad: float, yaw_rad: float) -> np.ndarray:
    return rot_z(yaw_rad) @ rot_y(pitch_rad) @ rot_x(roll_rad)


def body_to_ned_matrix(roll_rad: float, pitch_rad: float, yaw_rad: float) -> np.ndarray:
    return euler_zyx_to_matrix(roll_rad, pitch_rad, yaw_rad)


def camera_to_body_matrix(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)
    return euler_zyx_to_matrix(roll, pitch, yaw)

