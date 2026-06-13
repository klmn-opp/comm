from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float64MultiArray, String
from rclpy.time import Time

from target_geo_comm.geo_math import ned_offset_to_geodetic
from target_geo_comm.transforms import body_to_ned_matrix, camera_to_body_matrix

try:
    from pymavlink import mavutil
except ImportError:  # pragma: no cover
    mavutil = None


@dataclass
class AircraftState:
    lat_deg: float = 0.0
    lon_deg: float = 0.0
    alt_msl_m: float = 0.0
    rel_alt_m: float = 0.0
    roll_rad: float = 0.0
    pitch_rad: float = 0.0
    yaw_rad: float = 0.0
    vx_mps: float = 0.0
    vy_mps: float = 0.0
    vz_mps: float = 0.0
    airspeed_mps: float = 0.0
    groundspeed_mps: float = 0.0
    heading_deg: float = 0.0
    gps_fix_type: int = 0
    satellites_visible: int = 0
    last_position_time: float = 0.0
    last_attitude_time: float = 0.0
    last_speed_time: float = 0.0

    def is_ready(self, max_age_s: float) -> bool:
        now = time.time()
        return (
            now - self.last_position_time <= max_age_s
            and now - self.last_attitude_time <= max_age_s
            and now - self.last_speed_time <= max_age_s
        )


class TargetGeoNode(Node):
    def __init__(self) -> None:
        super().__init__("target_geo_node")

        self.declare_parameter("mavlink_connection", "udp:127.0.0.1:14550")
        self.declare_parameter("target_pose_topic", "/target_pose_camera")
        self.declare_parameter("mavlink_rate_hz", 20.0)
        self.declare_parameter("state_max_age_s", 1.0)
        self.declare_parameter("target_max_age_s", 0.5)
        self.declare_parameter("target_altitude_mode", "pnp")
        self.declare_parameter("target_altitude_offset_m", 0.0)
        self.declare_parameter("use_sim_target_geo", False)
        self.declare_parameter("sim_target_mode", "fixed_distance")
        self.declare_parameter("sim_target_distance_m", 80.0)
        self.declare_parameter("sim_target_distance_end_m", 0.0)
        self.declare_parameter("sim_target_closing_speed_mps", 0.0)
        self.declare_parameter("sim_target_reset_delay_s", 3.0)
        self.declare_parameter("sim_min_direction_distance_m", 3.0)
        self.declare_parameter("sim_target_bearing_offset_deg", 0.0)
        self.declare_parameter("publish_raw_target_geo", True)

        self.declare_parameter("camera_roll_deg", 0.0)
        self.declare_parameter("camera_pitch_deg", 90.0)
        self.declare_parameter("camera_yaw_deg", 0.0)
        self.declare_parameter("camera_x_m", 0.0)
        self.declare_parameter("camera_y_m", 0.0)
        self.declare_parameter("camera_z_m", 0.0)

        self.mavlink_connection = self.get_parameter("mavlink_connection").get_parameter_value().string_value
        target_pose_topic = self.get_parameter("target_pose_topic").get_parameter_value().string_value
        self.mavlink_rate_hz = float(self.get_parameter("mavlink_rate_hz").value)
        self.state_max_age_s = float(self.get_parameter("state_max_age_s").value)
        self.target_max_age_s = float(self.get_parameter("target_max_age_s").value)
        self.target_altitude_mode = self.get_parameter("target_altitude_mode").get_parameter_value().string_value
        self.target_altitude_offset_m = float(self.get_parameter("target_altitude_offset_m").value)
        self.use_sim_target_geo = bool(self.get_parameter("use_sim_target_geo").value)
        self.sim_target_mode = self.get_parameter("sim_target_mode").get_parameter_value().string_value
        self.sim_target_distance_m = float(self.get_parameter("sim_target_distance_m").value)
        self.sim_target_distance_end_m = float(self.get_parameter("sim_target_distance_end_m").value)
        self.sim_target_closing_speed_mps = float(self.get_parameter("sim_target_closing_speed_mps").value)
        self.sim_target_reset_delay_s = float(self.get_parameter("sim_target_reset_delay_s").value)
        self.sim_min_direction_distance_m = float(self.get_parameter("sim_min_direction_distance_m").value)
        self.sim_target_bearing_offset_deg = float(self.get_parameter("sim_target_bearing_offset_deg").value)
        self.publish_raw_target_geo = bool(self.get_parameter("publish_raw_target_geo").value)
        self.sim_start_time: Optional[float] = None
        self.sim_end_reached_time: Optional[float] = None
        self.warned_unsupported_sim_mode = False

        self.r_body_camera = camera_to_body_matrix(
            float(self.get_parameter("camera_roll_deg").value),
            float(self.get_parameter("camera_pitch_deg").value),
            float(self.get_parameter("camera_yaw_deg").value),
        )
        self.t_body_camera = np.array(
            [
                float(self.get_parameter("camera_x_m").value),
                float(self.get_parameter("camera_y_m").value),
                float(self.get_parameter("camera_z_m").value),
            ],
            dtype=np.float64,
        )

        self.state = AircraftState()
        self.state_lock = threading.Lock()
        self.mav = None
        self.stop_event = threading.Event()
        self.mav_thread = threading.Thread(target=self.mavlink_loop, daemon=True)

        self.target_sub = self.create_subscription(PoseStamped, target_pose_topic, self.target_pose_callback, 10)
        self.aircraft_state_pub = self.create_publisher(Float64MultiArray, "/aircraft_state", 10)
        self.target_local_pub = self.create_publisher(PointStamped, "/target_local_ned", 10)
        self.target_geo_raw_pub = self.create_publisher(NavSatFix, "/target_geo_raw", 10)
        self.target_geo_pub = self.create_publisher(NavSatFix, "/target_geo", 10)
        self.status_pub = self.create_publisher(String, "/target_geo_status", 10)

        self.state_timer = self.create_timer(0.1, self.publish_aircraft_state)
        self.mav_thread.start()

        self.get_logger().info(
            f"target_geo_node started, mavlink={self.mavlink_connection}, target_pose_topic={target_pose_topic}"
        )

    def destroy_node(self) -> bool:
        self.stop_event.set()
        if self.mav is not None:
            try:
                self.mav.close()
            except Exception:
                pass
        if self.mav_thread.is_alive():
            self.mav_thread.join(timeout=2.0)
        return super().destroy_node()

    def request_msg_rate(self, name: str, hz: float) -> None:
        if self.mav is None or mavutil is None or hz <= 0.0:
            return
        msg_id = getattr(mavutil.mavlink, f"MAVLINK_MSG_ID_{name}", None)
        if msg_id is None:
            return
        interval_us = int(1_000_000 / hz)
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            msg_id,
            interval_us,
            0,
            0,
            0,
            0,
            0,
        )

    def mavlink_loop(self) -> None:
        if mavutil is None:
            self.get_logger().error("pymavlink is not installed in this environment")
            return

        while not self.stop_event.is_set():
            try:
                self.get_logger().info(f"connecting MAVLink: {self.mavlink_connection}")
                self.mav = mavutil.mavlink_connection(self.mavlink_connection)
                heartbeat = self.mav.wait_heartbeat(timeout=10)
                if not heartbeat:
                    raise TimeoutError("MAVLink heartbeat timeout")

                self.get_logger().info(
                    f"MAVLink connected sys={self.mav.target_system} comp={self.mav.target_component}"
                )
                for name in [
                    "GLOBAL_POSITION_INT",
                    "ATTITUDE",
                    "VFR_HUD",
                    "GPS_RAW_INT",
                    "WIND",
                    "TERRAIN_REPORT",
                    "EKF_STATUS_REPORT",
                ]:
                    self.request_msg_rate(name, self.mavlink_rate_hz)

                wanted = ["GLOBAL_POSITION_INT", "ATTITUDE", "VFR_HUD", "GPS_RAW_INT"]
                while not self.stop_event.is_set():
                    msg = self.mav.recv_match(type=wanted, blocking=True, timeout=1)
                    if msg is None:
                        continue
                    self.handle_mavlink_msg(msg)
            except Exception as exc:
                if self.stop_event.is_set():
                    break
                self.get_logger().warning(f"MAVLink loop error: {exc}; reconnecting in 2s")
                time.sleep(2.0)

    def handle_mavlink_msg(self, msg) -> None:
        now = time.time()
        msg_type = msg.get_type()
        with self.state_lock:
            if msg_type == "GLOBAL_POSITION_INT":
                self.state.lat_deg = msg.lat / 1e7
                self.state.lon_deg = msg.lon / 1e7
                self.state.alt_msl_m = max(0.0, msg.alt / 1000.0)
                self.state.rel_alt_m = msg.relative_alt / 1000.0
                self.state.vx_mps = msg.vx / 100.0
                self.state.vy_mps = msg.vy / 100.0
                self.state.vz_mps = msg.vz / 100.0
                self.state.heading_deg = msg.hdg / 100.0 if msg.hdg != 65535 else self.state.heading_deg
                self.state.last_position_time = now
            elif msg_type == "ATTITUDE":
                self.state.roll_rad = msg.roll
                self.state.pitch_rad = msg.pitch
                self.state.yaw_rad = msg.yaw
                self.state.last_attitude_time = now
            elif msg_type == "VFR_HUD":
                self.state.airspeed_mps = msg.airspeed
                self.state.groundspeed_mps = msg.groundspeed
                self.state.heading_deg = float(msg.heading)
                self.state.last_speed_time = now
            elif msg_type == "GPS_RAW_INT":
                self.state.gps_fix_type = int(msg.fix_type)
                self.state.satellites_visible = int(msg.satellites_visible)

    def publish_aircraft_state(self) -> None:
        with self.state_lock:
            state = self.state
            msg = Float64MultiArray()
            msg.data = [
                state.lat_deg,
                state.lon_deg,
                state.alt_msl_m,
                state.rel_alt_m,
                state.roll_rad,
                state.pitch_rad,
                state.yaw_rad,
                state.vx_mps,
                state.vy_mps,
                state.vz_mps,
                state.airspeed_mps,
                state.groundspeed_mps,
                state.heading_deg,
            ]
        self.aircraft_state_pub.publish(msg)

    def target_pose_callback(self, msg: PoseStamped) -> None:
        now = self.get_clock().now()
        if msg.header.stamp.sec != 0 or msg.header.stamp.nanosec != 0:
            age = (now.nanoseconds - Time.from_msg(msg.header.stamp).nanoseconds) / 1e9
            if age > self.target_max_age_s:
                self.get_logger().debug(f"skip stale target pose age={age:.3f}s")
                self.publish_status(stage="stale_target_pose", target_age_s=f"{age:.3f}")
                return

        with self.state_lock:
            state = AircraftState(**self.state.__dict__)

        if not state.is_ready(self.state_max_age_s):
            self.get_logger().debug("skip target pose because aircraft state is not ready")
            wall_now = time.time()
            self.publish_status(
                stage="state_not_ready",
                position_age_s=f"{wall_now - state.last_position_time:.3f}",
                attitude_age_s=f"{wall_now - state.last_attitude_time:.3f}",
                speed_age_s=f"{wall_now - state.last_speed_time:.3f}",
            )
            return

        p_camera = np.array(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
            dtype=np.float64,
        )
        p_body = self.r_body_camera @ p_camera + self.t_body_camera
        r_ned_body = body_to_ned_matrix(state.roll_rad, state.pitch_rad, state.yaw_rad)
        p_ned = r_ned_body @ p_body

        target_alt = self.resolve_target_altitude(state, p_ned)
        raw_lat, raw_lon, raw_alt = ned_offset_to_geodetic(
            state.lat_deg,
            state.lon_deg,
            state.alt_msl_m,
            float(p_ned[0]),
            float(p_ned[1]),
            state.alt_msl_m - target_alt,
        )
        output_p_ned = self.simulate_target_offset(state, p_ned.copy()) if self.use_sim_target_geo else p_ned
        lat, lon, alt = ned_offset_to_geodetic(
            state.lat_deg,
            state.lon_deg,
            state.alt_msl_m,
            float(output_p_ned[0]),
            float(output_p_ned[1]),
            state.alt_msl_m - target_alt,
        )

        local_msg = PointStamped()
        local_msg.header.stamp = self.get_clock().now().to_msg()
        local_msg.header.frame_id = "aircraft_ned"
        local_msg.point.x = float(output_p_ned[0])
        local_msg.point.y = float(output_p_ned[1])
        local_msg.point.z = float(output_p_ned[2])
        self.target_local_pub.publish(local_msg)

        if self.publish_raw_target_geo:
            raw_geo_msg = NavSatFix()
            raw_geo_msg.header = local_msg.header
            raw_geo_msg.header.frame_id = "earth"
            raw_geo_msg.status.status = NavSatStatus.STATUS_FIX
            raw_geo_msg.status.service = NavSatStatus.SERVICE_GPS
            raw_geo_msg.latitude = raw_lat
            raw_geo_msg.longitude = raw_lon
            raw_geo_msg.altitude = raw_alt
            self.target_geo_raw_pub.publish(raw_geo_msg)

        geo_msg = NavSatFix()
        geo_msg.header = local_msg.header
        geo_msg.header.frame_id = "earth"
        geo_msg.status.status = NavSatStatus.STATUS_FIX
        geo_msg.status.service = NavSatStatus.SERVICE_GPS
        geo_msg.latitude = lat
        geo_msg.longitude = lon
        geo_msg.altitude = alt
        self.target_geo_pub.publish(geo_msg)
        self.publish_status(
            stage="published",
            raw_north_m=f"{float(p_ned[0]):.3f}",
            raw_east_m=f"{float(p_ned[1]):.3f}",
            out_north_m=f"{float(output_p_ned[0]):.3f}",
            out_east_m=f"{float(output_p_ned[1]):.3f}",
            sim=int(self.use_sim_target_geo),
            sim_mode=self.sim_target_mode,
            sim_distance_m=f"{self.sim_target_distance_m:.3f}",
            sim_active_distance_m=f"{math.hypot(float(output_p_ned[0]), float(output_p_ned[1])):.3f}",
            sim_distance_end_m=f"{self.sim_target_distance_end_m:.3f}",
            sim_closing_speed_mps=f"{self.sim_target_closing_speed_mps:.3f}",
            sim_reset_delay_s=f"{self.sim_target_reset_delay_s:.3f}",
            sim_bearing_offset_deg=f"{self.sim_target_bearing_offset_deg:.3f}",
            lat=f"{lat:.7f}",
            lon=f"{lon:.7f}",
        )

    def publish_status(self, **fields) -> None:
        msg = String()
        msg.data = " ".join(f"{key}={value}" for key, value in fields.items())
        self.status_pub.publish(msg)

    def simulate_target_offset(self, state: AircraftState, p_ned: np.ndarray) -> np.ndarray:
        target_distance = self.resolve_sim_target_distance(state)
        current_distance = math.hypot(float(p_ned[0]), float(p_ned[1]))
        min_direction_distance = max(0.0, self.sim_min_direction_distance_m)

        if current_distance >= min_direction_distance and current_distance > 1e-6:
            north_unit = float(p_ned[0]) / current_distance
            east_unit = float(p_ned[1]) / current_distance
        else:
            heading_rad = math.radians(state.heading_deg) if math.isfinite(state.heading_deg) else state.yaw_rad
            north_unit = math.cos(heading_rad)
            east_unit = math.sin(heading_rad)

        bearing_offset_rad = math.radians(self.sim_target_bearing_offset_deg)
        if abs(bearing_offset_rad) > 1e-9:
            cos_offset = math.cos(bearing_offset_rad)
            sin_offset = math.sin(bearing_offset_rad)
            north_unit, east_unit = (
                north_unit * cos_offset - east_unit * sin_offset,
                north_unit * sin_offset + east_unit * cos_offset,
            )

        p_ned[0] = north_unit * target_distance
        p_ned[1] = east_unit * target_distance
        return p_ned

    def resolve_sim_target_distance(self, state: AircraftState) -> float:
        start_distance = max(0.0, self.sim_target_distance_m)
        end_distance = max(0.0, self.sim_target_distance_end_m)

        if self.sim_target_mode == "fixed_distance":
            self.sim_start_time = None
            self.sim_end_reached_time = None
            return start_distance

        if self.sim_target_mode == "closing_distance":
            now = time.time()
            if self.sim_start_time is None:
                self.sim_start_time = now
                self.sim_end_reached_time = None

            closing_speed = self.sim_target_closing_speed_mps
            if closing_speed <= 0.0:
                closing_speed = max(0.0, state.groundspeed_mps)

            elapsed = max(0.0, now - self.sim_start_time)
            distance = start_distance - closing_speed * elapsed
            if distance <= end_distance:
                if self.sim_end_reached_time is None:
                    self.sim_end_reached_time = now
                    return end_distance

                reset_delay = max(0.0, self.sim_target_reset_delay_s)
                if now - self.sim_end_reached_time >= reset_delay:
                    self.sim_start_time = now
                    self.sim_end_reached_time = None
                    return start_distance

                return end_distance

            self.sim_end_reached_time = None
            return distance

        if not self.warned_unsupported_sim_mode:
            self.get_logger().warning(f"unsupported sim_target_mode={self.sim_target_mode}, using fixed_distance")
            self.warned_unsupported_sim_mode = True
        return start_distance

    def resolve_target_altitude(self, state: AircraftState, p_ned: np.ndarray) -> float:
        if self.target_altitude_mode == "ground_msl_offset":
            return self.target_altitude_offset_m
        if self.target_altitude_mode == "pnp":
            return state.alt_msl_m - float(p_ned[2])
        return state.alt_msl_m + self.target_altitude_offset_m


def main() -> None:
    rclpy.init()
    node = TargetGeoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
