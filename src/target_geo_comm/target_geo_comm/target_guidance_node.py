from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, String

try:
    from pymavlink import mavutil
except ImportError:  # pragma: no cover
    mavutil = None


EARTH_RADIUS_M = 6378137.0


@dataclass
class VehicleStatus:
    connected: bool = False
    armed: bool = False
    mode: str = "UNKNOWN"
    custom_mode: int = -1
    mission_seq: int = -1
    lat_deg: float = 0.0
    lon_deg: float = 0.0
    rel_alt_m: float = 0.0
    last_heartbeat_time: float = 0.0
    last_mission_time: float = 0.0
    last_position_time: float = 0.0


@dataclass
class LockedTarget:
    lat_deg: float
    lon_deg: float
    alt_m: float
    stamp_ns: int


def geodetic_to_local_ned(
    origin_lat_deg: float,
    origin_lon_deg: float,
    lat_deg: float,
    lon_deg: float,
) -> tuple[float, float]:
    lat0_rad = math.radians(origin_lat_deg)
    north_m = math.radians(lat_deg - origin_lat_deg) * EARTH_RADIUS_M
    east_m = math.radians(lon_deg - origin_lon_deg) * EARTH_RADIUS_M * max(math.cos(lat0_rad), 1e-9)
    return north_m, east_m


def destination_point(lat_deg: float, lon_deg: float, bearing_deg: float, distance_m: float) -> tuple[float, float]:
    lat1 = math.radians(lat_deg)
    lon1 = math.radians(lon_deg)
    bearing = math.radians(bearing_deg)
    angular_distance = distance_m / EARTH_RADIUS_M

    lat2 = math.asin(
        math.sin(lat1) * math.cos(angular_distance)
        + math.cos(lat1) * math.sin(angular_distance) * math.cos(bearing)
    )
    lon2 = lon1 + math.atan2(
        math.sin(bearing) * math.sin(angular_distance) * math.cos(lat1),
        math.cos(angular_distance) - math.sin(lat1) * math.sin(lat2),
    )
    return math.degrees(lat2), math.degrees(lon2)


def bearing_between(lat1_deg: float, lon1_deg: float, lat2_deg: float, lon2_deg: float) -> float:
    lat1 = math.radians(lat1_deg)
    lat2 = math.radians(lat2_deg)
    d_lon = math.radians(lon2_deg - lon1_deg)
    y = math.sin(d_lon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(d_lon)
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0


class TargetGuidanceNode(Node):
    def __init__(self) -> None:
        super().__init__("target_guidance_node")

        self.declare_parameter("mavlink_connection", "udp:127.0.0.1:14551")
        self.declare_parameter("target_geo_topic", "/target_geo")
        self.declare_parameter("target_max_age_s", 1.0)
        self.declare_parameter("require_auto", True)
        self.declare_parameter("require_armed", False)
        self.declare_parameter("target_wp_seq", 4)
        self.declare_parameter("exit_wp_seq", 5)
        self.declare_parameter("retry_wp_seq", 2)
        self.declare_parameter("fallback_release_wp_seq", 3)
        self.declare_parameter("max_visual_attempts", 2)
        self.declare_parameter("target_confirm_count", 5)
        self.declare_parameter("target_confirm_radius_m", 20.0)
        self.declare_parameter("min_update_interval_s", 2.0)
        self.declare_parameter("min_update_delta_m", 5.0)
        self.declare_parameter("lock_distance_to_target_m", 80.0)
        self.declare_parameter("pass_target_margin_m", 20.0)
        self.declare_parameter("exit_distance_m", 160.0)
        self.declare_parameter("attack_heading_deg", 271.8)
        self.declare_parameter("mission_altitude_m", 100.0)
        self.declare_parameter("accept_radius_m", 15.0)
        self.declare_parameter("fallback_release_topic", "/bomb_release/force_release")
        self.declare_parameter("bomb_released_topic", "/bomb_release/released")
        self.declare_parameter("debug_log", True)

        self.mavlink_connection = self.get_parameter("mavlink_connection").get_parameter_value().string_value
        target_geo_topic = self.get_parameter("target_geo_topic").get_parameter_value().string_value
        fallback_release_topic = self.get_parameter("fallback_release_topic").get_parameter_value().string_value
        bomb_released_topic = self.get_parameter("bomb_released_topic").get_parameter_value().string_value

        self.target_max_age_s = float(self.get_parameter("target_max_age_s").value)
        self.require_auto = bool(self.get_parameter("require_auto").value)
        self.require_armed = bool(self.get_parameter("require_armed").value)
        self.target_wp_seq = int(self.get_parameter("target_wp_seq").value)
        self.exit_wp_seq = int(self.get_parameter("exit_wp_seq").value)
        self.retry_wp_seq = int(self.get_parameter("retry_wp_seq").value)
        self.fallback_release_wp_seq = int(self.get_parameter("fallback_release_wp_seq").value)
        self.max_visual_attempts = max(1, int(self.get_parameter("max_visual_attempts").value))
        self.target_confirm_count = max(1, int(self.get_parameter("target_confirm_count").value))
        self.target_confirm_radius_m = float(self.get_parameter("target_confirm_radius_m").value)
        self.min_update_interval_s = float(self.get_parameter("min_update_interval_s").value)
        self.min_update_delta_m = float(self.get_parameter("min_update_delta_m").value)
        self.lock_distance_to_target_m = float(self.get_parameter("lock_distance_to_target_m").value)
        self.pass_target_margin_m = float(self.get_parameter("pass_target_margin_m").value)
        self.exit_distance_m = float(self.get_parameter("exit_distance_m").value)
        self.attack_heading_deg = float(self.get_parameter("attack_heading_deg").value)
        self.mission_altitude_m = float(self.get_parameter("mission_altitude_m").value)
        self.accept_radius_m = float(self.get_parameter("accept_radius_m").value)
        self.debug_log = bool(self.get_parameter("debug_log").value)

        self.status = VehicleStatus()
        self.status_lock = threading.Lock()
        self.mav_lock = threading.Lock()
        self.mav = None
        self.stop_event = threading.Event()
        self.mav_thread = threading.Thread(target=self.mavlink_loop, daemon=True)

        self.latest_target: Optional[NavSatFix] = None
        self.candidate_target: Optional[LockedTarget] = None
        self.confirm_count = 0
        self.locked_target: Optional[LockedTarget] = None
        self.target_frozen = False
        self.last_uploaded_target: Optional[LockedTarget] = None
        self.last_update_wall_time = 0.0
        self.visual_attempts = 0
        self.retry_commanded_for_seq = -1
        self.retry_entry_seen = True
        self.fallback_release_sent = False
        self.bomb_released = False
        self.last_status = ""

        self.sub = self.create_subscription(NavSatFix, target_geo_topic, self.target_callback, 10)
        self.bomb_released_sub = self.create_subscription(Bool, bomb_released_topic, self.bomb_released_callback, 10)
        self.active_pub = self.create_publisher(Bool, "/target_guidance_active", 10)
        self.status_pub = self.create_publisher(String, "/target_guidance_status", 10)
        self.locked_target_pub = self.create_publisher(NavSatFix, "/target_geo_locked", 10)
        self.force_release_pub = self.create_publisher(Bool, fallback_release_topic, 10)
        self.timer = self.create_timer(0.2, self.guidance_timer)

        self.mav_thread.start()
        self.get_logger().info(
            "target_guidance_node started in AUTO mission mode "
            f"mavlink={self.mavlink_connection} target_geo_topic={target_geo_topic}"
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

    def mavlink_loop(self) -> None:
        if mavutil is None:
            self.get_logger().error("pymavlink is not installed in this environment")
            return

        while not self.stop_event.is_set():
            try:
                self.get_logger().info(f"connecting MAVLink mission link: {self.mavlink_connection}")
                self.mav = mavutil.mavlink_connection(self.mavlink_connection)
                heartbeat = self.mav.wait_heartbeat(timeout=10)
                if not heartbeat:
                    raise TimeoutError("MAVLink heartbeat timeout")
                self.get_logger().info(
                    f"MAVLink mission connected sys={self.mav.target_system} comp={self.mav.target_component}"
                )

                wanted = [
                    "HEARTBEAT",
                    "MISSION_CURRENT",
                    "GLOBAL_POSITION_INT",
                    "COMMAND_ACK",
                    "STATUSTEXT",
                ]
                while not self.stop_event.is_set():
                    with self.mav_lock:
                        msg = self.mav.recv_match(type=wanted, blocking=True, timeout=1)
                    if msg is None:
                        continue
                    self.handle_mavlink_msg(msg)
            except Exception as exc:
                if self.stop_event.is_set():
                    break
                with self.status_lock:
                    self.status.connected = False
                self.get_logger().warning(f"MAVLink mission loop error: {exc}; reconnecting in 2s")
                time.sleep(2.0)

    def handle_mavlink_msg(self, msg) -> None:
        msg_type = msg.get_type()
        now = time.time()
        with self.status_lock:
            if msg_type == "HEARTBEAT":
                self.status.connected = True
                self.status.mode = mavutil.mode_string_v10(msg) if mavutil is not None else "UNKNOWN"
                self.status.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self.status.custom_mode = int(msg.custom_mode)
                self.status.last_heartbeat_time = now
            elif msg_type == "MISSION_CURRENT":
                self.status.mission_seq = int(msg.seq)
                self.status.last_mission_time = now
            elif msg_type == "GLOBAL_POSITION_INT":
                self.status.lat_deg = msg.lat / 1e7
                self.status.lon_deg = msg.lon / 1e7
                self.status.rel_alt_m = msg.relative_alt / 1000.0
                self.status.last_position_time = now

        if msg_type == "COMMAND_ACK" and self.debug_log:
            self.get_logger().info(f"COMMAND_ACK command={msg.command} result={msg.result}")
        elif msg_type == "STATUSTEXT" and self.debug_log:
            self.get_logger().info(f"STATUSTEXT {getattr(msg, 'text', '')}")

    def target_callback(self, msg: NavSatFix) -> None:
        if not self.valid_target(msg):
            return
        self.latest_target = msg

    def bomb_released_callback(self, msg: Bool) -> None:
        if msg.data:
            self.bomb_released = True

    def guidance_timer(self) -> None:
        active = Bool()
        active.data = False

        with self.status_lock:
            status = VehicleStatus(**self.status.__dict__)

        if not self.status_ready(status):
            self.publish_status(active, "waiting vehicle status")
            return

        if self.require_armed and not status.armed:
            self.publish_status(active, "vehicle not armed")
            return

        if self.require_auto and status.mode != "AUTO":
            self.publish_status(active, f"waiting AUTO current={status.mode}")
            return

        self.update_retry_entry_seen(status)

        if self.locked_target is not None and self.target_locked_by_distance(status):
            self.target_frozen = True

        self.update_target_confirmation()
        self.publish_locked_target()

        if self.locked_target is not None and not self.target_locked_by_distance(status):
            if self.maybe_update_mission_target(status):
                active.data = True

        self.handle_retry_or_fallback(status)

        if self.locked_target is None:
            self.publish_status(
                active,
                f"searching seq={status.mission_seq} confirms={self.confirm_count} attempts={self.visual_attempts}",
            )
            return

        distance = self.distance_to_locked_target(status)
        self.publish_status(
            active,
            f"locked seq={status.mission_seq} dist={distance:.1f}m "
            f"released={int(self.bomb_released)} attempts={self.visual_attempts}",
        )

    def status_ready(self, status: VehicleStatus) -> bool:
        now = time.time()
        return (
            status.connected
            and now - status.last_heartbeat_time <= 3.0
            and now - status.last_position_time <= 3.0
            and status.mission_seq >= 0
        )

    def update_target_confirmation(self) -> None:
        if self.target_frozen:
            return
        target = self.latest_target
        if target is None:
            return
        if target.header.stamp.sec != 0 or target.header.stamp.nanosec != 0:
            age = (self.get_clock().now().nanoseconds - Time.from_msg(target.header.stamp).nanoseconds) / 1e9
            if age > self.target_max_age_s:
                return

        incoming = LockedTarget(
            lat_deg=float(target.latitude),
            lon_deg=float(target.longitude),
            alt_m=float(target.altitude) if math.isfinite(target.altitude) else self.mission_altitude_m,
            stamp_ns=self.get_clock().now().nanoseconds,
        )

        if self.candidate_target is None:
            self.candidate_target = incoming
            self.confirm_count = 1
            return

        delta_m = self.distance_between(self.candidate_target, incoming)
        if delta_m <= self.target_confirm_radius_m:
            alpha = 1.0 / float(self.confirm_count + 1)
            self.candidate_target = LockedTarget(
                lat_deg=(1.0 - alpha) * self.candidate_target.lat_deg + alpha * incoming.lat_deg,
                lon_deg=(1.0 - alpha) * self.candidate_target.lon_deg + alpha * incoming.lon_deg,
                alt_m=(1.0 - alpha) * self.candidate_target.alt_m + alpha * incoming.alt_m,
                stamp_ns=incoming.stamp_ns,
            )
            self.confirm_count += 1
        else:
            self.candidate_target = incoming
            self.confirm_count = 1

        if self.confirm_count >= self.target_confirm_count:
            self.locked_target = self.candidate_target

    def maybe_update_mission_target(self, status: VehicleStatus) -> bool:
        if self.locked_target is None or self.mav is None or mavutil is None:
            return False
        now = time.time()
        if now - self.last_update_wall_time < self.min_update_interval_s:
            return False
        if self.last_uploaded_target is not None:
            delta_m = self.distance_between(self.last_uploaded_target, self.locked_target)
            if delta_m < self.min_update_delta_m:
                return False

        heading = self.resolve_attack_heading(status)
        exit_lat, exit_lon = destination_point(
            self.locked_target.lat_deg,
            self.locked_target.lon_deg,
            heading,
            self.exit_distance_m,
        )

        target_alt = self.mission_altitude_m
        try:
            self.update_waypoint_int(
                self.target_wp_seq,
                self.locked_target.lat_deg,
                self.locked_target.lon_deg,
                target_alt,
                self.accept_radius_m,
            )
            self.update_waypoint_int(
                self.exit_wp_seq,
                exit_lat,
                exit_lon,
                target_alt,
                max(self.accept_radius_m, 30.0),
            )
        except Exception as exc:
            self.get_logger().warning(f"mission update failed: {exc}")
            return False

        self.last_uploaded_target = self.locked_target
        self.last_update_wall_time = now
        self.get_logger().info(
            f"updated mission wp{self.target_wp_seq}/wp{self.exit_wp_seq} "
            f"target=({self.locked_target.lat_deg:.7f},{self.locked_target.lon_deg:.7f}) "
            f"exit=({exit_lat:.7f},{exit_lon:.7f}) heading={heading:.1f}"
        )
        return True

    def update_waypoint_int(
        self,
        seq: int,
        lat_deg: float,
        lon_deg: float,
        alt_m: float,
        accept_radius_m: float,
    ) -> None:
        if self.mav is None or mavutil is None:
            raise RuntimeError("MAVLink unavailable")

        with self.mav_lock:
            self.mav.mav.mission_write_partial_list_send(
                self.mav.target_system,
                self.mav.target_component,
                seq,
                seq,
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
            )

            req = self.mav.recv_match(type=["MISSION_REQUEST_INT", "MISSION_REQUEST"], blocking=True, timeout=2)
            if req is None:
                raise TimeoutError(f"vehicle did not request mission item {seq}")
            if int(req.seq) != seq:
                raise RuntimeError(f"vehicle requested mission item {req.seq}, expected {seq}")

            self.mav.mav.mission_item_int_send(
                self.mav.target_system,
                self.mav.target_component,
                seq,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0,
                1,
                0.0,
                float(accept_radius_m),
                0.0,
                0.0,
                int(round(lat_deg * 1e7)),
                int(round(lon_deg * 1e7)),
                float(alt_m),
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
            )

            ack = self.mav.recv_match(type="MISSION_ACK", blocking=True, timeout=2)
        if ack is None:
            raise TimeoutError(f"mission item {seq} update did not ACK")
        if int(ack.type) != mavutil.mavlink.MAV_MISSION_ACCEPTED:
            raise RuntimeError(f"mission item {seq} update rejected ack={ack.type}")

    def handle_retry_or_fallback(self, status: VehicleStatus) -> None:
        if self.bomb_released:
            return

        if self.locked_target is not None and self.has_passed_locked_target(status):
            self.command_retry_once(status, "passed locked target without release")
            return

        if (
            self.locked_target is None
            and self.visual_attempts >= self.max_visual_attempts - 1
            and self.retry_entry_seen
            and status.mission_seq > self.fallback_release_wp_seq
            and not self.fallback_release_sent
        ):
            self.send_force_release()
            return

        if (
            self.locked_target is None
            and status.mission_seq > self.target_wp_seq
            and self.visual_attempts < self.max_visual_attempts - 1
        ):
            self.command_retry_once(status, "passed target wp without visual lock")

    def command_retry_once(self, status: VehicleStatus, reason: str) -> None:
        if self.retry_commanded_for_seq == status.mission_seq:
            return
        self.visual_attempts += 1
        self.retry_commanded_for_seq = status.mission_seq
        self.retry_entry_seen = False
        self.candidate_target = None
        self.confirm_count = 0
        self.locked_target = None
        self.target_frozen = False
        self.last_uploaded_target = None
        self.last_update_wall_time = 0.0
        self.set_mission_current(self.retry_wp_seq)
        self.get_logger().warning(
            f"{reason}; retry attempt={self.visual_attempts}/{self.max_visual_attempts} wp={self.retry_wp_seq}"
        )

    def update_retry_entry_seen(self, status: VehicleStatus) -> None:
        if self.retry_entry_seen or self.visual_attempts <= 0:
            return
        if status.mission_seq <= self.retry_wp_seq:
            self.retry_entry_seen = True
            self.retry_commanded_for_seq = -1
            self.get_logger().info(f"retry entry wp{self.retry_wp_seq} reached; fallback is now armed")

    def send_force_release(self) -> None:
        msg = Bool()
        msg.data = True
        self.force_release_pub.publish(msg)
        self.fallback_release_sent = True
        self.bomb_released = True
        self.get_logger().warning("fallback force release requested")

    def set_mission_current(self, seq: int) -> None:
        if self.mav is None or mavutil is None:
            return
        with self.mav_lock:
            self.mav.mav.command_long_send(
                self.mav.target_system,
                self.mav.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MISSION_CURRENT,
                0,
                float(seq),
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            )

    def publish_locked_target(self) -> None:
        if self.locked_target is None:
            return
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "earth"
        msg.latitude = self.locked_target.lat_deg
        msg.longitude = self.locked_target.lon_deg
        msg.altitude = self.locked_target.alt_m
        self.locked_target_pub.publish(msg)

    def target_locked_by_distance(self, status: VehicleStatus) -> bool:
        if self.lock_distance_to_target_m <= 0.0:
            return False
        return self.distance_to_locked_target(status) <= self.lock_distance_to_target_m

    def distance_to_locked_target(self, status: VehicleStatus) -> float:
        if self.locked_target is None:
            return float("inf")
        north_m, east_m = geodetic_to_local_ned(
            self.locked_target.lat_deg,
            self.locked_target.lon_deg,
            status.lat_deg,
            status.lon_deg,
        )
        return math.hypot(north_m, east_m)

    def has_passed_locked_target(self, status: VehicleStatus) -> bool:
        if self.locked_target is None:
            return False
        heading = math.radians(self.resolve_attack_heading(status))
        north_m, east_m = geodetic_to_local_ned(
            self.locked_target.lat_deg,
            self.locked_target.lon_deg,
            status.lat_deg,
            status.lon_deg,
        )
        along_m = north_m * math.cos(heading) + east_m * math.sin(heading)
        return along_m > self.pass_target_margin_m or status.mission_seq > self.target_wp_seq

    def resolve_attack_heading(self, status: VehicleStatus) -> float:
        if 0.0 <= self.attack_heading_deg < 360.0:
            return self.attack_heading_deg
        if self.locked_target is not None:
            return bearing_between(status.lat_deg, status.lon_deg, self.locked_target.lat_deg, self.locked_target.lon_deg)
        return 0.0

    def publish_status(self, active: Bool, text: str) -> None:
        self.active_pub.publish(active)
        if text == self.last_status and not self.debug_log:
            return
        self.last_status = text
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    @staticmethod
    def valid_target(msg: NavSatFix) -> bool:
        return (
            math.isfinite(msg.latitude)
            and math.isfinite(msg.longitude)
            and -90.0 <= msg.latitude <= 90.0
            and -180.0 <= msg.longitude <= 180.0
        )

    @staticmethod
    def distance_between(a: LockedTarget, b: LockedTarget) -> float:
        north_m, east_m = geodetic_to_local_ned(a.lat_deg, a.lon_deg, b.lat_deg, b.lon_deg)
        return math.hypot(north_m, east_m)


def main() -> None:
    rclpy.init()
    node = TargetGuidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
