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


PLANE_MODE_GUIDED = 15


@dataclass
class VehicleStatus:
    connected: bool = False
    armed: bool = False
    mode: str = "UNKNOWN"
    custom_mode: int = -1
    last_heartbeat_time: float = 0.0


class TargetGuidanceNode(Node):
    def __init__(self) -> None:
        super().__init__("target_guidance_node")

        self.declare_parameter("mavlink_connection", "udp:127.0.0.1:14551")
        self.declare_parameter("target_geo_topic", "/target_geo")
        self.declare_parameter("send_rate_hz", 10.0)
        self.declare_parameter("target_max_age_s", 1.0)
        self.declare_parameter("require_guided", True)
        self.declare_parameter("require_armed", False)
        self.declare_parameter("allow_mode_change", False)
        self.declare_parameter("target_altitude_mode", "relative")
        self.declare_parameter("guided_altitude_m", 80.0)
        self.declare_parameter("loiter_radius_m", 60.0)
        self.declare_parameter("debug_log", True)

        self.mavlink_connection = self.get_parameter("mavlink_connection").get_parameter_value().string_value
        target_geo_topic = self.get_parameter("target_geo_topic").get_parameter_value().string_value
        self.send_rate_hz = float(self.get_parameter("send_rate_hz").value)
        self.target_max_age_s = float(self.get_parameter("target_max_age_s").value)
        self.require_guided = bool(self.get_parameter("require_guided").value)
        self.require_armed = bool(self.get_parameter("require_armed").value)
        self.allow_mode_change = bool(self.get_parameter("allow_mode_change").value)
        self.target_altitude_mode = self.get_parameter("target_altitude_mode").get_parameter_value().string_value
        self.guided_altitude_m = float(self.get_parameter("guided_altitude_m").value)
        self.loiter_radius_m = float(self.get_parameter("loiter_radius_m").value)
        self.debug_log = bool(self.get_parameter("debug_log").value)

        self.status = VehicleStatus()
        self.status_lock = threading.Lock()
        self.mav = None
        self.stop_event = threading.Event()
        self.mav_thread = threading.Thread(target=self.mavlink_loop, daemon=True)

        self.latest_target: Optional[NavSatFix] = None
        self.last_send_wall_time = 0.0

        self.sub = self.create_subscription(NavSatFix, target_geo_topic, self.target_callback, 10)
        self.enabled_pub = self.create_publisher(Bool, "/target_guidance_active", 10)
        self.status_pub = self.create_publisher(String, "/target_guidance_status", 10)
        self.timer = self.create_timer(0.2, self.guidance_timer)

        self.mav_thread.start()
        self.get_logger().info(
            f"target_guidance_node started, mavlink={self.mavlink_connection}, target_geo_topic={target_geo_topic}"
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
                self.get_logger().info(f"connecting MAVLink guidance link: {self.mavlink_connection}")
                self.mav = mavutil.mavlink_connection(self.mavlink_connection)
                heartbeat = self.mav.wait_heartbeat(timeout=10)
                if not heartbeat:
                    raise TimeoutError("MAVLink heartbeat timeout")
                self.get_logger().info(
                    f"MAVLink guidance connected sys={self.mav.target_system} comp={self.mav.target_component}"
                )

                while not self.stop_event.is_set():
                    msg = self.mav.recv_match(type=["HEARTBEAT", "COMMAND_ACK", "STATUSTEXT"], blocking=True, timeout=1)
                    if msg is None:
                        continue
                    self.handle_mavlink_msg(msg)
            except Exception as exc:
                with self.status_lock:
                    self.status.connected = False
                self.get_logger().warning(f"MAVLink guidance loop error: {exc}; reconnecting in 2s")
                time.sleep(2.0)

    def handle_mavlink_msg(self, msg) -> None:
        msg_type = msg.get_type()
        if msg_type == "HEARTBEAT":
            mode = mavutil.mode_string_v10(msg) if mavutil is not None else "UNKNOWN"
            armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            with self.status_lock:
                self.status.connected = True
                self.status.armed = armed
                self.status.mode = mode
                self.status.custom_mode = int(msg.custom_mode)
                self.status.last_heartbeat_time = time.time()
        elif msg_type == "COMMAND_ACK" and self.debug_log:
            self.get_logger().info(f"COMMAND_ACK command={msg.command} result={msg.result}")
        elif msg_type == "STATUSTEXT" and self.debug_log:
            text = getattr(msg, "text", "")
            self.get_logger().info(f"STATUSTEXT {text}")

    def target_callback(self, msg: NavSatFix) -> None:
        if not self.valid_target(msg):
            return
        self.latest_target = msg

    def guidance_timer(self) -> None:
        active = Bool()
        active.data = False
        target = self.latest_target
        status_msg = String()

        if target is None:
            status_msg.data = "waiting target_geo"
            self.publish_status(active, status_msg)
            return

        if target.header.stamp.sec != 0 or target.header.stamp.nanosec != 0:
            age = (self.get_clock().now().nanoseconds - Time.from_msg(target.header.stamp).nanoseconds) / 1e9
            if age > self.target_max_age_s:
                status_msg.data = f"target stale age={age:.2f}s"
                self.publish_status(active, status_msg)
                return

        with self.status_lock:
            status = VehicleStatus(**self.status.__dict__)

        if not status.connected or time.time() - status.last_heartbeat_time > 3.0:
            status_msg.data = "MAVLink not connected"
            self.publish_status(active, status_msg)
            return

        if self.require_armed and not status.armed:
            status_msg.data = "vehicle not armed"
            self.publish_status(active, status_msg)
            return

        if self.require_guided and status.mode != "GUIDED":
            if self.allow_mode_change:
                self.set_guided_mode()
                status_msg.data = f"request GUIDED, current={status.mode}"
            else:
                status_msg.data = f"waiting GUIDED, current={status.mode}"
            self.publish_status(active, status_msg)
            return

        now = time.time()
        if self.send_rate_hz > 0.0 and now - self.last_send_wall_time < 1.0 / self.send_rate_hz:
            status_msg.data = "rate limited"
            self.publish_status(active, status_msg)
            return

        ok = self.send_reposition(target)
        if ok:
            active.data = True
            self.last_send_wall_time = now
            status_msg.data = f"sent target lat={target.latitude:.7f} lon={target.longitude:.7f} alt={target.altitude:.1f}"
        else:
            status_msg.data = "send target failed"
        self.publish_status(active, status_msg)

    def publish_status(self, active: Bool, status_msg: String) -> None:
        self.enabled_pub.publish(active)
        self.status_pub.publish(status_msg)

    def valid_target(self, msg: NavSatFix) -> bool:
        return math.isfinite(msg.latitude) and math.isfinite(msg.longitude) and -90.0 <= msg.latitude <= 90.0 and -180.0 <= msg.longitude <= 180.0

    def set_guided_mode(self) -> None:
        if self.mav is None or mavutil is None:
            return
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            PLANE_MODE_GUIDED,
            0,
            0,
            0,
            0,
            0,
        )

    def send_reposition(self, target: NavSatFix) -> bool:
        if self.mav is None or mavutil is None:
            return False

        alt = self.guided_altitude_m if self.target_altitude_mode == "relative" else float(target.altitude)
        frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT if self.target_altitude_mode == "relative" else mavutil.mavlink.MAV_FRAME_GLOBAL
        lat_int = int(round(float(target.latitude) * 1e7))
        lon_int = int(round(float(target.longitude) * 1e7))

        self.mav.mav.command_int_send(
            self.mav.target_system,
            self.mav.target_component,
            frame,
            mavutil.mavlink.MAV_CMD_DO_REPOSITION,
            0,
            0,
            -1.0,
            mavutil.mavlink.MAV_DO_REPOSITION_FLAGS_CHANGE_MODE,
            max(0.0, self.loiter_radius_m),
            0.0,
            lat_int,
            lon_int,
            float(alt),
        )
        return True


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
