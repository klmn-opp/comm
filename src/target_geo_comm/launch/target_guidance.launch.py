from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("mavlink_connection", default_value="udp:127.0.0.1:14551"),
            DeclareLaunchArgument("target_geo_topic", default_value="/target_geo"),
            DeclareLaunchArgument("send_rate_hz", default_value="10.0"),
            DeclareLaunchArgument("target_max_age_s", default_value="1.0"),
            DeclareLaunchArgument("require_guided", default_value="true"),
            DeclareLaunchArgument("require_armed", default_value="false"),
            DeclareLaunchArgument("allow_mode_change", default_value="false"),
            DeclareLaunchArgument("target_altitude_mode", default_value="relative"),
            DeclareLaunchArgument("guided_altitude_m", default_value="80.0"),
            DeclareLaunchArgument("loiter_radius_m", default_value="60.0"),
            DeclareLaunchArgument("debug_log", default_value="true"),
            Node(
                package="target_geo_comm",
                executable="target_guidance_node",
                name="target_guidance_node",
                output="screen",
                prefix="/home/klmn/cudac/comm/.venv/bin/python3",
                parameters=[
                    {
                        "mavlink_connection": LaunchConfiguration("mavlink_connection"),
                        "target_geo_topic": LaunchConfiguration("target_geo_topic"),
                        "send_rate_hz": LaunchConfiguration("send_rate_hz"),
                        "target_max_age_s": LaunchConfiguration("target_max_age_s"),
                        "require_guided": LaunchConfiguration("require_guided"),
                        "require_armed": LaunchConfiguration("require_armed"),
                        "allow_mode_change": LaunchConfiguration("allow_mode_change"),
                        "target_altitude_mode": LaunchConfiguration("target_altitude_mode"),
                        "guided_altitude_m": LaunchConfiguration("guided_altitude_m"),
                        "loiter_radius_m": LaunchConfiguration("loiter_radius_m"),
                        "debug_log": LaunchConfiguration("debug_log"),
                    }
                ],
            ),
        ]
    )
