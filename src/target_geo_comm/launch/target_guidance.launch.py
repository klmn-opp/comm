from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("mavlink_connection", default_value="udp:127.0.0.1:14551"),
            DeclareLaunchArgument("target_geo_topic", default_value="/target_geo"),
            DeclareLaunchArgument("target_max_age_s", default_value="1.0"),
            DeclareLaunchArgument("require_auto", default_value="true"),
            DeclareLaunchArgument("require_armed", default_value="false"),
            DeclareLaunchArgument("target_wp_seq", default_value="4"),
            DeclareLaunchArgument("exit_wp_seq", default_value="5"),
            DeclareLaunchArgument("retry_wp_seq", default_value="2"),
            DeclareLaunchArgument("fallback_release_wp_seq", default_value="3"),
            DeclareLaunchArgument("max_visual_attempts", default_value="2"),
            DeclareLaunchArgument("target_confirm_count", default_value="5"),
            DeclareLaunchArgument("target_confirm_radius_m", default_value="20.0"),
            DeclareLaunchArgument("min_update_interval_s", default_value="2.0"),
            DeclareLaunchArgument("min_update_delta_m", default_value="5.0"),
            DeclareLaunchArgument("lock_distance_to_target_m", default_value="80.0"),
            DeclareLaunchArgument("pass_target_margin_m", default_value="20.0"),
            DeclareLaunchArgument("exit_distance_m", default_value="160.0"),
            DeclareLaunchArgument("attack_heading_deg", default_value="271.8"),
            DeclareLaunchArgument("mission_altitude_m", default_value="100.0"),
            DeclareLaunchArgument("accept_radius_m", default_value="15.0"),
            DeclareLaunchArgument("fallback_release_topic", default_value="/bomb_release/force_release"),
            DeclareLaunchArgument("bomb_released_topic", default_value="/bomb_release/released"),
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
                        "target_max_age_s": LaunchConfiguration("target_max_age_s"),
                        "require_auto": LaunchConfiguration("require_auto"),
                        "require_armed": LaunchConfiguration("require_armed"),
                        "target_wp_seq": LaunchConfiguration("target_wp_seq"),
                        "exit_wp_seq": LaunchConfiguration("exit_wp_seq"),
                        "retry_wp_seq": LaunchConfiguration("retry_wp_seq"),
                        "fallback_release_wp_seq": LaunchConfiguration("fallback_release_wp_seq"),
                        "max_visual_attempts": LaunchConfiguration("max_visual_attempts"),
                        "target_confirm_count": LaunchConfiguration("target_confirm_count"),
                        "target_confirm_radius_m": LaunchConfiguration("target_confirm_radius_m"),
                        "min_update_interval_s": LaunchConfiguration("min_update_interval_s"),
                        "min_update_delta_m": LaunchConfiguration("min_update_delta_m"),
                        "lock_distance_to_target_m": LaunchConfiguration("lock_distance_to_target_m"),
                        "pass_target_margin_m": LaunchConfiguration("pass_target_margin_m"),
                        "exit_distance_m": LaunchConfiguration("exit_distance_m"),
                        "attack_heading_deg": LaunchConfiguration("attack_heading_deg"),
                        "mission_altitude_m": LaunchConfiguration("mission_altitude_m"),
                        "accept_radius_m": LaunchConfiguration("accept_radius_m"),
                        "fallback_release_topic": LaunchConfiguration("fallback_release_topic"),
                        "bomb_released_topic": LaunchConfiguration("bomb_released_topic"),
                        "debug_log": LaunchConfiguration("debug_log"),
                    }
                ],
            ),
        ]
    )
