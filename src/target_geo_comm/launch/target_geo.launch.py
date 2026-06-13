from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("mavlink_connection", default_value="udp:127.0.0.1:14550"),
            DeclareLaunchArgument("target_pose_topic", default_value="/target_pose_camera"),
            DeclareLaunchArgument("mavlink_rate_hz", default_value="20.0"),
            DeclareLaunchArgument("state_max_age_s", default_value="1.0"),
            DeclareLaunchArgument("target_max_age_s", default_value="0.5"),
            DeclareLaunchArgument("target_altitude_mode", default_value="pnp"),
            DeclareLaunchArgument("target_altitude_offset_m", default_value="0.0"),
            DeclareLaunchArgument("use_sim_target_geo", default_value="false"),
            DeclareLaunchArgument("sim_target_distance_m", default_value="80.0"),
            DeclareLaunchArgument("sim_min_direction_distance_m", default_value="3.0"),
            DeclareLaunchArgument("sim_target_bearing_offset_deg", default_value="0.0"),
            DeclareLaunchArgument("publish_raw_target_geo", default_value="true"),
            DeclareLaunchArgument("camera_roll_deg", default_value="0.0"),
            DeclareLaunchArgument("camera_pitch_deg", default_value="90.0"),
            DeclareLaunchArgument("camera_yaw_deg", default_value="0.0"),
            DeclareLaunchArgument("camera_x_m", default_value="0.0"),
            DeclareLaunchArgument("camera_y_m", default_value="0.0"),
            DeclareLaunchArgument("camera_z_m", default_value="0.0"),
            Node(
                package="target_geo_comm",
                executable="target_geo_node",
                name="target_geo_node",
                output="screen",
                prefix="/home/klmn/cudac/comm/.venv/bin/python3",
                parameters=[
                    {
                        "mavlink_connection": LaunchConfiguration("mavlink_connection"),
                        "target_pose_topic": LaunchConfiguration("target_pose_topic"),
                        "mavlink_rate_hz": LaunchConfiguration("mavlink_rate_hz"),
                        "state_max_age_s": LaunchConfiguration("state_max_age_s"),
                        "target_max_age_s": LaunchConfiguration("target_max_age_s"),
                        "target_altitude_mode": LaunchConfiguration("target_altitude_mode"),
                        "target_altitude_offset_m": LaunchConfiguration("target_altitude_offset_m"),
                        "use_sim_target_geo": LaunchConfiguration("use_sim_target_geo"),
                        "sim_target_distance_m": LaunchConfiguration("sim_target_distance_m"),
                        "sim_min_direction_distance_m": LaunchConfiguration("sim_min_direction_distance_m"),
                        "sim_target_bearing_offset_deg": LaunchConfiguration("sim_target_bearing_offset_deg"),
                        "publish_raw_target_geo": LaunchConfiguration("publish_raw_target_geo"),
                        "camera_roll_deg": LaunchConfiguration("camera_roll_deg"),
                        "camera_pitch_deg": LaunchConfiguration("camera_pitch_deg"),
                        "camera_yaw_deg": LaunchConfiguration("camera_yaw_deg"),
                        "camera_x_m": LaunchConfiguration("camera_x_m"),
                        "camera_y_m": LaunchConfiguration("camera_y_m"),
                        "camera_z_m": LaunchConfiguration("camera_z_m"),
                    }
                ],
            ),
        ]
    )
