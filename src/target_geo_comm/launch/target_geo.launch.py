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
                parameters=[
                    {
                        "mavlink_connection": LaunchConfiguration("mavlink_connection"),
                        "target_pose_topic": LaunchConfiguration("target_pose_topic"),
                        "mavlink_rate_hz": LaunchConfiguration("mavlink_rate_hz"),
                        "state_max_age_s": LaunchConfiguration("state_max_age_s"),
                        "target_max_age_s": LaunchConfiguration("target_max_age_s"),
                        "target_altitude_mode": LaunchConfiguration("target_altitude_mode"),
                        "target_altitude_offset_m": LaunchConfiguration("target_altitude_offset_m"),
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
