import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    if os.environ.get("ROS_DOMAIN_ID") != "42":
        raise RuntimeError("ROS_DOMAIN_ID must be 42 before launching G1-D camera TF")
    for variable in (
        "AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH", "COLCON_PREFIX_PATH",
        "PYTHONPATH", "LD_LIBRARY_PATH"
    ):
        for entry in os.environ.get(variable, "").split(os.pathsep):
            if any(marker in entry for marker in (
                "g1d_ros2_ws", "g1d_ros2_yqqh", "unitree_ros_official"
            )):
                raise RuntimeError(f"{variable} contains forbidden overlay path: {entry}")
    home = EnvironmentVariable("HOME", default_value="/home/yushu")
    handeye = PathJoinSubstitution([home, "g1d_vision_data", "handeye"])
    stereo = PathJoinSubstitution([home, ".ros", "g1d_camera_calibration", "head_stereo.yaml"])
    return LaunchDescription([
        DeclareLaunchArgument("handeye_result", default_value=handeye),
        DeclareLaunchArgument("stereo_result", default_value=stereo),
        Node(
            package="lzh_bringup",
            executable="lzh_head_camera_tf_publisher",
            name="lzh_head_camera_tf_publisher",
            output="screen",
            parameters=[{
                "handeye_result": LaunchConfiguration("handeye_result"),
                "stereo_result": LaunchConfiguration("stereo_result"),
            }],
        ),
    ])
