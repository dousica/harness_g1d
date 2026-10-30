import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    if os.environ.get("ROS_DOMAIN_ID") != "42":
        raise RuntimeError("ROS_DOMAIN_ID must be 42 before launching the G1-D description")
    for variable in (
        "AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH", "COLCON_PREFIX_PATH",
        "PYTHONPATH", "LD_LIBRARY_PATH"
    ):
        for entry in os.environ.get(variable, "").split(os.pathsep):
            if any(marker in entry for marker in (
                "g1d_ros2_ws", "g1d_ros2_yqqh", "unitree_ros_official"
            )):
                raise RuntimeError(f"{variable} contains forbidden overlay path: {entry}")

    package_dir = get_package_share_directory(
        "lzh_description"
    )

    urdf_path = os.path.join(
        package_dir,
        "urdf",
        "g1_d_ros2.urdf"
    )

    with open(urdf_path, "r") as f:
        robot_description = f.read()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description":
                    robot_description
            }
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
    )

    return LaunchDescription([
        robot_state_publisher,
        rviz
    ])
