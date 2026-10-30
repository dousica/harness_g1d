import os


def require_environment():
    domain = os.environ.get("ROS_DOMAIN_ID")
    if domain != "42":
        raise RuntimeError(f"ROS_DOMAIN_ID must be 42; got {domain!r}")
    for variable in (
        "AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH", "COLCON_PREFIX_PATH",
        "PYTHONPATH", "LD_LIBRARY_PATH"
    ):
        for entry in os.environ.get(variable, "").split(os.pathsep):
            if any(marker in entry for marker in (
                "g1d_ros2_ws", "g1d_ros2_yqqh", "unitree_ros_official"
            )):
                raise RuntimeError(f"{variable} contains forbidden overlay path: {entry}")
