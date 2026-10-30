import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from ros_environment_guard import require_environment

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_static_virtual_joint_tfs_launch


def generate_launch_description():
    require_environment()
    moveit_config = MoveItConfigsBuilder("G1_D", package_name="lzh_moveit_config").to_moveit_configs()
    return generate_static_virtual_joint_tfs_launch(moveit_config)
