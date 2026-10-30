import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory


def _require_runtime_environment():
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


def generate_launch_description():
    _require_runtime_environment()

    network_interface = LaunchConfiguration("network_interface")
    real_execution = LaunchConfiguration("real_execution")
    gripper_real_execution = LaunchConfiguration("gripper_real_execution")
    use_rviz = LaunchConfiguration("use_rviz")

    max_path_delta_rad = LaunchConfiguration("max_path_delta_rad")
    max_start_error_rad = LaunchConfiguration("max_start_error_rad")
    goal_tolerance_rad = LaunchConfiguration("goal_tolerance_rad")
    settle_duration_sec = LaunchConfiguration("settle_duration_sec")
    settle_timeout_sec = LaunchConfiguration("settle_timeout_sec")

    moveit_share = get_package_share_directory("lzh_moveit_config")

    rsp_launch = os.path.join(
        moveit_share,
        "launch",
        "rsp.launch.py",
    )

    static_tf_launch = os.path.join(
        moveit_share,
        "launch",
        "static_virtual_joint_tfs.launch.py",
    )

    head_camera_tf_launch = os.path.join(
        get_package_share_directory("lzh_bringup"),
        "launch",
        "head_camera_tf.launch.py",
    )

    move_group_launch = os.path.join(
        moveit_share,
        "launch",
        "move_group.launch.py",
    )

    rviz_launch = os.path.join(
        moveit_share,
        "launch",
        "moveit_rviz.launch.py",
    )

    # Make Unitree SDK2 runtime library visible to the launched bridge nodes.
    sdk2_lib = PathJoinSubstitution(
        [
            EnvironmentVariable("HOME"),
            "unitree_sdk2",
            "install",
            "lib",
        ]
    )

    ld_library_path = [
        sdk2_lib,
        ":",
        EnvironmentVariable(
            "LD_LIBRARY_PATH",
            default_value="",
        ),
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "network_interface",
                default_value="eno2",
            ),

            # SAFE DEFAULT:
            # false = dry-run, no real LowCmd
            # true  = real G1-D execution enabled
            DeclareLaunchArgument(
                "real_execution",
                default_value="false",
            ),
            
            # Dex1-1 gripper real execution
            DeclareLaunchArgument(
                "gripper_real_execution",
                default_value="false",
            ),


            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
            ),

            # 15 degrees
            DeclareLaunchArgument(
                "max_path_delta_rad",
                default_value="0.261799",
            ),

            DeclareLaunchArgument(
                "max_start_error_rad",
                default_value="0.03",
            ),

            # Keep current strict value for now.
            DeclareLaunchArgument(
                "goal_tolerance_rad",
                default_value="0.03",
            ),

            DeclareLaunchArgument(
                "settle_duration_sec",
                default_value="0.20",
            ),

            DeclareLaunchArgument(
                "settle_timeout_sec",
                default_value="2.0",
            ),

            SetEnvironmentVariable(
                "LD_LIBRARY_PATH",
                ld_library_path,
            ),

            # 1. Real G1-D LowState -> /joint_states
            Node(
                package="lzh_sdk_bridge",
                executable="lowstate_bridge_node",
                name="lzh_lowstate_bridge",
                output="screen",
                parameters=[
                    {
                        "network_interface": network_interface,
                    }
                ],
            ),

            # 2. robot_state_publisher
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(rsp_launch),
            ),

            # 3. world -> AGV_link fixed virtual joint
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(static_tf_launch),
            ),

            # 3b. Fixed head-camera extrinsics. The right-camera transform is
            # derived from the calibrated left->right stereo transform.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(head_camera_tf_launch),
            ),

            # 4. MoveIt move_group
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(move_group_launch),
            ),

            # 5. RViz
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(rviz_launch),
                condition=IfCondition(use_rviz),
            ),

            # 6. MoveIt trajectory -> SDK2 bridge
            Node(
                package="lzh_sdk_bridge",
                executable="lzh_trajectory_action_server",
                name="lzh_trajectory_action_server",
                output="screen",
                parameters=[
                    {
                        # real_execution=false -> dry_run=true
                        "dry_run": ParameterValue(
                            PythonExpression(
                                ["'", real_execution, "'.lower() != 'true'"]
                            ),
                            value_type=bool,
                        ),

                        "enable_real_execution": ParameterValue(
                            real_execution,
                            value_type=bool,
                        ),

                        "enable_gripper_real_execution": ParameterValue(
                            gripper_real_execution,
                            value_type=bool,
                        ),
                        # 双臂联合实机控制许可
                        "enable_both_arms_real_execution": ParameterValue(
                            real_execution,
                            value_type=bool,
                        ),

            # 四路并行控制许可：只有机械臂和夹爪均启用实机控制时才开启
                        "enable_quad_real_execution": ParameterValue(
                            PythonExpression([
                                "'", real_execution, "'.lower() == 'true' and '",
                                gripper_real_execution, "'.lower() == 'true'",
                            ]),
                            value_type=bool,
                        ),

                        "network_interface": network_interface,

                        "max_path_delta_rad": ParameterValue(
                            max_path_delta_rad,
                            value_type=float,
                        ),

                        "max_start_error_rad": ParameterValue(
                            max_start_error_rad,
                            value_type=float,
                        ),

                        "goal_tolerance_rad": ParameterValue(
                            goal_tolerance_rad,
                            value_type=float,
                        ),

                        "settle_duration_sec": ParameterValue(
                            settle_duration_sec,
                            value_type=float,
                        ),

                        "settle_timeout_sec": ParameterValue(
                            settle_timeout_sec,
                            value_type=float,
                        ),
                    }
                ],
            ),
        ]
    )
