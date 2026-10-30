import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    domain = os.environ.get('ROS_DOMAIN_ID')
    if domain != '42':
        raise RuntimeError(f'ROS_DOMAIN_ID must be 42; got {domain!r}')
    for variable in (
        'AMENT_PREFIX_PATH', 'CMAKE_PREFIX_PATH', 'COLCON_PREFIX_PATH',
        'PYTHONPATH', 'LD_LIBRARY_PATH'
    ):
        for entry in os.environ.get(variable, '').split(os.pathsep):
            if any(marker in entry for marker in ('g1d_ros2_ws', 'g1d_ros2_yqqh', 'unitree_ros_official')):
                raise RuntimeError(f'{variable} contains forbidden overlay path: {entry}')
    defaults = {'server_ip': '192.168.123.164', 'topic_base': '/g1d/cameras',
                'head_port': '55555', 'left_wrist_port': '55557', 'right_wrist_port': '55556',
                'head_layout': 'sbs', 'head_order': 'lr', 'publish_rate': '15.0',
                'show_viewer': 'true', 'capture_dir': '~/g1d_vision_data',
                'calibration_dir': '~/.ros/g1d_camera_calibration', 'display_scale': '0.75', 'viewer_rate': '20.0'}
    typed = lambda name, kind: ParameterValue(LaunchConfiguration(name), value_type=kind)
    shared = {name: typed(name, str) for name in ('topic_base', 'head_layout')}
    bridge = dict(shared)
    bridge.update({name: typed(name, str) for name in ('server_ip', 'head_order', 'calibration_dir')})
    bridge.update({name: typed(name, int) for name in ('head_port', 'left_wrist_port', 'right_wrist_port')})
    bridge['publish_rate'] = typed('publish_rate', float)
    viewer = dict(shared)
    viewer.update({name: typed(name, int) for name in ('head_port', 'left_wrist_port', 'right_wrist_port')})
    viewer.update({'capture_dir': typed('capture_dir', str), 'display_scale': typed('display_scale', float)})
    viewer['viewer_rate'] = typed('viewer_rate', float)
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()],
        Node(package='lzh_camera_bridge', executable='lzh_camera_bridge', output='screen', parameters=[bridge]),
        Node(package='lzh_camera_bridge', executable='lzh_camera_viewer', output='screen', parameters=[viewer],
             condition=IfCondition(LaunchConfiguration('show_viewer'))),
    ])
