from glob import glob
from setuptools import setup

setup(name='lzh_camera_bridge', version='0.1.0', packages=['lzh_camera_bridge'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/lzh_camera_bridge']),
                  ('share/lzh_camera_bridge', ['package.xml']),
                  ('share/lzh_camera_bridge/launch', glob('launch/*.launch.py'))],
      install_requires=['setuptools'], zip_safe=True, license='MIT',
      maintainer='yushu', maintainer_email='yushu@todo.todo',
      description='Workstation-only TeleImager to ROS 2 bridge',
      entry_points={'console_scripts': [
          'lzh_camera_bridge = lzh_camera_bridge.bridge:main',
          'lzh_camera_viewer = lzh_camera_bridge.viewer:main']})
