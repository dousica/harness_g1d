cmake_minimum_required(VERSION 3.8)
project(lzh_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES README_HEAD_CAMERA_TF.md
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/publish_head_camera_tf.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME lzh_head_camera_tf_publisher
)

ament_package()
