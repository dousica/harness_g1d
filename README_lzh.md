# G1-D ROS 2 Workspace

`g1d_ros2_lzh` is the self-contained ROS 2 workspace used by the LZH G1-D
Harness integration. Its runtime is built from packages under this repository's
`src/`; it does not source or load another robot workspace.

## Packages

| Package | Responsibility |
| --- | --- |
| `lzh_description` | G1-D URDF, meshes, and robot-state display launch |
| `lzh_moveit_config` | MoveIt planning groups, limits, controllers, and launch files |
| `lzh_sdk_bridge` | Unitree SDK2 feedback and guarded arm/gripper actions |
| `lzh_bringup` | G1-D state, MoveIt, TF, and bridge launch orchestration |
| `lzh_camera_bridge` | TeleImager image and CameraInfo ROS publishers |
| `lzh_motion_demo` | Explicitly confirmed Dex1 action client |
| `lzh_test_pkg` | Small ROS 2 environment smoke-test node |

The public ROS interfaces keep the model's joint/frame names and Harness camera
topics stable. Package and custom node names use the `lzh_` prefix. Standard
ROS interfaces such as `/joint_states` remain unchanged for the Harness.

## Requirements

- ROS 2 Humble and the MoveIt 2 Humble packages
- `colcon`, `rosdep`, `xacro`, and the ROS control/message packages
- Unitree SDK2 installed at `~/unitree_sdk2/install`
- Python dependencies for OpenCV, NumPy, PyYAML, and ZeroMQ

`lzh_sdk_bridge` uses the SDK2 CMake package directly. `setup_env.sh` adds its
include/library paths and the workspace overlay. It deliberately does not set
`ROS_DOMAIN_ID`; callers must set the required value explicitly.

## Build

Use a clean terminal that has not sourced a different robot workspace:

```bash
export ROS_DOMAIN_ID=42
source /home/yushu/g1d_ros2_lzh/setup_env.sh
cd /home/yushu/g1d_ros2_lzh
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source /home/yushu/g1d_ros2_lzh/setup_env.sh
bash /home/yushu/g1d_ros2_lzh/check_isolation.sh
```

`setup_env.sh` fails if the domain is unset or not `42`, if a known foreign
overlay appears in ROS-related paths, or if this workspace's install overlay
is unavailable. Open a fresh terminal to clear a contaminated environment.

## Runtime

Run each command in a separate terminal. Set the domain and source this
workspace in every ROS terminal:

```bash
export ROS_DOMAIN_ID=42
source /home/yushu/g1d_ros2_lzh/setup_env.sh
```

Start the LZH bridge and MoveIt. Real arm and gripper command gates default to
disabled:

```bash
ros2 launch lzh_bringup real_moveit.launch.py
```

Start the camera publisher in another terminal; `show_viewer:=false` omits its
desktop-only image viewer:

```bash
ros2 launch lzh_camera_bridge cameras.launch.py show_viewer:=false
```

The default camera topics are `/g1d/cameras/<camera>/image_raw` and
`/g1d/cameras/<camera>/camera_info`. Override the TeleImager address and port
arguments in `cameras.launch.py` for the target workstation/network.

The Harness runs in another terminal with the same domain and this overlay. It
does not start these ROS nodes. Its live G1-D preflight checks Domain 42 and the
LZH overlay before probing feedback, planning, actions, and configured camera
topics.

## Safety and Isolation

- `ROS_DOMAIN_ID=42` is mandatory for every ROS node in this integration.
- Bringup, camera launch files, SDK bridge executables, and Harness live clients
  fail fast on a wrong/unset domain or a known foreign workspace overlay.
- `lzh_sdk_bridge` is the sole ROS-to-SDK2 command bridge. Do not start a second
  writer for the same robot.
- MoveIt bringup defaults to dry-run; do not enable real execution until the
  physical safety checks and commissioning procedures are complete.
- Unitree SDK2 has its own DDS transport configuration. That is separate from
  ROS 2 discovery and does not change the ROS domain requirement.
- Camera receive timestamps are not hardware exposure timestamps. Validate
  stereo synchronization and calibration physically before using 3D points for
  robot motion.
