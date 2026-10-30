# Fixed head-camera TF

`head_camera_tf.launch.py` publishes the calibrated transforms
`torso_link -> head_left_camera_optical_frame` and
`torso_link -> head_right_camera_optical_frame` on `/tf_static`.

The left transform is loaded from the eye-to-hand result. The right transform is
computed as `T_torso_left * inverse(T_left_right)` from the stereo result. Both
results use the original optical frames, not rectified-image coordinates.

Pass the measured calibration paths from the site configuration. To start only
this publisher:

```bash
export ROS_DOMAIN_ID=42
source /home/yushu/g1d_ros2_lzh/setup_env.sh
ros2 launch lzh_bringup head_camera_tf.launch.py
```

`real_moveit.launch.py` includes this publisher automatically. To use updated
calibration results, pass `handeye_result:=/path/to/result.yaml` and
`stereo_result:=/path/to/head_stereo.yaml` to either launch file.

Verify both edges after launch:

```bash
ros2 run tf2_ros tf2_echo torso_link head_left_camera_optical_frame
ros2 run tf2_ros tf2_echo torso_link head_right_camera_optical_frame
```

This adds the software TF edges only. Before using camera-derived points to
command the robot, physically verify the transforms with a stationary ChArUco
board at several known workspace positions. Do not run two copies of this
publisher at the same time.
