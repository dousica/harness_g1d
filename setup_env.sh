#!/usr/bin/env bash
# ============================================================
# 我的 ROS 2 工作空间环境脚本
#   用法: source ~/g1d_ros2_lzh/setup_env.sh
#
# 做三件事(顺序固定, 不可调换):
#   ① 检查并设置 ROS_DOMAIN_ID=42   (与师兄的 domain 0 隔离)
#   ② source 系统 ROS 2
#   ③ source 我的 overlay
#
# 隔离保证:
#   - 绝不 source 师兄的 ~/g1d_ros2_ws
#   - 绝不 source 任何 unitree_ros_official 之类的 overlay
#   - 每次 source 后都会做一次隔离自检, 发现污染立即报错
# ============================================================

export LZH_WS="$HOME/g1d_ros2_lzh"

# Unitree SDK2 is a local CMake package used by lzh_sdk_bridge.
export CMAKE_PREFIX_PATH="$HOME/unitree_sdk2/install${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="$HOME/unitree_sdk2/install/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

# ---------- ① ROS_DOMAIN_ID ----------
export ROS_DOMAIN_ID=42

# ---------- ② 系统 ROS ----------
if ! ros2 --help >/dev/null 2>&1; then
  source /opt/ros/humble/setup.bash
fi

# ---------- ③ 我的 overlay ----------
if [ -f "$LZH_WS/install/setup.bash" ]; then
  source "$LZH_WS/install/setup.bash"
else
  echo "[setup_env] 警告: $LZH_WS/install/setup.bash 不存在, 请先 colcon build" >&2
fi

# ---------- 隔离自检 ----------
if [ -n "${AMENT_PREFIX_PATH:-}" ]; then
  for p in $(echo "$AMENT_PREFIX_PATH" | tr ':' '\n'); do
    case "$p" in
      */g1d_ros2_ws*|*/unitree_ros_official*|*/g1d_ros2_yqqh*)
        echo "[setup_env] ❌ 隔离冲突: AMENT_PREFIX_PATH 中含有 $p" >&2
        echo "[setup_env]    请开一个全新的终端重试" >&2
        ;;
    esac
  done
fi

echo "[setup_env] ✅ ROS_DOMAIN_ID=$ROS_DOMAIN_ID  工作空间=$LZH_WS"
