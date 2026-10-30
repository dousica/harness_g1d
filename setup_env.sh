#!/usr/bin/env bash
# ============================================================
# 我的 ROS 2 工作空间环境脚本
#   用法: source ~/g1d_ros2_lzh/setup_env.sh
#
# 做三件事(顺序固定, 不可调换):
#   ① 要求调用者显式设置 ROS_DOMAIN_ID=42
#   ② source 系统 ROS 2
#   ③ source 我的 overlay
#
# 隔离保证:
#   - 绝不 source 师兄的 ~/g1d_ros2_ws
#   - 绝不 source 任何 unitree_ros_official 之类的 overlay
#   - 每次 source 后都会做一次隔离自检, 发现污染立即报错
# ============================================================

export LZH_WS="$HOME/g1d_ros2_lzh"

if [ "${ROS_DOMAIN_ID:-}" != "42" ]; then
  echo "[setup_env] 错误: 必须先显式 export ROS_DOMAIN_ID=42 (当前: ${ROS_DOMAIN_ID:-未设置})" >&2
  return 1 2>/dev/null || exit 1
fi

_lzh_check_overlay_paths() {
  local name value entry marker
  local forbidden=(g1d_ros2_ws g1d_ros2_yqqh unitree_ros_official)
  for name in AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH; do
    value="${!name:-}"
    IFS=: read -r -a entries <<< "$value"
    for entry in "${entries[@]}"; do
      for marker in "${forbidden[@]}"; do
        if [[ "$entry" == *"$marker"* ]]; then
          echo "[setup_env] 错误: $name 混入了禁止的 overlay: $entry" >&2
          return 1
        fi
      done
    done
  done
}

if ! _lzh_check_overlay_paths; then
  return 1 2>/dev/null || exit 1
fi

# Unitree SDK2 is a local CMake package used by lzh_sdk_bridge.
export CMAKE_PREFIX_PATH="$HOME/unitree_sdk2/install${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="$HOME/unitree_sdk2/install/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

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
if ! _lzh_check_overlay_paths; then
  return 1 2>/dev/null || exit 1
fi
has_lzh_overlay=0
IFS=: read -r -a ament_entries <<< "${AMENT_PREFIX_PATH:-}"
for entry in "${ament_entries[@]}"; do
  case "$entry" in
    "$LZH_WS"/install/*) has_lzh_overlay=1 ;;
  esac
done
if [ "$has_lzh_overlay" -ne 1 ]; then
  echo "[setup_env] 错误: 自己的 install overlay 未加载 ($LZH_WS/install)" >&2
  return 1 2>/dev/null || exit 1
fi

# 默认静默: 需要查看信息时先 export LZH_SETUP_QUIET=0 再 source
if [ -z "${LZH_SETUP_QUIET:-1}" ] || [ "${LZH_SETUP_QUIET}" = "0" ]; then
  echo "[setup_env] ROS_DOMAIN_ID=$ROS_DOMAIN_ID  工作空间=$LZH_WS"
fi
