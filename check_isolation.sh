#!/usr/bin/env bash
# ============================================================
# 隔离自检脚本 —— 全面检查 lzh 工作空间是否被污染
#   用法: bash ~/g1d_ros2_lzh/check_isolation.sh
#
# 检查项:
#   1. ROS_DOMAIN_ID 是否为 42
#   2. AMENT_PREFIX_PATH 是否只含 系统ROS + 我的workspace
#   3. 工作区 src 下是否只有 lzh_* 包
#   4. git 提交身份是否为本仓库级配置
#   5. credential helper 是否指向隔离的 gh 配置
#   6. 师兄工作空间是否未被修改
# ============================================================

RED='\033[0;31m'; GRN='\033[0;32m'; YLW='\033[0;33m'; RST='\033[0m'
FAIL=0

ok()   { echo -e "${GRN}  ✅ $1${RST}"; }
bad()  { echo -e "${RED}  ❌ $1${RST}"; FAIL=1; }
warn() { echo -e "${YLW}  ⚠️  $1${RST}"; }

WS="$HOME/g1d_ros2_lzh"

echo "============================================"
echo " lzh 工作空间 隔离自检"
echo "============================================"

# 1. ROS_DOMAIN_ID
echo "[1] ROS_DOMAIN_ID"
if [ "${ROS_DOMAIN_ID:-}" = "42" ]; then
  ok "ROS_DOMAIN_ID=42 (与师兄的 0 隔离)"
elif [ -z "${ROS_DOMAIN_ID:-}" ]; then
  bad "未设置! 请 source ~/g1d_ros2_lzh/setup_env.sh"
else
  bad "ROS_DOMAIN_ID=$ROS_DOMAIN_ID, 应为 42"
fi

# 2. ROS-related paths
echo "[2] ROS overlay paths"
bad_found=0
for name in AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH; do
  value="${!name:-}"
  IFS=: read -r -a entries <<< "$value"
  for p in "${entries[@]}"; do
    case "$p" in
      *g1d_ros2_ws*|*unitree_ros_official*|*g1d_ros2_yqqh*)
        bad "$name 污染: $p"; bad_found=1 ;;
    esac
  done
done
[ $bad_found -eq 0 ] && ok "ROS 相关路径未混入禁止的 overlay"

# 3. src 下的包名
echo "[3] src/ 包名规范"
if [ -d "$WS/src" ]; then
  bad_pkg=0
  for d in "$WS"/src/*/; do
    [ -d "$d" ] || continue
    n=$(basename "$d")
    case "$n" in
      lzh_*) ok "$n (合规前缀 lzh_)" ;;
      *) bad "$n (不是 lzh_ 前缀, 会与师兄撞名!)"; bad_pkg=1 ;;
    esac
  done
  [ $bad_pkg -eq 0 ] && ok "所有包名合规"
else
  warn "src/ 不存在"
fi

# 4. git 身份
echo "[4] git 提交身份"
gn=$(cd "$WS" && git config --local --get user.name 2>/dev/null)
ge=$(cd "$WS" && git config --local --get user.email 2>/dev/null)
if [ "$gn" = "dousica" ]; then
  ok "仓库级身份: $gn <$ge>"
else
  bad "仓库级 user.name=$gn (应为 dousica)"
fi

# 5. credential helper
echo "[5] git credential 隔离"
h=$(cd "$WS" && git config --local --get-all credential.https://github.com.helper 2>/dev/null | head -1)
case "$h" in
  *gh-lzh*) ok "helper 指向隔离配置: $h" ;;
  "") warn "未配置仓库级 helper (git push 会失败)" ;;
  *) bad "helper 指向: $h (应指向 gh-lzh)" ;;
esac

# 6. 师兄工作空间完整性
echo "[6] 师兄工作空间未被修改"
SRCDIR="$HOME/g1d_ros2_ws/src"
if [ -d "$SRCDIR" ]; then
  # 检查师兄的包里是否出现了 lzh_ 前缀(说明我们污染了他)
  cont=0
  for d in "$SRCDIR"/lzh_*; do
    [ -d "$d" ] && { bad "师兄工作空间里出现了 lzh_* 包: $d"; cont=1; }
  done
  [ $cont -eq 0 ] && ok "师兄工作空间干净 (共 $(ls -d $SRCDIR/*/ | wc -l) 个包, 无 lzh_ 残留)"
else
  warn "找不到 $SRCDIR"
fi

echo "============================================"
if [ $FAIL -eq 0 ]; then
  echo -e "${GRN} 总体结论: 隔离完好 ✅${RST}"
else
  echo -e "${RED} 总体结论: 存在隔离问题 ❌${RST}"
fi
echo "============================================"
exit $FAIL
