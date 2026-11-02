#!/usr/bin/env bash
# G1-D 控制台启动器
#   干净环境：numpy 必须是 2.x 才能带动 Pinocchio
#   依赖栈默认在同级目录的 unitree_gpt6/（可用 G1D_STACK 覆盖）
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STACK="${G1D_STACK:-$(cd "$ROOT/.." && pwd)/unitree_gpt6}"
PY="$STACK/g1d_draw/.venv/bin/python"

if [ ! -x "$PY" ]; then
  echo "找不到 Python 环境：$PY"
  echo "请把 unitree_robotskills 克隆到 $(dirname "$STACK")，或设置 G1D_STACK"
  exit 1
fi

unset PYTHONHOME || true
unset LD_LIBRARY_PATH || true
export PYTHONPATH="$ROOT:$(dirname "$STACK")"

mkdir -p "$ROOT/logs"
LOG="$ROOT/logs/console_$(date +%Y%m%d_%H%M%S).log"
exec > >(tee -a "$LOG") 2>&1
echo "[log] $LOG"

"$PY" - <<'PYEOF'
import numpy, pinocchio
print(f"[env] numpy {numpy.__version__}  pinocchio {pinocchio.__version__}")
if numpy.__version__.split(".")[0] != "2":
    raise SystemExit("[env] numpy must be 2.x")
PYEOF
exec "$PY" -u console.py "$@"
