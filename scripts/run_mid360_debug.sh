#!/usr/bin/env bash
#
# run_mid360_debug.sh
# 启动 FAST-LIVO2 的 mapping 节点，并保证崩溃时能生成 core 文件。
#
# 特点
#   * 只做“内存级”改动（core_pattern / ulimit），不写任何系统配置文件，重启即恢复
#   * 退出时（含 Ctrl-C）自动把 core_pattern 还原成原来的值（除非 --keep）
#   * roslaunch 异常退出后自动定位最新的 core，可选用 gdb 直接打印所有线程堆栈
#
# 用法
#   ./scripts/run_mid360_debug.sh [选项] [launch文件]
#
# 选项
#   -g, --gdb         退出后用 gdb 自动分析最新的 core
#   -d, --dir DIR     core 输出目录（默认 /tmp）
#   -p, --pkg PKG     launch 所在 ROS 包（默认 fast_livo2）
#   -k, --keep        退出后不还原 core_pattern
#   -n, --no-launch   只准备 core 环境，不启动 roslaunch（自测用）
#   -h, --help        显示帮助
#
# 示例
#   ./scripts/run_mid360_debug.sh                 # 跑 mid360 launch，崩溃后手动 gdb
#   ./scripts/run_mid360_debug.sh -g              # 崩溃后自动打印所有线程堆栈
#   ./scripts/run_mid360_debug.sh -d ~/cores -g   # core 放到 ~/cores
#

set -u -o pipefail

# ------------------------------------------------------------------ 默认参数
CORE_DIR="/tmp"
LAUNCH_PKG="fast_livo2"
LAUNCH_FILE="mapping_mid360.launch"
NODE_TYPE="fastlivo_mapping2"
RUN_GDB=0
KEEP_PATTERN=0
DO_LAUNCH=1
GDB_BIN="$(command -v gdb || true)"

# ------------------------------------------------------------------ 输出 helper
if [ -t 1 ]; then
  C_R=$'\e[31m'; C_G=$'\e[32m'; C_Y=$'\e[33m'; C_C=$'\e[36m'; C_0=$'\e[0m'
else
  C_R=; C_G=; C_Y=; C_C=; C_0=
fi
say()  { printf '%s[ .. ]%s %s\n' "$C_C" "$C_0" "$*"; }
ok()   { printf '%s[ OK ]%s %s\n' "$C_G" "$C_0" "$*"; }
warn() { printf '%s[WARN]%s %s\n' "$C_Y" "$C_0" "$*" >&2; }
die()  { printf '%s[FAIL]%s %s\n' "$C_R" "$C_0" "$*" >&2; exit 1; }

usage() {
  cat <<'EOF'
用法: run_mid360_debug.sh [选项] [launch文件]

选项:
  -g, --gdb         退出后用 gdb 自动分析最新的 core
  -d, --dir DIR     core 输出目录（默认 /tmp）
  -p, --pkg PKG     launch 所在 ROS 包（默认 fast_livo2）
  -k, --keep        退出后不还原 core_pattern
  -n, --no-launch   只准备 core 环境，不启动 roslaunch（自测用）
  -h, --help        显示本帮助

示例:
  ./scripts/run_mid360_debug.sh
  ./scripts/run_mid360_debug.sh -g
  ./scripts/run_mid360_debug.sh -d ~/cores -g
EOF
}

# ------------------------------------------------------------------ 解析参数
while [ $# -gt 0 ]; do
  case "$1" in
    -g|--gdb)       RUN_GDB=1; shift ;;
    -d|--dir)       CORE_DIR="${2:?缺少目录参数}"; shift 2 ;;
    -p|--pkg)       LAUNCH_PKG="${2:?缺少包名}"; shift 2 ;;
    -k|--keep)      KEEP_PATTERN=1; shift ;;
    -n|--no-launch) DO_LAUNCH=0; shift ;;
    -h|--help)      usage; exit 0 ;;
    -*)             die "未知选项: $1（用 -h 看帮助）" ;;
    *)              LAUNCH_FILE="$1"; shift ;;
  esac
done

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# ------------------------------------------------------------------ 定位工作空间
find_ws() {
  local c
  for c in "${FASTLIVO_WS:-}" "$REPO_ROOT/../.." "$HOME/lidar_ws" "$HOME/catkin_ws"; do
    [ -n "$c" ] || continue
    if [ -f "$c/devel/setup.bash" ] || [ -f "$c/install/setup.bash" ]; then
      (cd "$c" && pwd); return 0
    fi
  done
  return 1
}
WS="$(find_ws)" || die "找不到 catkin 工作空间，请先 export FASTLIVO_WS=/path/to/ws"

# ------------------------------------------------------------------ ROS 环境
if [ -z "${ROS_DISTRO:-}" ]; then
  for f in /opt/ros/*/setup.bash; do
    [ -f "$f" ] || continue
    set +u; . "$f"; set -u
  done
fi
[ -n "${ROS_DISTRO:-}" ] || die "未找到 ROS 环境（/opt/ros/*/setup.bash）"

set +u
[ -f "$WS/devel/setup.bash" ]   && . "$WS/devel/setup.bash"
[ -f "$WS/install/setup.bash" ] && . "$WS/install/setup.bash"
set -u
say "ROS_DISTRO=${ROS_DISTRO}   工作空间=${WS}"

# ------------------------------------------------------------------ 定位 launch / 可执行文件
resolve_launch() {
  local lf="$1" b
  b="$(basename "$lf")"
  if [ -f "$lf" ]; then printf '%s\n' "$lf"; return 0; fi
  if [ -f "$REPO_ROOT/launch/$b" ]; then printf '%s\n' "$REPO_ROOT/launch/$b"; return 0; fi
  return 1
}
LF="$(resolve_launch "$LAUNCH_FILE")" || die "找不到 launch 文件: $LAUNCH_FILE"

# 从 launch 文件里解析出该包的 node type，避免硬编码
t="$(tr '\n' ' ' < "$LF" \
     | grep -oE "<node[^>]*pkg=\"$LAUNCH_PKG\"[^>]*>" \
     | grep -oE 'type="[^"]+"' | head -1 | cut -d'"' -f2)"
[ -n "${t:-}" ] && NODE_TYPE="$t"

BIN=""
for c in "$WS/devel/lib/$LAUNCH_PKG/$NODE_TYPE" "$WS/install/lib/$LAUNCH_PKG/$NODE_TYPE"; do
  if [ -x "$c" ]; then BIN="$c"; break; fi
done
if [ -n "$BIN" ]; then
  say "可执行文件: $BIN"
else
  warn "找不到 $NODE_TYPE，需先 catkin_make；gdb 自动分析将不可用"
fi

# ------------------------------------------------------------------ core 环境
CORE_PATTERN="${CORE_DIR%/}/core.%e.%p.%t"
ORIG_PATTERN="$(cat /proc/sys/kernel/core_pattern 2>/dev/null || true)"

as_root() { if [ "$(id -u)" -eq 0 ]; then "$@"; else sudo "$@"; fi; }

restore_pattern() {
  [ "$KEEP_PATTERN" -eq 1 ] && return 0
  [ -n "$ORIG_PATTERN" ] || return 0
  [ "$(cat /proc/sys/kernel/core_pattern 2>/dev/null)" = "$ORIG_PATTERN" ] && return 0
  if sudo -n sysctl -w kernel.core_pattern="$ORIG_PATTERN" >/dev/null 2>&1; then
    ok "已还原 core_pattern -> $ORIG_PATTERN"
  else
    warn "还原 core_pattern 失败，可手动执行: sudo sysctl -w kernel.core_pattern='$ORIG_PATTERN'"
  fi
}
trap restore_pattern EXIT
trap 'exit 130' INT TERM

mkdir -p "$CORE_DIR" 2>/dev/null || as_root mkdir -p "$CORE_DIR"
[ -w "$CORE_DIR" ] || warn "目录 $CORE_DIR 当前用户不可写，core 可能写不进去"

if [ "$(cat /proc/sys/kernel/core_pattern 2>/dev/null)" = "$CORE_PATTERN" ]; then
  ok "core_pattern 已是 $CORE_PATTERN"
else
  say "修改 core_pattern -> $CORE_PATTERN（需要 root，可能会提示输入密码）"
  if as_root sysctl -w kernel.core_pattern="$CORE_PATTERN" >/dev/null 2>&1; then
    ok "core_pattern 设置成功（仅本次生效，重启自动恢复）"
  else
    warn "自动设置失败，请先手动执行："
    printf '        sudo sysctl -w kernel.core_pattern=%q\n' "$CORE_PATTERN"
  fi
fi

if ulimit -c unlimited 2>/dev/null; then
  ok "ulimit -c = $(ulimit -c)"
else
  warn "无法设置 ulimit -c unlimited（可能受 hard limit 限制）"
fi

say "崩溃时 core 将写入: $CORE_DIR"
printf '%s\n' "----------------------------------------------------------------------"

# ------------------------------------------------------------------ 启动
STAMP="$(mktemp)"; touch "$STAMP"
rc=0
if [ "$DO_LAUNCH" -eq 1 ]; then
  say "启动: roslaunch $LAUNCH_PKG $LAUNCH_FILE"
  roslaunch "$LAUNCH_PKG" "$LAUNCH_FILE"
  rc=$?
  say "roslaunch 已退出，退出码 = $rc"
else
  say "--no-launch：跳过 roslaunch（仅自测环境准备）"
fi

# ------------------------------------------------------------------ 崩溃后处理
NEW_CORE="$(find "$CORE_DIR" -maxdepth 1 -type f -name 'core.*' -newer "$STAMP" 2>/dev/null | head -n1)"
rm -f "$STAMP"

if [ -n "$NEW_CORE" ]; then
  ok "生成 core: $NEW_CORE  ($(du -h "$NEW_CORE" 2>/dev/null | cut -f1))"
  if [ "$RUN_GDB" -eq 1 ] && [ -n "$BIN" ] && [ -n "$GDB_BIN" ]; then
    say "gdb 分析中..."
    "$GDB_BIN" -q -batch \
      -ex 'set pagination off' \
      -ex 'echo \n===== 崩溃线程 bt =====\n' -ex 'bt' \
      -ex 'echo \n===== 所有线程 bt full =====\n' -ex 'thread apply all bt full' \
      "$BIN" "$NEW_CORE"
  elif [ "$RUN_GDB" -eq 1 ]; then
    warn "无法自动 gdb（缺少 gdb 或可执行文件），请手动：gdb $BIN $NEW_CORE"
  else
    say "手动分析: gdb ${BIN:-<可执行文件>} $NEW_CORE"
  fi
elif [ "$rc" -ne 0 ]; then
  warn "roslaunch 异常退出（码 $rc），但没找到新的 core"
  case "$ORIG_PATTERN" in
    \|*) warn "core_pattern 仍是 apport 管道，可检查 /var/crash/，或确认上面 core_pattern 是否设置成功" ;;
  esac
  warn "也可能是别的进程崩溃（rviz / republish 等），或用 launch-prefix 直接挂 gdb"
fi

exit "$rc"
