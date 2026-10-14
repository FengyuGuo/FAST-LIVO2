#!/usr/bin/env bash
#
# profile_heaptrack.sh
# 用 heaptrack 抓取 fastlivo_mapping2 的堆 profile，并直接输出分析结果。
#
# 用法:
#   ./scripts/profile_heaptrack.sh [bag文件] [播放秒数]
# 默认:
#   bag  = /media/guo/fs/dog/recording_20261013_151631.bag
#   secs = 40
#
# 产物:
#   ./heaptrack.fastlivo_mapping2.<pid>.gz   原始 profile
#   ./heaptrack_report.txt                   分析结果(同时打到屏幕)
#
# 注意(踩过的坑):
#   1. 不要把 roslaunch 整体放进 heaptrack, 那样 profile 的是 python;
#      这里用 launch 文件里的 launch_prefix 参数, 只把节点包进去。
#   2. 绝对不要 pkill heaptrack! heaptrack 只有在被追踪进程正常退出时才写 .gz,
#      强杀它会把 profile 丢掉。这里对节点发 SIGINT, 并等 heaptrack 自己收尾。
#   3. 取节点 PID 必须用 '^/绝对路径' 锚定, 否则 pgrep -f 会命中 heaptrack 的包装进程。

set -u

BAG="${1:-/media/guo/fs/dog/recording_20261013_151631.bag}"
SECS="${2:-40}"

WS="${FASTLIVO_WS:-$HOME/lidar_ws}"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NODE_RE="^$WS/devel/lib/fast_livo2/fastlivo_mapping2"

say() { printf '\033[1;36m[ .. ]\033[0m %s\n' "$*"; }
ok()  { printf '\033[1;32m[ OK ]\033[0m %s\n' "$*"; }
die() { printf '\033[1;31m[FAIL]\033[0m %s\n' "$*" >&2; exit 1; }

command -v heaptrack >/dev/null || die "没装 heaptrack: sudo apt install heaptrack"
[ -f "$BAG" ] || die "找不到 bag: $BAG"

# heaptrack 1.0 把 .gz 写到【被追踪进程的 cwd】。
# 坑: roslaunch 默认把节点 cwd 设成 ~/.ros, 不是脚本的 PWD!
# 所以两个地方都要找。
OUTDIR="$PWD"
SEARCH_DIRS=("$OUTDIR" "$HOME/.ros")
say "profile 查找目录: ${SEARCH_DIRS[*]}"

# 只清理本次可能产生的旧文件
rm -f "$HOME/.ros"/heaptrack.fastlivo_mapping2.*.gz 2>/dev/null

source /opt/ros/melodic/setup.bash >/dev/null 2>&1
source "$WS/devel/setup.bash" >/dev/null 2>&1 || die "source $WS/devel/setup.bash 失败"

# ---------------- 启动 ----------------
say "启动 roscore"
pgrep -f "bin/roscore" >/dev/null || (setsid nohup roscore >/tmp/prof_roscore.log 2>&1 </dev/null &)
sleep 5

say "启动节点(heaptrack 包装)"
setsid nohup roslaunch fast_livo2 mapping_mid360.launch rviz:=false launch_prefix:=heaptrack \
  >/tmp/prof_node.log 2>&1 </dev/null &

for i in $(seq 1 30); do
  NODE="$(pgrep -f "$NODE_RE" | head -1)"
  [ -n "${NODE:-}" ] && break
  sleep 1
done
[ -n "${NODE:-}" ] || die "节点没起来, 看 /tmp/prof_node.log"
ok "节点 PID=$NODE"

# ---------------- 喂数据 ----------------
say "播放 $SECS 秒数据: $(basename "$BAG")"
rosbag play -r 1 -u "$SECS" "$BAG" >/tmp/prof_bag.log 2>&1
ok "播放结束"

sleep 3
RSS=$(awk '/VmRSS/{printf "%.0f", $2/1024}' "/proc/$NODE/status" 2>/dev/null || echo "?")
say "结束前 RSS = ${RSS} MB"

# ---------------- 优雅收尾, 让 heaptrack 落盘 ----------------
say "对节点发 SIGINT, 等 heaptrack 写 .gz (最多 60s)"
kill -INT "$NODE" 2>/dev/null

GZ=""
for i in $(seq 1 90); do
  GZ="$(ls -t $(printf '%s/heaptrack.fastlivo_mapping2.*.gz ' "${SEARCH_DIRS[@]}") 2>/dev/null | head -1)"
  [ -n "$GZ" ] && sleep 3 && break
  sleep 1
done
if [ -z "$GZ" ]; then
  echo "--- heaptrack 进程 ---"; pgrep -af heaptrack | head -5
  echo "--- 节点是否还活着 ---"; pgrep -af '^.*fastlivo_mapping2' | head -3
  die "没生成 .gz。原始日志: /tmp/prof_node.log (注意: heaptrack 只在节点正常退出时才落盘)"
fi

ok "profile: $GZ ($(du -h "$GZ" | cut -f1))"

# ---------------- 分析 ----------------
REPORT="$OUTDIR/heaptrack_report.txt"
cp -f "$GZ" "$OUTDIR/" 2>/dev/null || true
say "分析中 -> $REPORT"
{
  echo "########## heaptrack_print: $(basename "$GZ") ##########"
  heaptrack_print -f "$GZ" \
    --print-peaks=1 --peak-limit=15 \
    --print-allocators=1 \
    --print-leaks=1 \
    --print-overall-allocated=1
} >"$REPORT" 2>&1

ok "完成。先看这几段:"
echo "---------------------------------------------------------------"
grep -nE "^[A-Z][A-Z ]+$" "$REPORT" | head -20
echo "---------------------------------------------------------------"
sed -n '1,25p' "$REPORT"
echo "---------------------------------------------------------------"
echo "完整报告: $REPORT"
echo
echo "把 $REPORT 发给我, 我来定位到具体代码行。"
