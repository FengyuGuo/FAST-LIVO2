# mmap_who.gdb - 抓取 ~1.5-1.8MB 的 mmap 调用栈, 用于定位"绕过 malloc 的内存增长"
#
# 用法(节点已经在跑的时候):
#   NODE=$(pgrep -f '^/home/guo/lidar_ws/devel/lib/fast_livo2/fastlivo_mapping2' | head -1)
#   sudo gdb -p $NODE -x scripts/mmap_who.gdb -batch
#   # 结果同时写到 /tmp/mmap_bt.txt
#
# 原理: mmap 的第 2 个参数(len)在 x86-64 上是 $rsi。
#       只对 1MiB ~ 2MiB 的请求打印调用栈, 命中 12 次后自动 detach 退出。

set pagination off
set confirm off
set height 0
set width 0
set logging file /tmp/mmap_bt.txt
set logging overwrite on
set logging on

set $n = 0

break mmap
commands
  silent
  # $rsi = length (x86-64 System V ABI 第 2 个参数)
  if $rsi >= 1048576 && $rsi <= 2097152
    set $n = $n + 1
    printf "\n########## mmap #%d  len=%ld bytes (%.2f MB) ##########\n", $n, $rsi, $rsi/1048576.0
    bt 12
  end
  if $n >= 12
    printf "\n[mmap_who] 已抓到 12 条, 停止。结果在 /tmp/mmap_bt.txt\n"
    detach
    quit
  end
  continue
end

printf "[mmap_who] 已挂上 mmap 断点, 等待 1-2MB 的 mmap 请求...\n"
continue
