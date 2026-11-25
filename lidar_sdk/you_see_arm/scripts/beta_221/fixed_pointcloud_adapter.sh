#!/bin/sh
# 仅供 221 固定点云布局下测试应用重开；不切换、停采或复位 DMA。
set -eu
BASE=/tmp/beta-test
case "${1:-}" in prepare|recover) ;; *) exit 2 ;; esac
case "${2:-}" in 1|2) ;; *) echo '仅允许原始点云和解算点云' >&2; exit 2 ;; esac
case "${3:-}" in -1|1|2) ;; *) echo '上一个模式不属于本测试布局' >&2; exit 2 ;; esac
sha256sum -c "$BASE/module.sha256" || exit 1
if fuser /dev/lidar > "$BASE/device-users.log" 2>&1; then
    echo '/dev/lidar 仍被占用，拒绝启动第二个采集应用' >&2
    cat "$BASE/device-users.log" >&2
    exit 1
fi
echo "固定点云测试：$1 -> work_mode=$2；内核模块不变、设备句柄已释放，不执行 DMA 复位"
