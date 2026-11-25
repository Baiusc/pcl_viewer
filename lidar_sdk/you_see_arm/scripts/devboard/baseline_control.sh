#!/bin/sh
# 仅控制独立开发板验收实例；不加载驱动、不操作电机、不修改生产配置。
set -eu
umask 077
RUN=$(dirname "$(readlink -f "$0")")
. "$RUN/paths.conf"
[ "$(cat "$RUN/TEST_ENVIRONMENT")" = "DEVBOARD-MOCK:$RUN_ID" ] || exit 1
case "$DATA" in /mnt/mtdblock2/lidar-acceptance/run-*) ;; *) exit 1 ;; esac
case "$PARENT" in /run/lidar-services/acceptance_run-*) ;; *) exit 1 ;; esac
export LIDAR_SERVICE_CONFIG=$RUN/service.conf
export LIDAR_DEVBOARD_FLAGS=$RUN/control
SERVICE=/usr/libexec/lidar/lidar-service
BOOT=/usr/libexec/lidar/lidar_boot
CHILD=$RUN/services/lidar_mgrd

owned_data() {
    [ ! -L "$DATA" ] && [ "$(cat "$DATA/TEST_ENVIRONMENT")" = "DEVBOARD-MOCK:$RUN_ID" ]
}
stopped() {
    if "$BOOT" ready "$RUN/boot.json" >/dev/null 2>&1; then
        echo '验收实例仍在运行，拒绝修改测试包' >&2
        return 1
    fi
}
case "${1:-}" in
setup)
    grep -F ' /mnt/mtdblock2 jffs2 rw,' /proc/mounts >/dev/null
    [ ! -e "$PARENT" ] && [ ! -L "$PARENT" ] || exit 1
    if [ -d "$RUN/seed" ]; then
        [ ! -e "$DATA" ] && [ ! -L "$DATA" ] || exit 1
        [ ! -L /mnt/mtdblock2/lidar-acceptance ] || exit 1
        mkdir -p /mnt/mtdblock2/lidar-acceptance
        chmod 700 /mnt/mtdblock2/lidar-acceptance
        mv "$RUN/seed" "$DATA"
    fi
    owned_data
    "$BOOT" verify "$RUN/boot.json"
    "$SERVICE" enable
    cp -R "$RUN/http-template" "$RUN/services/http"
    # 放入已经运行的镜像监督树，不修改 inittab 或系统启动文件。
    mv "$RUN/parent-template" "$PARENT"
    ;;
status) owned_data; "$BOOT" status "$RUN/boot.json" ;;
switch) owned_data; "$CTL" "$RUN/control.sock" switch "${2:?缺少模式}" ;;
start|stop|restart) owned_data; "$SERVICE" "$1" ;;
fail-mode1) : > "$RUN/control/timeout-mode-1" ;;
clear-failure) rm -f "$RUN/control/timeout-mode-1" ;;
crash-manager)
    owned_data
    read -r pid < "$CHILD/supervise/pid"
    case "$pid" in ''|*[!0-9]*) exit 1 ;; esac
    [ "$(readlink "/proc/$pid/exe")" = "$DATA/releases/mock-v1/bin/lidar_mgrd" ] || exit 1
    kill -KILL "$pid"
    ;;
tamper)
    owned_data; stopped
    [ ! -e "$RUN/original-you-see" ] || exit 1
    cp -p "$DATA/releases/mock-v1/bin/you_see" "$RUN/original-you-see"
    printf '\nDEVBOARD_BASELINE_TAMPER\n' >> "$DATA/releases/mock-v1/bin/you_see"
    ;;
restore-package)
    owned_data; stopped
    [ -f "$RUN/original-you-see" ] || exit 1
    cp -p "$RUN/original-you-see" "$DATA/releases/mock-v1/bin/you_see"
    "$BOOT" verify "$RUN/boot.json"
    ;;
verify) owned_data; "$BOOT" verify "$RUN/boot.json" ;;
manager-pid) cat "$CHILD/supervise/pid" ;;
failures) cat "$RUN/failures" ;;
cleanup)
    owned_data
    "$SERVICE" stop
    /usr/bin/sv -w 8 shutdown "$RUN/services/http"
    /usr/bin/sv -w 8 shutdown "$CHILD"
    : > "$PARENT/down"
    /usr/bin/sv -w 8 shutdown "$PARENT"
    mv "$PARENT" "$RUN/detached-parent"
    echo '仅验收实例已停止；Flash 测试数据和本机报告保留。'
    ;;
*) echo '用法：control.sh setup|status|switch 1或2|start|stop|restart|fail-mode1|clear-failure|crash-manager|tamper|restore-package|verify|manager-pid|failures|cleanup' >&2; exit 2 ;;
esac
