#!/bin/sh
# 仅供开发板 RAM 演示，所有命令限定到本测试目录。
set -eu
BASE=$(dirname "$(readlink -f "$0")")
[ "$(cat "$BASE/TEST_ENVIRONMENT")" = DEVBOARD-MOCK ] || exit 1
export LIDAR_SERVICE_CONFIG=$BASE/service.conf
export LIDAR_SERVICE_EXEC=$BASE/tools/lidar-service
export LIDAR_DEVBOARD_FLAGS=$BASE/control
SERVICE=$BASE/services/lidar_mgrd
SOCKET=$BASE/run/control.sock
CTL=$BASE/tools/lidar_mgrctl
SV=/usr/bin/sv
INIT_ENTRY="::respawn:$BASE/test.sh scan"

manager_pid() {
    read -r pid < "$SERVICE/supervise/pid"
    case "$pid" in ''|*[!0-9]*) return 1 ;; esac
    [ "$(readlink "/proc/$pid/exe")" = "$BASE/data/releases/mock-v1/bin/lidar_mgrd" ] || return 1
    printf '%s\n' "$pid"
}

stopped() {
    if "$BASE/tools/lidar_boot" ready "$BASE/boot.json" >/dev/null 2>&1; then
        echo '请先执行 sh test.sh stop，不能修改正在使用的软件包或状态' >&2
        return 1
    fi
}

case "${1:-}" in
setup)
    cd "$BASE"
    sha256sum -c bundle.sha256 > setup-verification.log
    "$BASE/tools/lidar_boot" verify "$BASE/boot.json"
    "$BASE/tools/lidar-storage" start
    "$BASE/tools/lidar-service" enable
    if [ ! -d "$BASE/services/http" ]; then cp -R "$BASE/template/http" "$BASE/services/http"; fi
    if ! grep -F -x "$INIT_ENTRY" /etc/inittab >/dev/null; then
        [ -f "$BASE/original-inittab" ] || cp -p /etc/inittab "$BASE/original-inittab"
        printf '\n# 开发板 RAM 服务测试；整板重启后消失\n%s\n' "$INIT_ENTRY" >> /etc/inittab
        kill -HUP 1
    fi
    echo '已在 RAM 的 inittab 启用测试监督树；保留原串口、网络和 SWUpdate。'
    echo '稍后运行 sh test.sh status 查看状态。'
    ;;
scan)
    echo $$ > "$BASE/scan.pid"
    exec "$BASE/tools/lidar-service" scan
    ;;
status)
    echo '【开发板 RAM 测试】DEVBOARD-MOCK；只有模拟采集，无真实点云；整板重启会清空。'
    "$SV" status "$SERVICE" || true
    "$SV" status "$BASE/services/http" || true
    if ! "$CTL" "$SOCKET" status; then echo '管理服务当前不可查询；停止服务时属于预期。'; fi
    ;;
switch)
    case "${2:-}" in 1|2) ;; *) echo '用法：sh test.sh switch 1 或 switch 2' >&2; exit 2 ;; esac
    "$CTL" "$SOCKET" switch "$2"
    ;;
start|stop|restart)
    exec "$BASE/tools/lidar-service" "$1"
    ;;
state)
    cat "$BASE/data/state/state.json"
    printf '\n'
    ;;
logs)
    grep 'lidar_mgrd' /var/log/messages | tail -30 || true
    [ ! -f "$BASE/http/error.log" ] || tail -15 "$BASE/http/error.log"
    [ ! -f "$BASE/run/service-error" ] || cat "$BASE/run/service-error"
    ;;
crash-manager)
    pid=$(manager_pid)
    echo "仅结束本测试的管理进程 PID=$pid，观察 runsv 自动拉起。"
    kill -KILL "$pid"
    ;;
fail-mode1)
    : > "$BASE/control/timeout-mode-1"
    echo '已设置：下次进入模拟模式 1 时不报告数据，验证超时回滚。'
    ;;
clear-failure)
    rm -f "$BASE/control/timeout-mode-1"
    echo '已移除模式 1 超时标记。'
    ;;
tamper)
    stopped
    [ -f "$BASE/backup/you_see" ] || cp -p "$BASE/data/releases/mock-v1/bin/you_see" "$BASE/backup/you_see"
    printf '\nDEVBOARD_TAMPER\n' >> "$BASE/data/releases/mock-v1/bin/you_see"
    echo '仅修改了开发板模拟程序的测试副本；签名保持原值，下一次启动应被拒绝。'
    ;;
restore-package)
    stopped
    [ -f "$BASE/backup/you_see" ] || { echo '没有需要恢复的测试副本'; exit 1; }
    cp -p "$BASE/backup/you_see" "$BASE/data/releases/mock-v1/bin/you_see"
    "$BASE/tools/lidar_boot" verify "$BASE/boot.json"
    echo '模拟软件包已恢复；可执行 sh test.sh start。'
    ;;
reset-test)
    stopped
    if [ -f "$BASE/data/state/state.json" ]; then
        cp -p "$BASE/data/state/state.json" "$BASE/history/state.$(date +%s).$$.json"
        rm "$BASE/data/state/state.json"
    fi
    rm -f "$BASE/control/timeout-mode-1"
    echo '已备份并重置 DEVBOARD-MOCK 模拟状态。此命令不能用于真实雷达。'
    ;;
cleanup)
    "$BASE/tools/lidar-service" stop
    grep -F -v "$INIT_ENTRY" /etc/inittab > "$BASE/inittab.without-test"
    cat "$BASE/inittab.without-test" > /etc/inittab
    kill -HUP 1
    if [ -f "$BASE/scan.pid" ]; then
        read -r pid < "$BASE/scan.pid"
        case "$pid" in ''|*[!0-9]*) exit 1 ;; esac
        if [ -r "/proc/$pid/cmdline" ] && tr '\000' ' ' < "/proc/$pid/cmdline" | grep -F "$BASE/services" >/dev/null; then
            kill -TERM "$pid"
        fi
    fi
    "$SV" -w 5 shutdown "$BASE/services/http" || true
    "$SV" -w 5 shutdown "$SERVICE" || true
    echo '测试监督树已停止，保留目录与日志；没有覆盖其他 inittab 配置。'
    ;;
*) echo '用法：sh test.sh setup|status|switch 1|switch 2|start|stop|restart|state|logs|crash-manager|fail-mode1|clear-failure|tamper|restore-package|reset-test|cleanup' >&2; exit 2 ;;
esac
