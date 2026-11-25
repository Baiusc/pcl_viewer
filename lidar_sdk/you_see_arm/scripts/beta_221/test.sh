#!/bin/sh
# 测试文件全部位于 RAM；不覆盖原应用、驱动库、标定或 Flash。
set -eu
BASE=/tmp/beta-test
CTL=$BASE/payload/bin/lidar_mgrctl
SOCKET=$BASE/control.sock
mgr_pid() {
    [ -f "$BASE/manager.pid" ] || return 1
    pid=$(cat "$BASE/manager.pid")
    case "$pid" in ''|*[!0-9]*) return 1 ;; esac
    [ "$(readlink "/proc/$pid/exe" 2>/dev/null || true)" = "$BASE/payload/bin/lidar_mgrd" ] || return 1
    echo "$pid"
}
stop_original() {
    for pid in $(pidof you_see 2>/dev/null || true); do
        if [ "$(readlink "/proc/$pid/exe" 2>/dev/null || true)" = /root/you_see ]; then
            echo "停止原应用 PID=$pid"
            kill -INT "$pid"
            count=0
            while kill -0 "$pid" 2>/dev/null; do
                count=$((count + 1))
                if [ "$count" -ge 100 ]; then
                    echo '原应用 10 秒内未退出，测试未启动；可重启雷达恢复后联系研发检查' >&2
                    return 1
                fi
                sleep 0.1
            done
        fi
    done
}
stop_manager() {
    pid=$(mgr_pid) || { echo '测试管理服务未运行'; return 0; }
    kill -TERM "$pid"
    count=0
    while kill -0 "$pid" 2>/dev/null; do
        count=$((count + 1))
        if [ "$count" -ge 120 ]; then echo '管理服务未按时退出，请检查日志或重启设备' >&2; return 1; fi
        sleep 0.1
    done
    echo '测试管理服务已退出；DMA 未宣称停止，保持原有点云硬件布局'
}
cd "$BASE"
case "${1:-}" in
check)
    sha256sum -c bundle.sha256
    tools/lidar_crypto dgst -sha256 -verify test-public.pem -signature payload/manifest.sig payload/manifest.json
    sha256sum -c module.sha256
    ;;
start)
    if mgr_pid >/dev/null; then echo '测试管理服务已运行，请先 stop' >&2; exit 1; fi
    sh "$BASE/test.sh" check
    stop_original
    if fuser /dev/lidar > "$BASE/device-users.log" 2>&1; then cat "$BASE/device-users.log" >&2; exit 1; fi
    session="$(date +%s)-$$"
    config="$BASE/manager.$session.json"
    sed -e "s|$BASE/state|$BASE/state-$session|g" \
        -e "s|$BASE/runtime|$BASE/runtime-$session|g" manager.template.json > "$config"
    printf '%s\n' "$config" > active-config
    nohup payload/bin/lidar_mgrd "$config" > manager.log 2>&1 &
    echo $! > manager.pid
    echo "已启动测试会话：$session；每次 start 新建状态目录，旧记录保留"
    sleep 1
    "$CTL" "$SOCKET" status
    ;;
status) "$CTL" "$SOCKET" status ;;
switch)
    case "${2:-}" in 1|2) ;; *) echo '用法：sh test.sh switch 1 或 switch 2' >&2; exit 2 ;; esac
    "$CTL" "$SOCKET" switch "$2"
    ;;
stop) stop_manager ;;
restore)
    stop_manager
    if fuser /dev/lidar > "$BASE/device-users.log" 2>&1; then
        echo '设备仍被占用；若原程序已运行则无需重复恢复' >&2
        cat "$BASE/device-users.log" >&2
        exit 1
    fi
    nohup /root/you_see > "$BASE/original-restored.log" 2>&1 &
    echo "已重新启动原应用，PID=$!"
    ;;
logs) tail -60 manager.log; [ ! -f app.log ] || tail -30 app.log ;;
*) echo '用法：sh test.sh check|start|status|switch 1|switch 2|stop|restore|logs'; exit 2 ;;
esac
