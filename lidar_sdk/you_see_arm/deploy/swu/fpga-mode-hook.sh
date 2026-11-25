#!/bin/sh
# 由签名 SWU 调用，只负责 FPGA 配置 Flash 的烧录和回读。
set -eu
umask 077
. "${LIDAR_SERVICE_CONFIG:-/etc/lidar/service.conf}"

stage=${LIDAR_FPGA_STAGE:-/run/lidar-fpga-stage}
service=${LIDAR_SERVICE_EXEC:-/usr/libexec/lidar/lidar-service}
platform=${LIDAR_PLATFORM_EXEC:-/usr/libexec/lidar/lidar-platform-beta}
state_dir=$LIDAR_RAM_DATA/lidar/state
profile=$state_dir/fpga-profile
jtag=$LIDAR_RUN/fpga-jtag

fail() { echo "FPGA 更新：$*" >&2; exit 1; }
regular() { [ -f "$1" ] && [ ! -L "$1" ] && [ -s "$1" ]; }
digest() {
    checksum=$(sha256sum "$1") || return 1
    printf '%s\n' "${checksum%% *}"
}
save() {
    "$LIDAR_STORAGE_EXEC" save "$1" "$2" &&
        "$LIDAR_BOOT" storage-replace "$2" "$3"
}
program() {
    if [ -n "${SWUPDATE_INFO_FD:-}" ]; then
        "$stage/lattice-flash" program --progress-fd "$SWUPDATE_INFO_FD" "$1"
    else
        "$stage/lattice-flash" program "$1"
    fi
}
[ "${LIDAR_STORAGE_MODE:-}" = boot-copy ] || fail '需要支持按需保存的 Beta 系统'
case "${1:-}" in
    preinst|postinst)
        exec "$LIDAR_BOOT" storage-lock "$LIDAR_RAM_DATA.ota-lock" "$0" "$1-locked"
        ;;
    preinst-locked)
        [ ! -L "$stage" ] || fail '暂存目录非法'
        mkdir -p "$stage"
        rm -f "$stage/target.bit" "$stage/lattice-flash"
        ;;
    postinst-locked) ;;
    *) exit 2 ;;
esac
[ "$1" = postinst-locked ] || exit 0

if [ -e "$jtag" ] || [ -L "$jtag" ]; then
    regular "$jtag" && [ "$(cat "$jtag")" = active ] || fail 'JTAG 烧录期间不能执行网口升级'
fi
[ -d "$stage" ] && [ ! -L "$stage" ] || fail '暂存目录缺失'
for file in target.bit lattice-flash; do
    regular "$stage/$file" || fail "缺少载荷 $file"
done
[ -x "$stage/lattice-flash" ] || fail '烧录器不可执行'
new_hash=$(digest "$stage/target.bit") || fail '无法计算目标镜像摘要'
mkdir -p "$state_dir" "$LIDAR_RUN"
work=$(mktemp -d "$LIDAR_RUN/fpga-flash.XXXXXX")

finish() {
    result=$?
    trap - EXIT HUP INT TERM
    if [ "$result" != 0 ]; then
        echo 'FPGA 更新失败，可通过 SWUpdate 重新上传签名包' >&2
    fi
    rm -rf "$work"
    exit "$result"
}
trap finish EXIT
trap 'exit 1' HUP INT TERM
printf 'pending %s\n' "$new_hash" > "$work/pending"
printf 'installed %s\n' "$new_hash" > "$work/new-profile"
"$service" stop
save fpga-profile "$work/pending" "$profile"
"$platform" quiesce
program "$stage/target.bit" || fail '目标镜像烧写、回读或重载失败'
save fpga-profile "$work/new-profile" "$profile"
rm -f "$jtag"
echo 'FPGA 烧录、回读及重载完成'
