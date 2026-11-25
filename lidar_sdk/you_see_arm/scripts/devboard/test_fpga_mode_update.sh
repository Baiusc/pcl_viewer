#!/bin/sh
# 仅在本测试树内模拟 Flash、烧录和硬件，采集健康由模拟程序发送。
set -eu
umask 077
root=$(cd "$(dirname "$0")" && pwd)
export FPGA_TEST_ROOT=$root LIDAR_SERVICE_CONFIG=$root/service.conf LIDAR_DEVBOARD_FLAGS=$root/flags
export PATH=$root/mock-bin:$PATH
ram=$root/ram/lidar
medium=$root/medium/lidar
run=$root/run
stage=$root/stage
boot=$root/tools/lidar_boot
mkdir "$root/mock-bin" "$LIDAR_DEVBOARD_FLAGS" "$root/mount" "$root/medium" "$run"
mkdir -p "$ram/releases" "$ram/state" "$ram/config" "$ram/calibration"
cp -pR "$root/package" "$ram/releases/test"
ln -s releases/test "$ram/current"
echo '模拟标定' > "$ram/calibration/ys_calibration.ini"
echo '模拟角度' > "$ram/calibration/ys_encoder_angle.csv"
cp "$root/package/conf/ys_conf.ini" "$ram/config/ys_conf.ini"
printf 'pointcloud\n' > "$root/hardware"
cp -pPR "$ram" "$root/medium/"
: > "$root/mounts"

cat > "$root/mock-bin/mount" <<'EOF'
#!/bin/sh
set -eu
mv "$6" "$FPGA_TEST_ROOT/empty"
mv "$FPGA_TEST_ROOT/medium" "$6"
printf '%s %s jffs2 %s 0 0\n' "$5" "$6" "$4" > "$FPGA_TEST_ROOT/mounts"
EOF
cat > "$root/mock-bin/umount" <<'EOF'
#!/bin/sh
set -eu
mv "$1" "$FPGA_TEST_ROOT/medium"
mv "$FPGA_TEST_ROOT/empty" "$1"
: > "$FPGA_TEST_ROOT/mounts"
EOF
cat > "$root/mock-bin/platform" <<'EOF'
#!/bin/sh
set -eu
root=$FPGA_TEST_ROOT
case "$1" in
    quiesce) [ ! -f "$root/service-up" ]; : > "$root/module-unloaded" ;;
    fpga-version)
        if [ -f "$root/version-override" ]; then
            cat "$root/version-override"
        else
            case "$(cat "$root/hardware")" in
                pointcloud) echo 0x11000001 ;;
                histogram) echo 0x11010002 ;;
                gray) echo 0x110300ff ;;
                spot) echo 0x110200a7 ;;
                *) exit 1 ;;
            esac
        fi
        ;;
    recover|prepare)
        case "$2" in 0) format=histogram ;; 1|2) format=pointcloud ;; 3) format=gray ;; 4) format=spot ;; *) exit 1 ;; esac
        [ "$(cat "$root/hardware")" = "$format" ]
        expected_mode=$2
        [ "$2" != 1 ] || expected_mode=2
        [ "${LIDAR_FPGA_ACTIVE_MODE:-}" = "$expected_mode" ]
        rm -f "$root/module-unloaded"
        ;;
    *) exit 2 ;;
esac
EOF
cat > "$root/mock-bin/service" <<'EOF'
#!/bin/sh
set -eu
root=$FPGA_TEST_ROOT
case "$1" in
    stop) rm -f "$root/service-up" ;;
    start) "$root/tools/lidar_boot" prepare "$root/boot.json"; : > "$root/service-up" ;;
    *) exit 2 ;;
esac
EOF
cat > "$root/mock-bin/sv" <<'EOF'
#!/bin/sh
set -eu
[ "$1" != status ] || exit 0
[ "$1" = -w ]
exec "$FPGA_TEST_ROOT/mock-bin/service" "$3"
EOF
cat > "$root/mock-bin/burner" <<'EOF'
#!/bin/sh
set -eu
root=$FPGA_TEST_ROOT
[ "$1" = program ] && [ -f "$root/module-unloaded" ] && [ ! -f "$root/service-up" ]
shift
if [ "${1:-}" = --progress-fd ]; then shift 2; fi
hash=$(sha256sum "$1")
[ "$(cat "$root/medium/lidar/state/fpga-profile")" = "pending ${hash%% *}" ]
printf 'partial\n' > "$root/hardware"
[ ! -e "$root/fail-write" ]
cp "$1" "$root/hardware"
[ ! -e "$root/fail-readback" ]
EOF
chmod 700 "$root/mock-bin/"*
cat > "$LIDAR_SERVICE_CONFIG" <<EOF
LIDAR_STORAGE_MODE=boot-copy
LIDAR_STORAGE_DEVICE=/dev/mock-lidar-test
LIDAR_STORAGE_MOUNT=$root/mount
LIDAR_STORAGE_MOUNTS=$root/mounts
LIDAR_RAM_DATA=$root/ram
LIDAR_RUN=$run
LIDAR_SCAN=$run/services
LIDAR_TEMPLATE=$root/service-template
LIDAR_RUNSV_DIR=/bin/true
LIDAR_LOGGER=/bin/true
LIDAR_SV=$root/mock-bin/sv
LIDAR_STOP_SECONDS=5
LIDAR_BOOT=$boot
LIDAR_BOOT_CONFIG=$root/boot.json
LIDAR_STORAGE_EXEC=$root/lidar-storage
LIDAR_SERVICE_EXEC=$root/mock-bin/service
LIDAR_PLATFORM_EXEC=$root/mock-bin/platform
LIDAR_FPGA_STAGE=$stage
EOF
cat > "$root/boot.json" <<EOF
{"model":"CS96-beta","fpga_probe":true,"data_root":"$ram","release_root":"$ram/releases","package_dir":"$ram/current",
"public_key":"$root/test-public.pem","openssl":"$root/tools/lidar_crypto","adapter":"$root/mock-bin/platform",
"boot_runtime":"$run","runtime_dir":"$run/manager","socket_path":"$run/control.sock",
"state_dir":"$ram/state","base_config":"$ram/config/ys_conf.ini","user_settings":"$ram/config/user-settings.json",
"calibration_dir":"$ram/calibration","initial_work_mode":2,"validated_work_modes":[1,2],
"disable_file_log":true,"verify_timeout_ms":1500,"settle_ms":100,"stop_timeout_ms":800,"adapter_timeout_ms":500}
EOF
[ "${1:-}" != setup ] || exit 0

stage_update() {
    "$root/fpga-mode-hook.sh" preinst
    printf '%s\n' "$1" > "$stage/target.bit"
    cp -p "$root/mock-bin/burner" "$stage/lattice-flash"
}
restart_from_flash() {
    previous=$(mktemp -d "$root/previous-ram.XXXXXX")
    mv "$root/ram" "$previous/ram"
    "$root/lidar-storage" start
}
pending() {
    expected=$(sha256sum "$stage/target.bit")
    [ "$(cat "$medium/state/fpga-profile")" = "pending ${expected%% *}" ]
    [ ! -e "$root/service-up" ]
    [ -f "$root/module-unloaded" ]
    [ ! -s "$root/mounts" ]
    if "$boot" prepare "$root/boot.json"; then exit 1; fi
    "$boot" package-path "$root/boot.json" >/dev/null
}
installed() {
    "$root/fpga-mode-hook.sh" postinst
    expected=$(sha256sum "$stage/target.bit")
    [ "$(cat "$medium/state/fpga-profile")" = "installed ${expected%% *}" ]
    [ ! -e "$root/service-up" ]
    [ -f "$root/module-unloaded" ]
    cmp "$root/package/conf/ys_conf.ini" "$ram/config/ys_conf.ini"
}

capture() (
    config=${2:-$root/boot.json}
    LIDAR_PERSIST_COMMAND=$root/lidar-storage "$boot" run "$config" &
    capture_pid=$!
    trap 'kill "$capture_pid" 2>/dev/null || :; wait "$capture_pid" || :' EXIT
    status=
    for attempt in 1 2 3 4 5; do
        status=$("$boot" status "$config" 2>/dev/null) || :
        if printf '%s\n' "$status" | grep -q '"phase":"COMMITTED"'; then break; fi
        sleep 1
    done
    printf '%s\n' "$status" | grep -q '"phase":"COMMITTED"'
    printf '%s\n' "$status" | grep -q "\"actual_mode\":$1,"
    if [ "$1" = 4 ]; then
        printf '%s\n' "$status" | grep -q '"initialized":true'
        printf '%s\n' "$status" | grep -q '"pipeline_verified":false'
    fi
    kill "$capture_pid"
    wait "$capture_pid"
    trap - EXIT
)

[ ! -e "$medium/state/fpga-profile" ] && [ ! -e "$medium/state/state.json" ]
printf 'histogram\n' > "$root/hardware"
"$boot" prepare "$root/boot.json"
grep -q '"initial_work_mode":0' "$run/manager.json"
printf 'pointcloud\n' > "$root/hardware"
stage_update gray
installed
[ ! -e "$medium/state/state.json" ]
restart_from_flash
capture 3
cp "$medium/state/state.json" "$root/confirmed"
echo '通过：首次 SWU 只烧录灰度，重启后按实读模式采集'

stage_update histogram
: > "$root/fail-write"
if "$root/fpga-mode-hook.sh" postinst; then exit 1; fi
pending
rm "$root/fail-write"
restart_from_flash
pending
stage_update histogram
installed
cmp "$root/confirmed" "$medium/state/state.json"
capture 0
echo '通过：写入失败后保持停采，重传成功后正常启动识别新布局'

stage_update gray
: > "$root/fail-readback"
if "$root/fpga-mode-hook.sh" postinst; then exit 1; fi
pending
rm "$root/fail-readback"
restart_from_flash
pending
stage_update pointcloud
installed
capture 2
grep -q '"validated_work_modes":\[1,2\]' "$run/manager.json"
echo '通过：回读失败保持停采，重启后改刷点云，正常启动支持模式 1/2'

stage_update spot
installed
mv "$ram/config" "$root/spot-saved-config"
mv "$ram/calibration" "$root/spot-saved-calibration"
rm -f "$run/manager/ys_conf.ini"
capture 4
grep -q '"validated_work_modes":\[4\]' "$run/manager.json"
grep -q '"confirmed_mode":4' "$medium/state/state.json"
[ ! -e "$run/manager/ys_conf.ini" ]
mv "$root/spot-saved-config" "$ram/config"
mv "$root/spot-saved-calibration" "$ram/calibration"
stage_update pointcloud
installed
capture 2
cp "$medium/state/state.json" "$root/confirmed"
service=$root/lidar-service
"$service" jtag-prepare
[ -f "$root/module-unloaded" ] && [ ! -e "$root/service-up" ]
if "$boot" prepare "$root/boot.json"; then exit 1; fi
cp "$medium/state/fpga-profile" "$root/installed-profile"
stage_update gray
if "$root/fpga-mode-hook.sh" postinst; then exit 1; fi
cmp "$root/installed-profile" "$medium/state/fpga-profile"
[ "$(cat "$root/hardware")" = pointcloud ]
printf 'gray\n' > "$root/hardware"
"$service" jtag-resume
grep -q '"validated_work_modes":\[3\]' "$run/manager.json"
grep -q '"state_dir":".*/fpga-jtag-state"' "$run/manager.json"
cmp "$root/installed-profile" "$medium/state/fpga-profile"
cmp "$root/confirmed" "$medium/state/state.json"

capture 3
cmp "$root/confirmed" "$medium/state/state.json"

for version in 0x11040000 0x12020000 0x11020100 0x11030100 0x00000000 0xffffffff invalid; do
    printf '%s\n' "$version" > "$root/version-override"
    if "$boot" prepare "$root/boot.json"; then exit 1; fi
done
rm "$root/version-override"
"$service" jtag-end
printf '2 %064d\n' 0 > "$ram/state/fpga-profile"
"$boot" prepare "$root/boot.json"
grep -q '"initial_work_mode":3' "$run/manager.json"
sed 's/"fpga_probe":true/"fpga_probe":true,"fpga_jtag_boot":true/' "$root/boot.json" > "$root/lab-boot.json"
"$boot" prepare "$root/lab-boot.json"
grep -q '"state_dir":".*/fpga-jtag-state"' "$run/manager.json"
echo '通过：光斑无需数据流即可就绪，SWU 和 JTAG 使用同一版本识别逻辑，不兼容版本保持停止'
