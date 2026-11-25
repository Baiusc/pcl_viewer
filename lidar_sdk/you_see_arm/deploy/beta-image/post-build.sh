#!/bin/sh
# 由 Buildroot 调用，只修改离线 TARGET_DIR；不执行烧录或连接设备。
set -eu
[ "$#" -ge 1 ] || { echo '缺少 Buildroot TARGET_DIR' >&2; exit 2; }
tools=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
kit=$(dirname -- "$tools")
python3 "$tools/beta_image_tools.py" check "$kit"
python3 "$tools/integrate_lidar_rootfs.py" "$1" --overlay "$kit/overlay" --replace-legacy-startup
for name in usr/bin/runsv usr/bin/runsvdir usr/bin/sv usr/bin/logger usr/bin/fuser sbin/insmod; do
    [ -e "$1/$name" ] || [ -L "$1/$name" ] || { echo "系统缺少 /$name，请核对 BusyBox 配置及安装路径" >&2; exit 1; }
done
if [ ! -e "$1/lib/libcrypto.so.1.1" ] && [ ! -e "$1/usr/lib/libcrypto.so.1.1" ] &&
   [ ! -e "$1/lib/libcrypto.so.3" ] && [ ! -e "$1/usr/lib/libcrypto.so.3" ]; then
    echo '系统缺少 lidar_crypto 需要的 libcrypto.so.1.1 或 libcrypto.so.3' >&2
    exit 1
fi
for library in libstdc++.so.6 libgcc_s.so.1 libc.so.6 libm.so.6 ld-linux-aarch64.so.1; do
    found=no
    for folder in lib usr/lib lib64 usr/lib64; do
        if [ -e "$1/$folder/$library" ] || [ -L "$1/$folder/$library" ]; then found=yes; fi
    done
    [ "$found" = yes ] || { echo "应用缺少动态依赖 $library，请核对 Buildroot 运行库" >&2; exit 1; }
done
echo 'Beta 离线集成完成；请继续生成系统镜像，并按 README 核对 DFU 与本机标定。'
