#!/usr/bin/env bash
# 一次性配置 cloud_viewer 的雷达自动连接能力：
#   1. 安装 /usr/local/bin/cv-radar-net（配置雷达网段地址）
#   2. 写入 sudoers 免密规则（仅允许上述助手脚本）
#   3. 写入 ~/.config/cloud_viewer/radar.conf（雷达 SSH 凭据，权限 600）
#
# 用法（需要 sudo）：
#   sudo bash tools/setup_radar_autoconnect.sh [iface] [cidr] [radar_host] [radar_user] [radar_pass]
# 省略 radar_pass 时会交互式输入。
set -euo pipefail

IFACE="${1:-enp4s0}"
CIDR="${2:-192.168.137.100/24}"
RADAR_HOST="${3:-192.168.137.200}"
RADAR_USER="${4:-root}"
RADAR_PASS="${5:-}"

if [[ $EUID -ne 0 ]]; then
    echo "请使用 sudo 运行：sudo bash $0 ..." >&2
    exit 1
fi

TARGET_USER="${SUDO_USER:-$(id -un)}"
TARGET_GROUP="$(id -gn "$TARGET_USER")"
TARGET_HOME="$(getent passwd "$TARGET_USER" | cut -d: -f6)"
if [[ -z "$TARGET_HOME" ]]; then
    echo "无法解析用户 $TARGET_USER 的 HOME" >&2
    exit 1
fi

# 1) 网段配置助手：固定命令、root 所有，避免给用户开放任意 sudo。
install -d -m 0755 /usr/local/bin
cat > /usr/local/bin/cv-radar-net <<'EOF'
#!/bin/sh
set -e
IFACE="${1:-enp4s0}"
CIDR="${2:-192.168.137.100/24}"
ip addr replace "$CIDR" dev "$IFACE"
ip link set "$IFACE" up
EOF
chown root:root /usr/local/bin/cv-radar-net
chmod 0755 /usr/local/bin/cv-radar-net

# 2) sudoers 免密：只允许执行该助手脚本。
SUDOERS_FILE=/etc/sudoers.d/cloud-viewer-radar-net
cat > "$SUDOERS_FILE" <<EOF
$TARGET_USER ALL=(root) NOPASSWD: /usr/local/bin/cv-radar-net
EOF
chmod 0440 "$SUDOERS_FILE"
if command -v visudo >/dev/null 2>&1; then
    visudo -cf "$SUDOERS_FILE" >/dev/null
fi

# 3) 雷达 SSH 凭据（供自动拉起驱动使用）。
if [[ -z "$RADAR_PASS" ]]; then
    read -r -s -p "请输入雷达 root SSH 密码: " RADAR_PASS
    echo
fi
install -d -m 0700 -o "$TARGET_USER" -g "$TARGET_GROUP" "$TARGET_HOME/.config/cloud_viewer"
CONF_FILE="$TARGET_HOME/.config/cloud_viewer/radar.conf"
cat > "$CONF_FILE" <<EOF
RADAR_SSH_HOST=$RADAR_HOST
RADAR_SSH_USER=$RADAR_USER
RADAR_SSH_PASS=$RADAR_PASS
RADAR_SCRIPT=/root/Linux_pcl/fsLAA.sh
RADAR_WORKDIR=/root/Linux_pcl
EOF
chown "$TARGET_USER":"$TARGET_GROUP" "$CONF_FILE"
chmod 0600 "$CONF_FILE"

echo "安装完成："
echo "  网段助手 : /usr/local/bin/cv-radar-net ($IFACE, $CIDR)"
echo "  sudoers  : $SUDOERS_FILE"
echo "  SSH 配置 : $CONF_FILE"
echo "之后 cloud_viewer 连接时会自动补齐网段，并在无数据时自动 SSH 拉起雷达驱动。"
