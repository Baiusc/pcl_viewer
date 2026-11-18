#!/usr/bin/env bash
# 雷达网络准备与连通性检查（需要 sudo）
#
# 背景：雷达单播发往 192.168.137.100，因此 Linux 宿主若要直接收点云，
# 必须占用 .100；若 .100 已被同事电脑占用，请先协调释放，否则会产生 IP 冲突。
#
# 用法:
#   bash tools/connect_radar.sh [网卡名]
#   IFACE=enp4s0 HOST_IP=192.168.137.100 RADAR_IP=192.168.137.200 bash tools/connect_radar.sh

set -u

IFACE="${1:-${IFACE:-enp4s0}}"
RADAR_IP="${RADAR_IP:-192.168.137.200}"
HOST_IP="${HOST_IP:-192.168.137.100}"

echo "[1/4] 为 ${IFACE} 追加 ${HOST_IP}/24"
sudo ip addr add "${HOST_IP}/24" dev "${IFACE}" 2>/dev/null || echo "  地址已存在，跳过"
sudo ip link set "${IFACE}" up

echo "[2/4] 路由检查"
ip route get "${RADAR_IP}"

echo "[3/4] 连通与冲突检查"
if ping -c 2 -W 1 "${RADAR_IP}" >/dev/null 2>&1; then
    echo "  雷达 ${RADAR_IP} 可达"
else
    echo "  雷达不可达：检查网线、雷达是否上电、是否已启动 fsLAA.sh"
fi
arp -n "${RADAR_IP}" 2>/dev/null || true
echo "  注意：若 arp 显示 .100 的 MAC 不是本机，说明存在 IP 冲突"

echo "[4/4] 抓包 3 秒，确认 UDP 点云包"
sudo timeout 3 tcpdump -i "${IFACE}" -nn "host ${RADAR_IP} and udp" 2>/dev/null | head -20 || true

echo
echo "完成。启动 cloud_viewer 后点击 Connect Radar；"
echo "若雷达不在 ${RADAR_IP}，用 CV_RADAR_IP 覆盖："
echo "  CV_RADAR_IP=${RADAR_IP} ./build/cloud_viewer"
