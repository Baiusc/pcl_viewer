#!/bin/bash
set -e

# FL6532 (MF150-fl6532) 板端 GDB 调试部署脚本
# 用法: ./scripts/remote-debug-fl6532.sh [目标IP]
# 默认目标 IP: 10.42.0.44

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
TARGET_IP="${1:-10.42.0.44}"
PASSWORD='111'
MODEL=MF150-fl6532
PRESET=fl6532-pointcloud-debug
GDB_PORT=1234
REMOTE_DIR=/root
SDK_LIB=third_party/fs_sdk/bsp/rockchip/rv1126b/sdk_libs/lib_FL6x32_algo.so

echo ">>> Building for ${MODEL} (debug, -O0 -g)..."
if [ "${SKIP_BUILD:-0}" != "1" ]; then
    python3 "${REPO_ROOT}/scripts/build.py" build "${PRESET}"
fi

echo ">>> Syncing program to target ${TARGET_IP}..."
sshpass -p "${PASSWORD}" scp -o StrictHostKeyChecking=no \
    "${REPO_ROOT}/build/${PRESET}/bin/you_see" root@${TARGET_IP}:${REMOTE_DIR}/you_see

echo ">>> Syncing dynamic library to ${TARGET_IP}..."
sshpass -p "${PASSWORD}" ssh -o StrictHostKeyChecking=no root@${TARGET_IP} \
    "mkdir -p ${REMOTE_DIR}/lib"
sshpass -p "${PASSWORD}" scp -o StrictHostKeyChecking=no \
    "${REPO_ROOT}/${SDK_LIB}" root@${TARGET_IP}:${REMOTE_DIR}/lib/

echo ">>> Starting gdbserver (multi mode) on ${TARGET_IP}:${GDB_PORT}..."
sshpass -p "${PASSWORD}" ssh -o StrictHostKeyChecking=no root@${TARGET_IP} \
    "killall gdbserver you_see 2>/dev/null" || true
sshpass -p "${PASSWORD}" ssh -o StrictHostKeyChecking=no -f root@${TARGET_IP} \
    "gdbserver --multi :${GDB_PORT}"
sleep 2

echo ">>> gdbserver started on ${TARGET_IP}:${GDB_PORT}"
cat <<EOF

本地 GDB 连接指引:
  aarch64-linux-gnu-gdb ${REPO_ROOT}/build/${PRESET}/bin/you_see
  (gdb) target extended-remote ${TARGET_IP}:${GDB_PORT}
  (gdb) set remote exec-file ${REMOTE_DIR}/you_see
  (gdb) break main
  (gdb) run
EOF
