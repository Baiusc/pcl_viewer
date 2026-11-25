#!/bin/bash
set -e

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PRESET=fl6031-pointcloud-debug
python3 "${REPO_ROOT}/scripts/build.py" build "${PRESET}"

echo ">>> Syncing program to target..."
sshpass -p '111' scp -o StrictHostKeyChecking=no "${REPO_ROOT}/build/${PRESET}/bin/you_see" root@10.42.0.35:/root/you_see

echo ">>> Starting gdbserver (multi mode)..."
sshpass -p '111' ssh -o StrictHostKeyChecking=no root@10.42.0.35 'killall gdbserver you_see 2>/dev/null' || true
sshpass -p '111' ssh -o StrictHostKeyChecking=no -f root@10.42.0.35 'gdbserver --multi :1234'
sleep 2

echo ">>> gdbserver started on 10.42.0.35:1234"
