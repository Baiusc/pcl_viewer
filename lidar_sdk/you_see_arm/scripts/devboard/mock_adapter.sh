#!/bin/sh
# 开发板专用，不调用 insmod、rmmod、DMA 或电机接口。
set -eu
case "${1:-}" in prepare|recover) ;; *) exit 2 ;; esac
case "${2:-}" in 1|2) ;; *) exit 1 ;; esac
case "${3:-}" in -1|1|2) ;; *) exit 1 ;; esac
echo "【开发板模拟适配】$1：$3 -> $2；未操作任何硬件"
