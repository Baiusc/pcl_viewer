# Beta FPGA 固件更新包

整机固件包交付系统和 ARM 应用。FPGA 配置 Flash 由独立的签名 FPGA SWU 更新。

## 板端安装

使用启用了 `fpga_probe` 的 Beta 系统镜像。镜像须包含新版 `lidar_boot`、`lidar-platform-beta`、`lidar-service` 和统一 DMA 模块，应用包须包含新版 `lidar_mgrd`，SWUpdate 须启用上传和验签。启动采集时校验签名应用对实读布局和驱动 ABI `2` 的支持。

安装脚本停采，在擦写前记录镜像摘要，再卸载模块并调用烧录器。擦写、回读及 FPGA 重载完成便返回 SWU 成功，采集保持停止。启动采集服务或重启设备后，板端与 JTAG 使用同一条启动路径，通过 SPI 读取 `version_num`（`0x3008`），以实际运行的 FPGA 固件确定采集布局。[正常启动和 JTAG 流程](../runit/README.md#beta-fpga-格式识别与-jtag-调试)

Linux 用户分区的 `lidar/state/fpga-profile` 记录 `pending <镜像摘要>` 或 `installed <镜像摘要>`。烧录或回读失败时保留 `pending`，重启后仍阻止采集。SWUpdate 的 8080 端口继续提供恢复入口，可重新上传同一包或其他 Beta 固件包。

## Jenkins 打包

使用已固定提交的 Lattice 烧录器归档和发布方确认的 FPGA 镜像。构造供 `archive` 条目解包的 `fpga-mode.tar.gz`，根目录包含以下两个文件。

| 文件 | 内容 |
|---|---|
| `target.bit` | Beta 板型的 FPGA 镜像 |
| `lattice-flash` | 具备回读校验的 AArch64 CLI，归档内权限保留执行位 |

用 [sw-description.in](sw-description.in) 生成最终 `sw-description`，替换发布版本、压缩归档和安装脚本的 SHA-256。使用现有 CMS 流程签名和验包。组包顺序为 `sw-description`、`sw-description.sig`、`fpga-mode.tar.gz`、`fpga-mode-hook.sh`。归档最终 `.swu`、镜像摘要和烧录器制品哈希。

## 软件测试

```sh
python3 scripts/test_fpga_mode_update.py
python3 scripts/test_fpga_mode_update.py --swupdate /path/to/swupdate
python3 scripts/build.py build beta
python3 scripts/test_fpga_mode_update.py --board-tree /tmp/lidar-fpga-test-build
```

传入具备 CMS、archive 和 Web 支持的宿主 SWUpdate 可运行本机 8080 的签名上传测试。将板端测试树传到独立的 `/tmp` 目录，以 root 属主解包后执行 `test_fpga_mode_update.sh`。软件测试使用真实的启动器、管理进程和存储脚本，以模拟器替代烧录、采集和挂载。实机采集仍由正常启动流程验证有效数据。
