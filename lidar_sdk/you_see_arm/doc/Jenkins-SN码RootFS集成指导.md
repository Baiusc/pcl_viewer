# Jenkins：SN 写入工具集成 RootFS 指导

## 目标

Jenkins 生成的完整系统固件必须在 Buildroot RootFS 中包含出厂 SN 写入工具。设备烧录完整固件并启动 Linux 后，产线再依据 MES 为该台设备分配的 SN 执行写入。

通用固件只包含写入程序，不包含任何设备 SN，也不在开机脚本中自动写入。SN 必须由 MES 按设备分配，不能由 Jenkins、固件或雷达按时间生成。

SN 写入工具属于系统 RootFS，不属于第三分区签名应用包和普通应用升级包。当前 Beta 系统镜像整理脚本已经自动构建并纳入该工具；Jenkins 应优先调用现有整理流程，再将生成的 overlay 合入 Buildroot `TARGET_DIR`。

## SN 格式与设备端写入

实际 SN 是 19 个大写 ASCII 字符，不带分隔符：型号 7 位、硬件版本 3 位、组装地 1 位、生产年周 4 位、生产批号 4 位。例如：

```text
CS96XXXB01026020001
```

产线在设备首次启动后执行：

```sh
/usr/libexec/lidar/lidar_sn_provision <19位SN>
/usr/libexec/lidar/lidar-network-service restart
```

写入工具验证 SN 格式、MTD 分区标签和擦除块布局，只接受空分区、同一 SN 的重复执行或目标 SN 的未完成写入重试；成功后回读。Beta 当前约束为 `/dev/mtd3`、`/sys/class/mtd/mtd3/name=sn`，且分区大小必须等于一个擦除块。第二条命令让 `lidar_netd` 重新读取刚写入的 SN，并更新 Avahi 发布。

旧 UDP v1 广播的身份字段只有 18 字节，继续使用 MAC 身份；完整 SN 通过 Avahi TXT 记录和 `CMD_ID_GET_SYS_SN` 提供。不要截断 SN 填入旧字段。

## 涉及文件

| 文件 | 职责 | Jenkins 集成要求 |
| --- | --- | --- |
| `network/device_identity.c`、`network/network.h` | SN 格式校验、Flash 只读和运行时身份回退 | 构建网络守护进程与写入工具时使用同一套格式定义 |
| `network/lidar_sn_provision.c` | 出厂写入、分区检查、覆盖保护、写后回读 | 作为独立目标构建；不可作为守护进程或开机脚本自动运行 |
| `network/CMakeLists.txt` | 定义 `lidar_sn_provision` 显式构建目标 | 目标为 `EXCLUDE_FROM_ALL`，Jenkins 需显式构建 |
| `network/lidar_netd.c`、`network/network_support.c`、`network/network_discovery.c` | 运行时读取 SN 并由 Avahi 发布 | 设备重启网络服务后读取最新 SN |
| `scripts/prepare_beta_image_kit.py` | Beta 完整镜像整理入口 | 自动构建 `lidar_sn_provision`，并交给启动包整理流程 |
| `scripts/prepare_boot_bundle.py` | 生成系统 `rootfs-overlay` | `--sn-provisioner` 为必需输入；复制到 `usr/libexec/lidar/lidar_sn_provision`，记录 SHA256 |
| `scripts/integrate_lidar_rootfs.py` | 将 overlay 合入 Buildroot `TARGET_DIR` | 缺少 SN 写入工具时拒绝合入 |
| `scripts/beta_image_tools.py` | 检查 Beta 镜像交付目录 | 验证写入工具为目标 ELF，并核对报告中的 SHA256 |
| `deploy/runit/rootfs/usr/libexec/lidar/lidar-network-service` | 管理网络守护进程 | 提供 `restart`，供首次 SN 写入后刷新 Avahi 身份 |
| `ys_business/ys_business_sys.c`、`ys_business/ys_def.h` | 旧管理协议的 SN 查询应答 | 应答为 1 字节状态码加 19 字节 SN；不是 RootFS 写入入口 |
| `driver/src/yousee_lidar_api.c` | 源码驱动 API | `SN_set` 已禁用，写入只经独立出厂工具 |

## Jenkins 流程

### 1. 整理完整 Beta 镜像交付包

调用现有完整镜像入口。它会构建签名应用包、单独构建 `lidar_sn_provision`，再生成带 RootFS overlay 和校验报告的镜像交付目录：

```sh
python3 scripts/prepare_beta_image_kit.py \
  --output <新建的镜像交付目录> \
  --sign-key <受控签名私钥>
```

Jenkins 应保存该交付目录和外层归档；归档中的 `SHA256SUMS`、`kit-report.json` 以及 `overlay/usr/libexec/lidar/lidar_sn_provision` 必须一并保留。SN 写入工具不进入 `third-partition/lidar/current` 的签名清单。

如果 Jenkins 使用基础启动包流程而非完整镜像入口，先分别构建应用包和写入工具：

```sh
python3 scripts/build.py package beta --output <应用包目录> --sign-key <受控签名私钥>
cmake --build build/beta --target lidar_sn_provision

python3 scripts/prepare_boot_bundle.py \
  --application <已签名应用包目录> \
  --public-key <发布公钥> \
  --kernel-module <配套内核模块> \
  --sn-provisioner build/beta/bin/lidar_sn_provision \
  --motor-init <Beta电机程序> \
  --cgi-dir <管理CGI目录> \
  --output <新建的启动包目录>
```

### 2. 合入 Buildroot 并生成系统镜像

将交付目录放到 Jenkins 管理的 Buildroot 工作区，保留系统同事已有的 overlay 和 post-build hooks，并将本交付包的 overlay 与 `tools/post-build.sh` 接入 Buildroot。Beta 交付流程使用以下配置：

```text
BR2_ROOTFS_OVERLAY="<交付目录>/overlay"
BR2_ROOTFS_POST_BUILD_SCRIPT="<交付目录>/tools/post-build.sh"
```

若已有多个 overlay 或 post-build 脚本，应在原配置上追加并按依赖顺序调用，不能覆盖原设置。`tools/post-build.sh` 会先执行 `beta_image_tools.py check`，再调用 `integrate_lidar_rootfs.py`；它只修改离线 `TARGET_DIR`。手工执行集成脚本时可用：

```sh
python3 scripts/integrate_lidar_rootfs.py \
  <Buildroot输出目录>/target \
  --overlay <镜像交付目录>/overlay \
  --replace-legacy-startup
```

随后由 Jenkins 使用项目现有 Buildroot 配置生成完整系统镜像。生成的 RootFS 必须包含可执行文件：

```text
<TARGET_DIR>/usr/libexec/lidar/lidar_sn_provision
```

网络服务启用时，集成脚本还会读取同一 Buildroot 输出目录下 `build/avahi-*/config.h`，以 `HAVE_DBUS` 的实际编译值设置 `avahi_dbus_supported`。若编译目录不在默认位置，手工集成命令需增加 `--avahi-config-header <Avahi构建目录>/config.h`。目标镜像应在专用板验证 Avahi 正常启动，`mdns_running=true` 保持超过 60 秒，且服务名可解析。

Jenkins 应保存完整系统镜像、应用包、集成报告和校验清单。集成报告中的 `sn_provisioner_sha256` 必须与 RootFS 内程序一致。

## 构建验收

Jenkins 至少检查以下事项：

1. RootFS 文件存在且有执行权限，ELF 架构与目标设备一致。
2. 镜像集成报告包含 `sn_provisioner_sha256`，并与实际文件摘要一致。
3. 缺少 `--sn-provisioner` 时，基础启动包生成失败；overlay 缺少程序时，RootFS 合入失败。
4. SN 写入器不出现在第三分区应用包的签名文件清单中；升级应用包不会覆盖第四分区。
5. 通用系统镜像中不包含设备 SN，不会在启动阶段自动运行写入器。
6. 产线任务在设备启动 Linux 后从 MES 获取本机 SN，执行写入，再重启网络服务并核对 Avahi TXT 中的完整 SN。

当前写入器按 Beta 的 `/dev/mtd3` 和 `sn` 分区标签保护。Jenkins 不应将该工具用于未核实分区布局的其他机型；扩展到其他机型前，先确认 MTD 节点、标签、分区大小和擦除块布局。
