# Beta 开机集成与持久化

产品交付边界见[系统镜像与应用独立更新](../../doc/系统镜像与应用独立更新交付边界-20261106.md)。2026-11-06 新增完整 Beta 镜像交付入口 `scripts/prepare_beta_image_kit.py`，业务 Web/CGI 已纳入第三分区签名版本，验签后复制到 RAM，系统保留独立诊断页；可生成实际 `user.jffs2`，集成步骤见[交付流程](../beta-image/集成流程.md)。下文单独调用旧 `prepare_boot_bundle.py` 的 `/www` 随镜像布局仍为兼容入口。现有 `flash-seed` 仅为首次初始化资产，不是在线升级器。

本阶段已经实现离线集成代码，保持 BusyBox init，启用 `runsv/runsvdir/sv`。221 尚未更换镜像或修改 Flash；独立开发板 30 已完成 RAM 测试，238 不在测试范围。

2026-11-05 已在独立开发板 10.42.0.30 验证 RAM 运行链路，保留环境供人工操作，见[开发板人工验证手册](../../doc/开发板30-RAM模式管理人工验证.md)。开发板为 TFTP/RAM 系统，无真实雷达采集，不作为断电持久化或硬件验收。

## 已交付的行为

- 系统预置的 `lidar_boot` 先使用外部公钥、预置 `lidar_crypto` 校验包签名及全部文件，再执行包内 mgrd。包目录固定到验证过的版本真实路径。
- `runsvdir → runsv → lidar_boot → lidar_mgrd → you_see`。启动工具通过 exec 变成 mgrd，监督器跟踪的主进程 PID 不变。
- 开机读取 Flash 上的最后确认模式，初始化基准配置和本机标定时只补齐 RAM 中缺失的文件。启动和切换的中间状态写入 RAM。
- 回波、坐标设置保存到独立用户参数文件；工作模式仍由 mgrd 事务决定。握手目标地址和接收端口不持久化。
- stop 等待管理与采集退出，保持服务 down；正常维护不会被重新拉起。连续短时间退出三次后停止自动重启，显式 start 才重置进程层重试计数，不清理业务 FAULT。
- 模式状态记录 Linux boot_id。同一次开机内保留异常恢复次数限制。产品镜像将切换成功的 COMMITTED 和新发生的硬件或采集 FAULT 写入 Flash。仍兼容旧版持久状态的保守恢复行为。
- 管理 API 可访问即表示服务就绪；业务 FAULT 保持可查询，不被监督器误当成进程失败。采集成功仍按现有 HTTP 契约判定。
- 管理与采集输出通过 runit 日志服务转入已有 syslog；托管运行配置取消应用自行追加文件日志，避免无限增长。无需新增 svlogd。

## Beta FPGA 格式识别与 JTAG 调试

系统镜像的 Beta 启动配置启用 `fpga_probe`。启动器通过 SPI 读取 `version_num`（`0x3008`），检查项目编号 B1=`0x11`、布局版本 B3=`0x00`。B4 为常规迭代号。功能版本 B2 决定启动模式。

| B2 | 功能 | `work_mode` | 电机 |
| --- | --- | --- | --- |
| `0x00` | 点云、原始点云 | `2`、`1` | 启动 |
| `0x01` | 原始直方图 | `0` | 停转 |
| `0x02` | 光斑 | `4` | 启动 |
| `0x03` | 灰度图（576×168） | `3` | 停转 |

SWU 和 JTAG 刷写后，启动器按实读功能配置。上次确认的模式与当前 FPGA 兼容时继续沿用，否则按新版本重新验证。光斑模式由 FPGA 版本选择，签名应用包须声明 `spot` 能力。平台启动电机后，`you_see` 仅通过 SPI 读取并记录 `0x3000` 至 `0x3010` 的五个版本寄存器。光斑模式不加载采集模块，不启动 DMA 和业务数据传输。

通过 JTAG 调试时，在板端 SSH 执行 `/usr/libexec/lidar/lidar-service jtag-prepare`，待服务停采并卸载模块后用 IDE 烧录。随后执行 `/usr/libexec/lidar/lidar-service jtag-resume`，板端按实读模式启动。用 `lidar-service status` 查看实际模式，`COMMITTED` 表示采集模式通过出数检查或光斑模式通过版本寄存器校验。调试状态位于 `/run`。需要退出调试或取消烧录时执行 `jtag-end`，服务保持停止。

若调试板通过 JTAG 写入 FPGA 配置 Flash 并在开机时从 Flash 加载，开机按实读模式启动。在可信的 `/etc/lidar/boot.json` 设置 `"fpga_jtag_boot": true` 可让开机后的采集状态只保存在 RAM。通过 JTAG 直接下载到易失配置时，使用 SSH 恢复采集即可保留当前位流。

## 镜像同事现在需要准备的内容

1. BusyBox 保留现有 init、Shell、logger、syslogd、挂载和驱动工具，并新增 `CONFIG_RUNSV=y`、`CONFIG_RUNSVDIR=y`、`CONFIG_SV=y`。保存项目实际使用的 BusyBox 配置文件。
2. 保留独立网络启动流程与 SSH；当前老镜像的一部分联网和 SSH 启用原来藏在 app_startup.sh 中，移除 S99user 前必须把它们纳入系统服务。这里不实现 SN 推导 IP 或 Link Local。
3. PTP 保持一个明确的服务管理入口，禁止再通过 Flash install.sh 覆盖系统启动脚本。
4. 提供当前内核配套的 Beta 点云 `.ko`。电机程序由本仓库 [platform/beta/motor](../../platform/beta/motor/README.md) 独立编译，Beta 构建产物为 `build/beta/platform/bin/demo_i2c_motor`。模块必须带 `.note.gnu.build-id`，集成工具从 ELF 中计算与 sysfs 相同的标识摘要；不能把另一台设备或另一个内核版本当作已验证配套。
5. 系统镜像提供完整 `/www`：lighttpd、模块及依赖库、配置、网页。也可以在生成集成包时通过 `--www-root` 提供它。Web 使用独立 `S90lidar_web`，不要再同时保留其他 lighttpd 启动入口。
6. 当前 CGI 和 mgrd 沿用同 UID 运行，与已验收的 221 配置一致；CGI 配置 `/etc/lidar/http-bridge.json`、UDS 0600。不用改上位机 HTTP 接口。
7. 确认 syslog 已配置容量限制和轮转，日志位于 RAM，不持续写满 NOR Flash。`service.conf` 中的工具路径必须与镜像实际安装位置一致。

## 目录与启动顺序

```text
/etc/lidar/
  service.conf                  可信系统服务路径与固定硬件配套
  boot.json                     启动与管理配置
  release-public.pem            系统预置的发布公钥
  http-bridge.json               CGI 的 UDS 路径
/usr/libexec/lidar/
  lidar_boot、lidar_crypto       系统预置的信任入口
  lidar_sn_provision             出厂 SN 写入工具，首次启动后由产线显式调用
  lidar-service、lidar-storage、lidar-platform-beta
  platform/                     指定内核模块、电机初始化程序
/usr/share/lidar/runit/lidar_mgrd/
  run、check、finish、log/run    只读服务模板
/mnt/mtdblock2/lidar/
  releases/<版本>/              签名包，使用期间禁止原地覆盖
  current -> releases/<版本>
  config/ys_conf.ini             本机基准配置
  config/user-settings.json      用户参数，首次修改时创建
  calibration/                  本机标定，独立于发布版本
  state/state.json               最后确认模式与恢复事务
/run/lidar/                      管理配置、UDS、崩溃计数
/run/lidar-services/lidar_mgrd/   实际监督目录、控制 FIFO 和日志服务
/run/lidar-platform/             本轮冷启动的硬件初始化标记
```

Beta 产品默认使用 `LIDAR_STORAGE_MODE=boot-copy`。`S35lidar_storage` 以只读方式挂载 JFFS2，将当前版本、配置和标定复制到 `/run/lidar-data`，卸载成功后才启用 RAM 副本。`S90lidar_web` 从 RAM 验签并启动 Web，`S98lidar` 准备服务。rcS 返回后，runit 从 RAM 拉起管理与采集。手工停止、启动 Web 或管理服务继续使用本次开机的 RAM 副本。

确需保存时，`lidar-storage save` 串行执行读写挂载、临时文件同步、原子替换和卸载。正常启动、握手、模式切换的中间状态和未确认的改址只操作 RAM。保存失败向调用方返回错误。历史部署仍可用 `managed` 管理常驻挂载，`directory` 使用现有目录。此前要求设备常驻读写挂载的开发板 Flash 验收脚本对应旧版 `managed` 环境。

首次迁移优先使用新版目录中的本机配置与标定，缺少的文件从旧 `you_see_arm/config` 副本导入 RAM。新出厂镜像直接提供本机标定，`/etc/ys_calibration.ini` 和 `/etc/ys_encoder_angle.csv` 指向 RAM。维护时可用 `lidar-storage save base-config <文件>` 和 `lidar-storage save encoder-angle <文件>` 写入已核对的本机文件，整机下次启动时加载。

## 构建与生成离线集成包

先构建统一应用包；私钥始终留在构建主机包外：

```bash
python3 scripts/build.py package beta \
  --output /tmp/beta-application \
  --sign-key /可信发布目录/private.pem
```

正式发布额外使用 `--release`，要求干净来源与准确的版本标签；开发阶段的签名包仍标识 release=false，不冒充正式版本。

两个 CGI 从已实现的 Web 子模块构建：

```bash
make -C ../you_see_www/you_see_cgi/src/lidar_management \
  CC=/home/huyidao/workspace/you_see_arm/cc/bin/aarch64-buildroot-linux-gnu-gcc \
  BIN_DIR=/tmp/beta-management-cgi
```

手工生成启动包时，先单独构建只用于系统镜像的 SN 工具；完整 Beta 镜像整理脚本会自动执行这一步：

```bash
cmake --build build/beta --target lidar_sn_provision
```

使用已确认的硬件资产和信任公钥生成新输出目录：

```bash
python3 scripts/prepare_boot_bundle.py \
  --application /tmp/beta-application \
  --public-key /可信发布目录/public.pem \
  --kernel-module /系统发布目录/yousee_lidar.ko \
  --sn-provisioner build/beta/bin/lidar_sn_provision \
  --motor-init build/beta/platform/bin/demo_i2c_motor \
  --cgi-dir /tmp/beta-management-cgi \
  --www-root /系统发布目录/www \
  --output /tmp/beta-boot-bundle
```

输出包括 rootfs-overlay、flash-seed.tar.gz、inittab.fragment、集成脚本和 integration-report.json。工具先验证应用包；清单和签名不被重写。Flash 初始化归档统一使用 root 属主，并保持配置、状态目录私有。

`rootfs-overlay/usr/libexec/lidar/lidar_sn_provision` 随系统镜像部署。设备首次启动 Linux 后，产线根据 MES 分配结果显式执行写入和网络服务重启：

```bash
/usr/libexec/lidar/lidar_sn_provision CS96XXXB01026020001
/usr/libexec/lidar/lidar-network-service restart
```

第二步让 `lidar_netd` 重新读取 SN 并更新 Avahi 发布；写入命令不由启动脚本自动调用，也不进入第三分区应用更新包。

如果目标镜像已有完整 `/www`，可以省略 `--www-root`。工具会删除旧 `switch_raw_mode.cgi/switch_point_mode.cgi` 后备路径，旧页面按钮需要按既有指导接入新 API。

在 Buildroot 主机的 TARGET_DIR 合入，不能对运行中的系统根目录执行：

```bash
python3 /tmp/beta-boot-bundle/integrate_lidar_rootfs.py \
  /Buildroot工作目录/output/target \
  --overlay /tmp/beta-boot-bundle/rootfs-overlay \
  --replace-legacy-startup
```

集成脚本保留原 inittab 的串口、系统初始化和关机项，仅追加一条 BusyBox 格式的 respawn 入口。旧 S99user 移到 `/usr/share/lidar/legacy/` 备份，不再参与 rcS。标定默认文件也保留离线备份。脚本拒绝目标父目录链接越出 TARGET_DIR，避免写入构建主机的真实根目录。

同样的命令可纳入 Buildroot post-build hook；应把 overlay 及脚本保存到项目维护路径，不能长期依赖本例的 /tmp。不要同时再启动旧 app_startup.sh，不要直接照搬 SysV 的 inittab 写法。

flash-seed 归档只用于首次受控初始化、新建 lidar 目录。它不是在线升级器；禁止将归档直接覆盖到已有 lidar 部署，更不能覆盖 current 后继续运行旧服务。旧 histogram/point_cloud 等目录暂不删除，待新镜像启动与恢复验收完成后再清理。

## Beta 平台适配

点云（2）、原始点云（1）和光斑（4）在首次进入时启动电机，两个点云模式之间切换时复用已启动的电机。原始直方图（0）和灰度图（3）在采集启动前执行停转并确认读回零转速。FPGA 烧录和 JTAG 维护也会在卸载模块前确认停转，下次进入需要转动的模式时重新启动电机。

平台适配器加载指定 `.ko` 并验证模块构建标识。如果模块由旧入口预先加载、模块标识不符、设备节点缺失或仍被占用，则拒绝继续启动。板级命令经可信工具限时执行，适配器退出时不遗留其直接初始化子进程。

这里的 recover 沿用 221 固定点云布局测试策略，**不会停止或复位 DMA**。当前驱动缺失真实停采接口的问题仍然存在，不能把软件测试当作硬件异常恢复认证。跨直方图/灰度、强杀后的硬件可靠性和正式停采仍需驱动配套及实机验收。

## 主机验证

```bash
python3 scripts/test_lidar_mgrd.py
python3 scripts/test_runtime_config_path.py
LIDAR_TEST_RUNIT_BIN=/真实工具目录/bin python3 scripts/test_boot_integration.py
```

集成测试使用真实 lidar_boot、真实 mgrd、签名夹具，以及真实 runsv/runsvdir/sv；采集和板级命令为隔离模拟。没有监督工具时明确跳过监督测试，不用自写假监督器代替。本次开发在 /tmp 单独编译了 BusyBox 1.36.1 的这三个 applet 进行验证，不修改主机系统或雷达。

覆盖签名前拒绝执行、机型/链接/路径错误、配置与标定不覆盖、模式跨进程恢复、RAM 运行目录丢失后从持久参数恢复、真实监督启停与单实例、mgrd 崩溃重启、三次失败熔断、FAULT 保持可查询、离线合入幂等及防止旧启动入口双实例。用户参数回归直接编译真实业务 setter，验证落盘失败时不改变内存参数、重复设置不重写文件。

## 新镜像到位后的实机验收

1. 先检查镜像中的三个 applet、可信工具、公钥、硬件资产和完整 Web 资源，确认旧 S99user 已移出启动目录。
2. 在约定的测试设备完成首次初始化，保存旧目录与分区备份，不操作 238。
3. 冷启动观察模块、电机、管理 API 和数据流水线就绪，确认唯一 mgrd 与采集进程。
4. 通过现有上位机执行 2→1→2，重新握手并验证数据；验证回波/坐标参数跨模式和重启保留。
5. 验证正常服务 stop 不被重拉、start 不产生双实例、重启恢复最后确认模式、损坏包保留诊断能力。
6. 受控验证 JFFS2 状态提交和断电恢复。主机测试无法代替真实断电和硬件停采验证。

本次交付是软件与离线集成能力；221 仍使用旧操作系统，新启动链尚未在实机启用。
