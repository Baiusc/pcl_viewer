# lidar_mgrd 设备管理服务

2026-11-05 已新增 [BusyBox/runit 开机集成](../deploy/runit/README.md)：可信启动验包、离线镜像合入、固定持久目录、用户参数保存和真实监督测试。新镜像仍在编译，未在 221 启用新启动链。

独立开发板 10.42.0.30 已完成 RAM 模拟采集的真实 init/runit/HTTP 实测，保留运行环境供人工验证，见[逐步操作手册](../doc/开发板30-RAM模式管理人工验证.md)。无真实雷达硬件，不作为断电持久化或 DMA 验收。

设备状态更新（2026-11-04）：上位机模式切换已测试通过。221 已重启，此前 RAM 临时部署已消失，当前运行旧启动链的原采集程序。下一阶段的实机依据与启动方案见 [221 Flash 核查与产品化方案](../doc/设备启动-221-Flash核查与产品化方案-20261104.md)。下文实机测试记录保留其发生时的状态。

## 当前交付与边界

原生 C 服务独立于采集应用运行，通过 Unix `SOCK_SEQPACKET` 套接字接收 JSON 请求。提供模式查询、受控切换、采集进程监督和可恢复状态。原生命令行客户端为 `lidar_mgrctl`，不依赖 Python；网页后端已经通过同一协议提供 CGI。

Beta/Gamma 构建生成并安装管理进程、命令行客户端、轻量验签工具及可信启动工具。独立主机构建仅用于模拟验证：

```bash
cmake -S manager -B build/manager-host -G Ninja
cmake --build build/manager-host
python3 scripts/test_lidar_mgrd.py
python3 scripts/test_management_app.py
```

管理进程当前不提供自动升级、冷重启或 Flash 挂载。挂载和开机入口由独立系统集成层负责，尚待新镜像实机验收。`upgrade` 等未实现操作返回 501。238 属于其他同事使用的设备，不在本轮部署或测试范围。

## 生产启动前提

参见 `config.example.json`。配置和包外公钥必须来自可信部署环境，不能由网页请求指定或替换。当前机型从可信配置读取并与签名包比对；未假设 SN 分区已经包含机型。

- 正常启动必须验证 `manifest.sig`，再遍历校验全部包内文件的大小和 SHA256；拒绝额外文件、符号链接和机型不匹配。
- 原生服务通过配置中的 `openssl` 路径调用摘要/验签程序，参数固定且不经过 shell。可使用 OpenSSL 命令，也可使用新增的 `lidar_crypto`，后者动态加载设备已有的 libcrypto 1.1/3，只提供本服务所需的 SHA256 与 RSA/ECDSA 验签，不实现密码算法。缺少对应库或接口时拒绝校验。验证工具及信任公钥必须通过可信方式预置，不能只信任包内自带副本。
- `--simulation` 显式跳过签名验证，但仍检查包内摘要，并在所有状态响应标识 `simulation=true`。它仅供隔离主机测试，不能加入设备自启动配置。
- 包目录在使用期间应保持只读、可信，不得原地覆盖；服务启动和每次用户切换前重新校验包。
- `validated_work_modes` 是设备能力与实机验证结果的白名单，还要与包内 `supported_modes` 取交集。示例保持空列表，不能直接启动采集。
- 持久化目录和运行目录由本服务独占，要求属主为当前用户、无其他用户写权限。父目录由可信系统配置预先建立。
- 初始工作模式必须位于白名单中。采集模式要求初始化和数据验证通过。Beta 光斑模式（4）限启动器实读识别并获签名包支持时启用，以版本寄存器校验作为就绪条件。

```bash
lidar_mgrd /etc/lidar/manager.json
lidar_mgrctl /run/lidar/control.sock status
lidar_mgrctl /run/lidar/control.sock switch 0
```

进程前台运行，由 BusyBox init 等系统监督；本轮没有修改设备启动项。

## 平台适配接口

配置中的 `adapter` 是受信任的绝对路径，服务以以下参数调用：

```text
platform-adapter prepare 目标work_mode 最后确认work_mode
platform-adapter recover 目标work_mode 最后确认work_mode
```

`prepare` 在应用正常退出后执行；首次启动、管理服务重启恢复、采集进程被强杀或回滚时使用 `recover`。适配器必须同步完成必要的停采复位、FPGA/传感器配置、内核模块操作和硬件格式确认，成功才返回 0，失败返回非 0。不得后台启动其他进程，不得自行重启设备。

SIGKILL 只能说明进程停止，不证明 DMA 已停止。因此强制停机后的后续操作必须经过 `recover`，不能直接加载新采集应用。适配器超时后进入故障状态，不并发启动另一个重配任务。底层必须冷重启的平台当前保持拒绝切换，待实机设计后实现受控重启流程。

## 请求协议

一条连接处理一个 JSON 数据包，长度小于 1024 字节。套接字权限为 0600，并用 `SO_PEERCRED` 限制为与服务相同的用户；网页应通过同用户的本地转发组件访问，暂不扩大套接字权限。

```json
{"op":"status"}
```

```json
{"op":"switch","work_mode":0}
```

响应包含 `code`、`model`、`version`、`operation_id`、`phase`、`confirmed_mode`、`actual_mode`、`desired_mode`、`pid`、`error`、包内能力、验证白名单及健康信息。这里的 code 是 JSON 字段，HTTP CGI 按它设置真实 HTTP 状态。

- 200：查询或已经处于目标模式。
- 202：切换已受理，继续查询直到 COMMITTED 或 FAULT。
- 409：已有操作进行中。
- 422：模式未验证、未包含或软件包校验失败。
- 500：意图落盘失败，保留原采集进程。
- 503：服务故障，拒绝继续切换。
- 501：本阶段未实现的操作。

## 切换与恢复

内存状态为 `STOPPING → PREPARING → VERIFYING → COMMITTED`，失败时进入恢复流程或 FAULT。`confirmed_mode` 始终表示最后验证成功的模式；切换期间 `actual_mode=-1`，不会把旧模式误报为正在正常采集。

在停止应用前写入 PENDING；恢复前写入 RECOVERING；验证成功后写入 COMMITTED。状态采用临时文件、文件 fsync、rename、父目录 fsync；所有错误都处理为失败。JFFS2 真正的断电行为仍需板端验证，主机进程中断测试不能代替断电测试。

采集应用接收 SIGTERM，超时后强杀，硬件重新初始化由适配器承担。每次异常最多自动恢复一次，恢复失败或恢复途中再次重启则进入 FAULT。正常停止守护进程会优雅停采并记录 STOPPED。

数据验证超时进入 FAULT 时会记录 `fault_code=verification_timeout`。下次 Linux 开机允许重新验证一次，有已确认且仍受支持的模式时沿用该模式，从未确认过模式时使用 `initial_work_mode`。验证成功后用 `COMMITTED` 更新 Flash 记录。同一次开机内仍保留故障和重试限制。其他故障及未记录原因的旧版 `FAULT` 需要人工诊断及受控恢复。

## 应用协作

管理服务为每个应用实例创建私有 socketpair 和随机 token，以环境变量传入 `LIDAR_MGR_FD`、`LIDAR_MGR_TOKEN`、`YS_CONFIG_PATH`、`LIDAR_WORK_MODE`。应用从独立运行期配置读取模式，握手、回波和坐标参数写回同一路径，不覆盖设备原始主配置。不会再通过 `check_system()` 杀掉其他同名进程；受管理时旧业务协议的 reboot 请求返回失败。标定操作仍沿用原有接口和文件，不在本轮测试范围。

启动配置可额外指定 `user_settings`，管理服务以 `YS_USER_SETTINGS_PATH` 传给应用。只持久保存回波和坐标字段，写入采用临时文件、fsync、rename、父目录 fsync；相同设置不重复写入。应用确认落盘成功后才更新内存并回复成功。工作模式和握手目标不放入用户参数文件。生成运行配置时应用该白名单，损坏设置拒绝启动，不能静默回到默认值。

顶层 SIGTERM/SIGINT 处理函数只设置退出标志，主线程停止控制线程、停采、回收数据线程，再释放驱动和配置。原有工作线程内部退出机制仍使用项目已有信号方式，其真实退出时延和 SDK 行为需要实机确认。

事件循环就绪发送 INITIALIZED；消费者连续观察到 10 个符合当前包结构检查的流水线数据包后发送 PIPELINE_PROGRESS。健康消息必须同时匹配本实例私有通道、token 和工作模式，再经过稳定观察时间才提交。

Beta 光斑模式读取并核对 FPGA 版本寄存器后发送 `INITIALIZED`，经过同样的稳定观察时间即可提交。状态中的 `pipeline_verified` 为 `false`，实际模式由 `actual_mode=4` 表示。

特别注意：当前计数单位是数据包，不是完整帧。ARM 生成的外层头不能证明 FPGA 原始格式正确，也不能证明数据数值或整帧完整。因此必须结合经过实机验证的平台适配器；状态字段 `pipeline_verified` 只表示流水线有结构有效的数据。后续应在各机型适配器中继续完善帧级及硬件格式验证，不能仅凭此通知开放未经验证的模式。

当前监督能检测启动超时、子进程退出及适配器超时；尚未实现提交后持续的数据心跳看门狗。

## 主机验证

`test_lidar_mgrd.py` 启动真实服务并使用临时模拟采集程序，覆盖互斥、模式白名单、数据超时、过期 token、仅初始化不提交、失败回滚、强杀恢复、恢复次数上限、进程中断重启、包篡改、落盘失败、独占锁、正式签名启动及状态损坏。

`test_management_app.py` 编译真实应用管理模块，验证连续计数被无效包清零、私有健康消息及 SIGTERM 退出标志。测试不会接触雷达、Flash 或系统服务。

## 221 首轮只读核对

2026-11-03 已核对 10.42.0.221：设备配置标识为 CS96-beta，AM62 平台，Linux 6.1.83；现有应用使用点云 work_mode=2，日志持续出现四面处理记录。Flash 用户分区未挂载，没有上传或替换程序、停止应用、切模式、重启或刷写。读取前后应用、驱动库及主配置 SHA256 一致。238 未连接。

设备有 `/usr/lib/libcrypto.so.1.1`，但未找到 OpenSSL 可执行程序。后续需确认该库的接口与交叉构建依赖，不能通过 simulation 绕过正常验签路径。

源码检查发现 am6204 的 `sys_workmode_set()` 直接返回 0，未执行硬件操作。Beta/Gamma 非 LOCAL_TEST 构建现在会在退出时回收线程并明确返回失败，通知管理服务走 recover，不能谎报硬件已停采。真实 DMA 停止、复位和配套格式确认仍是平台适配器的实施前提。

## 221 固定点云测试包

使用 `python3 scripts/prepare_beta_221_test.py /tmp/新的输出目录` 生成开发测试包，测试私钥不会纳入上传文件。上传目录为 `/tmp/beta-test`，不替换系统文件；`scripts/beta_221/test.sh` 提供 check、start、status、switch 1/2、stop、restore 和 logs。start 新建独立测试会话，restore 重新启动原 /root/you_see。

该测试适配器仅检查已核对的点云内核模块构建标识、无其他设备句柄占用，并允许相同 DMA 布局下重新打开应用；不执行也不宣称 DMA 复位，不用于其他型号或直方图/灰度切换。221 已验证轻量工具使用其 libcrypto.so.1.1 完成签名校验，并通过真实 HTTP 完成点云 2→1→2 切换，每次约 3.2 秒。当前保留模式 2 采集和管理服务运行，供上位机联调。

## 上位机与网页接入

完整的客户端开发、CGI 桥接契约、状态显示和联调验收说明见 [设备模式切换：上位机与网页开发指导](../doc/设备模式切换-上位机与网页开发指导.md)。HTTP CGI 已安装到 221 的 RAM 根文件系统，真实 lighttpd 与管理服务转发通过。部署结果与恢复命令见 [221 HTTP 联调记录](../doc/设备模式切换-221-HTTP联调记录-20261103.md)。
