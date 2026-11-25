# CS96 Beta 新版 ARM 固件（lidar_mgrd）适配记录

> 实测日期：2026-11-25　设备：`169.254.7.30`（SN `CS96000B01026470003`，CS96-beta，96×800）
> 发布包：`/run/lidar-data/lidar/releases/20261124T074036Z-737ad66a5722`
> 内核：Linux 6.1.83 aarch64

## 1. 结论先说

新版 ARM 包把 `you_see` 交给 **lidar_mgrd**（runsv 守护）管理，上位机的操作方式整体换了。
旧代码里的三处硬编码在新固件上**全部失效**：

| 旧代码假设 | 新固件实际 | 后果 |
|---|---|---|
| `/etc/ys_conf.ini` | 不存在；基准配置在 `/run/lidar-data/lidar/config/ys_conf.ini` | 读配置直接 `CONFIG_INVALID` |
| `/root/you_see` + `pidof you_see` | 程序在 `/run/lidar-data/lidar/releases/*/bin/you_see`，由 mgrd 拉起 | 手工 kill 会与守护进程抢进程，pidof 校验也不可信 |
| 改 INI 的 `work_mode` 再重启 | mgrd 每次拉起 `you_see` 都按自身状态**重写**运行期配置 | 改 INI 的 `work_mode` 无效 |

新固件另有三个可用入口（本次已实机验证）：

| 操作 | 入口 |
|---|---|
| 查状态 | `GET /cgi-bin/lidar_status.cgi` → JSON |
| 切 work_mode | `POST /cgi-bin/lidar_switch.cgi`，body 只允许 `{"work_mode":N}` |
| 重启 | `/usr/libexec/lidar/lidar-service restart`（封装 mgrd），`status` 看 phase |

## 2. 路径与命令（实测）

```text
基准配置        /run/lidar-data/lidar/config/ys_conf.ini      ← 改这里
运行期配置      /run/lidar/manager/ys_conf.ini                ← mgrd 每次拉起 you_see 重新生成
标定            /run/lidar-data/lidar/calibration/ys_calibration.ini
                /etc/ys_calibration.ini 是指向它的软链接（读旧路径仍然可用）
程序            /run/lidar-data/lidar/releases/<ver>/bin/you_see
管理命令        /usr/libexec/lidar/lidar-service {status|restart|...}
模式命令        /run/lidar-data/lidar/current/bin/lidar_mgrctl /run/lidar/control.sock status|switch N
mgrd 描述       /run/lidar/manager.json
持久状态        /run/lidar-data/lidar/state/state.json
```

`lidar-service status` 的真实输出（`run:` 行是 runsv，随后一行是 JSON）：

```json
{"code":200,"message":"状态","model":"CS96-beta","phase":"COMMITTED",
 "confirmed_mode":2,"actual_mode":2,"desired_mode":2,"error":"","pid":1353,
 "initialized":true,"pipeline_verified":true,
 "supported_modes":["histogram","pointcloud","gray","spot"],
 "validated_work_modes":[1,2]}
```

要点：

- **`validated_work_modes` 才是该发布包允许的 work_mode**（本次为 `[1,2]`）。
  `supported_modes` 是平台能力名（还有 histogram/gray/spot），不能直接当 work_mode 用。
- `actual_mode=-1` 表示正在停采/重启，此时不能判成功。
- 切模式返回 `202`（已受理），必须等到 `phase=COMMITTED && actual_mode=N`。

## 3. 模式切换实测

```bash
# 切原始点云
curl -X POST -H 'Content-Type: application/json' \
     -d '{"work_mode":1}' http://<ip>/cgi-bin/lidar_switch.cgi
# -> 202 {"phase":"STOPPING","actual_mode":-1,"desired_mode":1,...}
# 数秒后
curl -s http://<ip>/cgi-bin/lidar_status.cgi
# -> 200 {"phase":"COMMITTED","confirmed_mode":1,"actual_mode":1,"pid":5952,...}
```

- body 多写任何字段都会被拒：`{"code":400,"message":"请求体必须只包含整数 work_mode（0～4）"}`。
- 切换会停采并重启 `you_see`（`pid` 变化），UDP 会中断几秒，随后按新包长继续推流。
- 实测包长：`work_mode=1` → 1561B，`work_mode=2` → 1266B，与解析器常量一致。
- 旧的 `/cgi-bin/switch_point_mode.cgi`、`switch_raw_mode.cgi` 已改为返回 501「此旧入口已停用」。

## 4. 推流目标（center_ip）怎么改才生效

`/run/lidar/manager/ys_conf.ini` 是 mgrd **生成**的，直接改它没用。流程必须是：

```bash
vi /run/lidar-data/lidar/config/ys_conf.ini     # 改 [network] center_ip / center_data_port
/usr/libexec/lidar/lidar-service restart
/usr/libexec/lidar/lidar-service status          # 等到 phase=COMMITTED
grep -E 'center_ip|center_data_port' /run/lidar/manager/ys_conf.ini   # 校验运行期配置
```

实测一个容易误判的现象：

- 连上设备时运行期配置里 `center_ip=169.254.222.6`（本机），但基准配置是出厂值 `192.168.110.15:50027`；
- 只要发生一次 mgrd 重启（切模式也算），运行期配置就**回落到基准配置**，雷达立刻改发往出厂地址。

所以：**判断推流目标是否生效，必须看 `/run/lidar/manager/ys_conf.ini`，不能看基准配置，更不能只看「文件写成功了」。**

## 5. cloud_viewer 的适配方式

`platform/LidarServiceClient` 负责 HTTP 管理接口（跨平台 winsock/POSIX，无 curl 依赖），
`ViewerApp` 按「设备是否暴露 lidar_status.cgi」自动分流：

- 连接时先探测状态：有 JSON 就用 `actual_mode` 初始化当前模式，用
  `validated_work_modes` 生成可选模式位图（新固件没有 `/mnt/mtdblock2/mode`）。
- `Mode` 按钮 / `CV_BETA_SET_MODE`：新固件走 HTTP switch 并轮询到 COMMITTED，**不需要 SSH 密码**；
  旧固件保持「改 INI + 重启 you_see」。
- `Beta Target`：新固件改基准配置 → `lidar-service restart` → 轮询运行期配置校验，
  超时自动回滚原配置并再 restart 一次。
- `Restart you_see`：新固件走 `lidar-service restart` 并用 mgrctl 确认 COMMITTED；
  没配 SSH 密码时退化为「把当前模式再下发一次」，同样能让 mgrd 重拉 you_see。

环境变量 `CV_BETA_HTTP_PORT` 可覆盖设备 HTTP 端口（默认 80，调试用）。

## 6. 控制台中文乱码

MSVC 下源码按 `/utf-8` 编译，但 Windows 控制台默认按本地代码页（GBK）解释输出，
`std::cout` 的中文字面量会显示成乱码。`cloud_viewer_main.cpp` 在 `main` 入口调用
`SetConsoleOutputCP(CP_UTF8)` + `SetConsoleCP(CP_UTF8)`（仅 `_WIN32`）。
重定向到文件/管道时仍是 UTF-8 字节，不改变原有行为。

## 7. 断电重启实测（20261125）：配置不持久化

- **`/run` 是 tmpfs，配置每次开机都重置。** 挂载表里只有 `rootfs`（ramfs）和 `/run`
  （tmpfs），`/mnt/mtdblock2`（jffs2）以 **`boot-copy`** 模式只读挂进 `/run/lidar-data`
  （`/etc/lidar/service.conf` 的 `LIDAR_STORAGE_MODE=boot-copy`），`lidar-storage` 脚本里
  没有 commit 回写路径。所以 `center_ip` / `center_data_port` 改完只在本次开机有效，
  **重启后回落镜像默认值**（本机实测 `192.168.110.15:50027`）。
- 结论：**每次上电后连设备都必须重写一遍 center_ip**，这一步只能靠 SSH；因此
  `betaSshPassword()` 在没有 `CV_BETA_SSH_PASS` / `beta_ssh.conf` 时直接用出厂 `111`，
  不再退化成空值。
- 重启后实测链路：`phase=COMMITTED / mode=2` → 核对目标并
  `CENTER_UPDATED=192.168.110.15:50027-169.254.222.6:50007` → `Radar Live 76800 点`。
- **sshd 比 web 慢**：上电后 80 端口先可用，22 端口十几秒后才 listen，
  期间 `ensureSshReady()` 会轮询，日志区分「已就绪（等待设备启动完成）」和「web 可用但 22 未监听」。
- **`set_ssh.cgi` / `reboot.cgi` 在新固件上已停用**，返回
  `{"code":501,"msg":"此旧入口已停用，请使用配套管理接口。"}` / `501 Not Implemented`。
  网页端「服务配置」页仍在，但后端同样走不通，不要指望 HTTP 拉起 sshd 或重启。
- **sshd 主机密钥每次重启都变**（`ssh-ed25519 SHA256:...` 随机器密钥重生成），
  TOFU 记录会命中 `hostKeyMismatch`，程序自动更新信任并重试一次，日志打 `[HOST-KEY]`。
- **`lidar-service` 反复重启会进 `phase=FAULT`**（`actual_mode=-1`、`pid=0`、
  `error=上次恢复未完成、状态损坏或恢复次数耗尽`）。FAULT 下改配置既不重启 you_see，
  也不会重写运行期配置，`Beta Target` 会报 `TARGET_APPLY_FAILED` 并回滚。恢复手段是断电重启
  （状态文件在 tmpfs，重启即清零）。
- 169.254 直连链路本身会间歇抖动（实测 80/22 同时掉 2 秒又恢复），HTTP 探测与 SSH 都需要重试：
  连接时的固件探测改为 3 次重试；老路径报 `CONFIG_MISSING` 时会自动改走 mgrd 路径再试一次。

## 8. 仍未验证 / 下一步

- **心跳**：厂商客户端（`YSCloudScene`）先向雷达 `65000` 发 16B 握手包（含本机 IP、
  command_port、data_port、mode），雷达回 ACK 到 command_port，之后周期性发心跳；
  断开时日志出现 `command_stopped=true`。本次 `cloud_viewer` **只收不发**，实测连续推流
  十几分钟未中断，因此暂未实现握手。若要彻底摆脱 SSH/配置改写（也就摆脱上电后必须改
  center_ip 这一步），需要抓这 16B 包并补齐心跳——这是最有价值的下一个方向。
- **网页终端**：设备 `/www` 带 `web_terminal` 页面（xterm.js），若其 WebSocket 可脚本化，
  可作为 SSH 之外的第二条恢复通道；本次未接入。
