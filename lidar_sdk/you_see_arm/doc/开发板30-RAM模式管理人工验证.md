# 开发板 30：RAM 模式管理人工验证

2026-11-05 已在 10.42.0.30 实测。当前环境保留运行，正常状态为模拟模式 2。

这是 TFTP 启动的 RAM 系统，没用 eMMC 或 Flash。整板断电、重启后测试目录会消失；本文验证“重启服务后保留状态”，不把它当作“掉电后持久保存”。本轮未操作 221、238，原 SWUpdate 的 8080 服务继续运行。

## 先认识三个对象

```text
BusyBox init
  └─ runsvdir：管理测试服务目录
       ├─ runsv：监督 lidar_mgrd
       │    └─ lidar_mgrd：负责切换、恢复、监督采集
       │         └─ you_see：本次为模拟采集程序
       └─ runsv：监督测试 Web 服务（8081）
```

管理进程、监督器、签名校验和 HTTP CGI 都是真实程序；只有采集源和硬件适配器是模拟的，不初始化电机、不加载雷达驱动、不产生真实点云。状态中的 `model=DEVBOARD-MOCK` 是明确的测试标识。`simulation=false` 只表示没有跳过签名验证，不表示存在真实雷达硬件。

所有操作在开发板上执行，先登录：

```sh
ssh root@10.42.0.30
cd /tmp/lidar-board-test
```

密码沿用已有测试凭据。下面每次修改后等待 2～3 秒，再执行 status，不需要安装 jq 或 Python。

## 1. 看当前状态

```sh
sh test.sh status
```

预期前两行服务状态都是 `run:`，JSON 中为：

```text
model             DEVBOARD-MOCK
phase             COMMITTED
actual_mode       2
confirmed_mode    2
error             空字符串
initialized       true
pipeline_verified true
```

这里 `COMMITTED` 表示已经完成本轮模拟数据验证。模式 1/2 是模拟运行标签，不代表开发板产生了真实点云。

注意两种 PID：`run: .../lidar_mgrd: (pid ...)` 是管理进程；JSON 的 `pid` 是采集子进程。切换模式通常只改变后者。

## 2. 亲手切到模式 1

```sh
sh test.sh switch 1
sleep 2
sh test.sh status
```

切换请求先返回 `code=202`，只表示受理。等待后应看到 `phase=COMMITTED`、`actual_mode=1`、`confirmed_mode=1`、`error=""`。采集 PID 改变，管理 PID 保持不变。

管理服务做的是：停模拟采集 → 生成模式 1 的运行配置 → 重新启动采集 → 收到私有健康通知 → 确认模式。没有改名模式目录或复制驱动文件。

## 3. 正常停止：观察它不会自动重启

```sh
sh test.sh stop
sleep 2
sh test.sh status
sh test.sh state
```

预期管理服务显示 `down:`，HTTP 服务仍为 `run:`。管理 JSON 暂时不可查询；最后的状态文件为 `phase=STOPPED`、`confirmed_mode=1`。

这是正常维护，runit 已被告知“保持停止”，所以不会自动拉起。不要用反复 kill 代替正常 stop。

## 4. 再启动：观察它记住模式 1

```sh
sh test.sh start
sleep 2
sh test.sh status
```

预期恢复为模式 1，而不是默认模式 2。这证明：管理进程虽然重启，RAM 中的状态文件仍然存在并被重新读取。

切回模式 2，准备下一项：

```sh
sh test.sh switch 2
sleep 2
sh test.sh status
```

## 5. 模拟管理程序崩溃：观察它被自动拉起

```sh
sh test.sh crash-manager
sleep 3
sh test.sh status
```

预期管理 PID、采集 PID 都改变，最后仍是模式 2、COMMITTED。这次没有告诉 runit 停止服务，所以它把异常退出的管理进程重新拉起。管理进程消失时，其旧采集子进程也会被终止，随后由新实例恢复。

此命令只结束经过路径核对的本测试管理进程，不是 killall。不要在真实雷达上照搬测试故障注入。连续重复异常可能消耗恢复预算并进入 FAULT，这是预期保护。

## 6. 可选：体验一次切换失败后的自动回滚

先确保处于模式 2；按以下顺序设置“模式 1 没有数据”的模拟标记：

```sh
sh test.sh fail-mode1
sh test.sh switch 1
sleep 5
sh test.sh status
```

预期结果是 `actual_mode=2`、`confirmed_mode=2`、`phase=COMMITTED`，但 `error` 包含超时原因。这表示“目标模式 1 失败，已恢复模式 2”，不能仅看到 COMMITTED 就说目标成功。

恢复正常，再完成一次成功切换：

```sh
sh test.sh clear-failure
sh test.sh switch 1
sleep 2
sh test.sh switch 2
sleep 2
sh test.sh status
```

## 7. 可选：体验签名拦截与启动熔断

只改测试目录中的模拟应用，备份位于包外；先正常停止，避免修改正在执行的文件：

```sh
sh test.sh stop
sh test.sh tamper
sh test.sh start
```

这次 start 应失败，可能等待约十几秒。随后：

```sh
sh test.sh status
sh test.sh logs
cat run/failures
```

预期管理服务保持停止，日志显示验签/摘要失败，连续三次启动失败后熔断，failures 为 3。此时 Web 仍运行，查询 API 返回 503；模拟采集未启动，原模式状态不被清空。

恢复原包：

```sh
sh test.sh restore-package
sh test.sh start
sleep 2
sh test.sh status
```

预期回到原有正常模式。手工 start 只重置进程层的启动失败计数，不清理业务 FAULT。

## 8. HTTP 从电脑或浏览器验证

浏览器直接打开：

```text
http://10.42.0.30:8081/cgi-bin/lidar_status.cgi
```

电脑上的命令行也可以执行：

```sh
curl http://10.42.0.30:8081/cgi-bin/lidar_status.cgi
curl -H 'Content-Type: application/json' \
  -d '{"work_mode":1}' http://10.42.0.30:8081/cgi-bin/lidar_switch.cgi
```

上位机管理地址需要包含 **8081**。8080 是原 SWUpdate 服务，两个服务互不替代。新模式生效以继续查询得到的最终状态为准，开发板不提供真实点云数据连接。

## 9. 查看文件与退出测试

```sh
sh test.sh state
sh test.sh logs
cat run/manager/ys_conf.ini
ps | grep -E 'runsv|lidar|lighttpd'
```

这里可以把“最后确认模式”“运行配置”“实际进程”对应起来。开发板时间尚未校准，日志中的 1970 年日期不作为耗时依据。

只想暂停采集管理，执行 stop；想结束本轮全部测试服务，执行：

```sh
sh test.sh cleanup
```

cleanup 会移除本轮追加的 inittab 入口并停止两个测试服务，保留目录和日志。不会覆盖其他人随后增加的启动项。

如果故意反复注入故障导致 FAULT，可在本模拟环境中执行下列重置。它会先备份测试状态，再开始全新的模式 2 演示；这不是生产故障解除接口：

```sh
sh test.sh stop
sh test.sh reset-test
sh test.sh start
```

整板重启后本目录会消失。重新部署时，把开发机生成的 `lidar-board-test.tar.gz` 解压到 `/tmp`，执行 setup。不要为保留本轮演示而刷写 eMMC 或 Flash。

## 已完成的开发板实测

已通过：真实签名启动、HTTP 模式 2→1→2、并发 409、未开放模式 422、stop 后不重启、start 保留模式、管理进程崩溃后自动重拉、模拟数据超时回滚、篡改包拒绝启动、三次失败熔断及恢复原包。最终保留模式 2 正常运行。

原 inittab 所有内容保留，原 SWUpdate 8080 服务继续运行。测试原始响应位于开发机 `/tmp/ys-devboard-30-20261105/acceptance/`。本轮没有验证整板断电持久化、JFFS2 或真实雷达硬件行为。
