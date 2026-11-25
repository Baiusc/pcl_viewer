# Beta 电机启停程序

`demo_i2c_motor` 是执行单次启停操作的板级程序。源码从原 `dist/demo_i2c_motor.c` 迁入本目录，由 Git 和 CMake 管理。`dist` 仅保存生成的交付物。

程序不依赖 `you_see`、算法库或 `liblidar.so`，保留原可执行文件名。当前只用于 Beta 已有硬件配套，不能直接套用到其他机型。

## 编译

在仓库根目录执行，使用现有 Beta 工具链：

```bash
cmake --preset beta
cmake --build --preset beta --target demo_i2c_motor
```

产物为 `build/beta/platform/bin/demo_i2c_motor`。正常执行 `python3 scripts/build.py build beta` 也会构建它；Gamma 和其他机型不增加此目标。

只编译电机程序时，也可以独立配置本目录，不配置采集应用和子模块：

```bash
cmake -S platform/beta/motor -B build/beta-motor -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_TOOLCHAIN_FILE="$PWD/aarch64-toolchain.cmake" \
  -DYS_TOOLCHAIN_ROOT="$PWD/cc" \
  -DYS_TOOLCHAIN_PREFIX=aarch64-buildroot-linux-gnu
cmake --build build/beta-motor
```

独立构建产物为 `build/beta-motor/platform/bin/demo_i2c_motor`。Buildroot 同事可以使用实际配套的交叉工具链文件替换上述参数，无需修改源码。

## 镜像集成

设备安装位置为 `/usr/libexec/lidar/platform/demo_i2c_motor`，与现有 `service.conf` 的 `LIDAR_MOTOR` 保持一致。`lidar-platform-beta` 在采集准备阶段按目标模式调用，在 FPGA 烧录或 JTAG 维护卸载采集模块前确认电机已停转。平台脚本与支持 `stop` 的电机程序须一同装入系统镜像。

它作为系统镜像中的平台组件交付。普通 `cmake --install` 和 `scripts/build.py package beta` 不安装它；需要显式选择组件，避免混入第三分区应用升级包：

```bash
cmake --install build/beta --component lidar_platform \
  --prefix "$PWD/build/beta-motor-overlay"
```

输出为 `build/beta-motor-overlay/usr/libexec/lidar/platform/demo_i2c_motor`。独立构建也支持相同安装命令，将构建目录换为 `build/beta-motor` 即可。

`scripts/prepare_beta_image_kit.py` 默认采用本次 Beta 构建的电机程序，放入交付包的 `overlay/usr/libexec/lidar/platform/`，记录路径与 SHA256。显式指定 `--motor-init` 时，提供已确认配套且支持 `stop` 的二进制。

单独使用 `scripts/prepare_boot_bundle.py` 时，传入：

```text
--sn-provisioner build/beta/bin/lidar_sn_provision
--motor-init build/beta/platform/bin/demo_i2c_motor
```

## 硬件行为与退出状态

点云模式（2）、原始点云模式（1）和光斑模式（4）沿用原启动参数和执行顺序。

1. 打开 `/dev/i2c-0`，I2C 地址为 `0x00`。
2. 依次写寄存器 `0x04=0x03`、`0x02=0x4a`、`0x03=0x40`。
3. 每秒读取 `0x44/0x45` 计算转速，最多十次；只有 `140 < speed < 160` 才视为达标。
4. 通过 sysfs 导出 GPIO 374，设置输出，输出约 1 ms 高电平脉冲并恢复低电平。

原始直方图模式（0）和灰度图模式（3）执行 `stop`，将转速设定值写零并保持 I2C 覆盖控制。最多采样十次，读回零转速后返回成功。GPIO 374 就绪脉冲在电机启动达标后发送。

成功返回 0。设备打开、I2C 设置、寄存器读写、GPIO 操作失败，以及十次采样仍未达到目标状态，均返回非零。程序退出时关闭已打开的 I2C 句柄，GPIO 已导出时允许复用。

配套平台脚本的电机执行上限由原来的 5 秒调整为 12 秒，避免提前打断十次采样；整个适配器上限由 12 秒调整为 20 秒，容纳电机和原有 5 秒模块加载。阻塞由外层 `lidar_boot bounded` 和管理服务超时处理。

## 验证范围

```bash
python3 scripts/test_beta_motor.py
```

主机测试编译真实源码，在链接时替换 I2C、GPIO 和等待调用，检查寄存器顺序、转速判断、错误退出及资源释放。实机验收需要确认零设定值能使电机停转并保持静止，以及切回点云后能重新达到目标转速并发出就绪脉冲。
