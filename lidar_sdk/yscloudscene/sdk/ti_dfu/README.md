# TI USB DFU 烧录 SDK

将本目录交给使用 C++17、CMake 3.20+ 和 Qt 5/6 的调用方。在宿主 CMake 中加入目录并链接所需目标。

```cmake
add_subdirectory(ti_dfu)
target_link_libraries(your_app PRIVATE TiDfu::Widgets)
```

界面入口为 `ti_dfu::FlashDialog`（`flash_dialog.h`）。只需接入业务逻辑时，链接 `TiDfu::Core` 并使用 `ti_dfu::Flasher`（`flasher.h`）。在 Qt 事件循环所在线程调用 `Start(Options)`，让对象存活到 `Completed` 发出。信号提供阶段、当前阶段的传输百分比（`-1` 表示等待设备）、工具输出和最终结果。已在执行时再次调用 `Start` 会返回 `false`。

运行时选择 dfu-util 0.11 可执行文件、匹配板卡的 boot 目录和镜像。使用动态版 dfu-util 时，将随包提供的 DLL 保留在可执行文件旁。Windows 需要可识别各阶段 USB DFU 接口的驱动。每次只连接一台待烧录设备，当前模块使用 TI VID/PID `0451:6165`。烧录与阶段判断仅通过 USB DFU 完成，不需要连接串口。

- 专用两阶段更新先向 `bootloader` 发送 `tiboot3.bin`。同时检测到 `alt=0 tispl.bin` 和 `alt=1 u-boot.img` 后，才向 alt 0 发送所选 `update.img`。工厂 NOR 镜像应使用 Flash Writer 流程。
- Flash Writer 依次加载 `tiboot3.bin`、`tispl.bin` 和 `u-boot.img`，检测到 `flash` 接口后才发送所选 Flash 镜像。引导镜像或 U-Boot 启动配置必须提供板卡对应的 Flash DFU 服务。若最后一次 USB 枚举没有 `flash`，用 `dfu-util -l` 核对实际接口，并检查 USB 连接、DFU 驱动及 boot 文件与设备固件是否匹配。

SDK 以 dfu-util 的传输完成信息和退出码报告结果。设备重启后的固件版本需按设备流程验证。两种流程均未添加独立的擦除命令。
