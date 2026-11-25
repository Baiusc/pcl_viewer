#ifndef YS_FS_SDK_BOOT_H
#define YS_FS_SDK_BOOT_H

/**
 * fs_sdk(FL6532) 业务侧启动适配
 * 由 core 层调用，替代 fs_sdk examples 中写死 PCL 的启动流程，
 * 使 SDK 工作模式可以由业务配置(work_mode)决定。
 */

/**
 * @brief 初始化并启动 fs_sdk
 * @param biz_work_mode 业务工作模式(common.h 中的 RAW_MODE/TEST_POINT_CLOUD_MODE/FORMAL_POINT_CLOUD_MODE)
 *                      TEST_POINT_CLOUD_MODE(=1, 原始数据) -> fs_sdk RANGE 模式(输出 0x2E)
 *                      其它                              -> fs_sdk PCL 模式(输出 0x21)
 * @return RET_OK 成功, 否则失败
 */
int fs_sdk_boot_init(int biz_work_mode);

/**
 * @brief 反初始化 fs_sdk
 */
void fs_sdk_boot_exit(void);

#endif /* YS_FS_SDK_BOOT_H */
