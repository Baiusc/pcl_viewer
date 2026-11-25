/*
 * fs_sdk(FL6532) 业务侧启动适配
 * 说明: 原实现直接调用 third_party/fs_sdk/examples 的 examples_init(),
 *       其内部固定 fs_sdk_setup_mode(FS_SDK_WORK_MODE_PCL)。
 *       本文件复刻该启动序列, 但 SDK 工作模式由业务 work_mode 决定:
 *         - work_mode=1(原始点云模式) -> FS_SDK_WORK_MODE_RANGE, 输出 Range(0x2E) 原始数据
 *         - 其它                      -> FS_SDK_WORK_MODE_PCL,   保持原有行为
 * 仅 MF150-fl6532 机型编译(其余机型为空编译单元)。
 */

#if MF150_FL6532 == 1

#include "ys_fs_sdk_boot.h"
#include "ys_base.h"
#include "ys_conf.h"

#include "../common/common.h"

#include "fs_sdk_api.h"
#include "fspc_support.h"
#include "fs_err.h"

static fs_work_mode_map_t s_mode_map[FS_SDK_WORK_MODE_CNT] = {
	{
		.work_mode = FS_SDK_WORK_MODE_IDLE,
		.lidar_mode = RX_LIDAR_MODE_IDLE,
	},
	{
		.work_mode = FS_SDK_WORK_MODE_GRAY,
		.lidar_mode = RX_LIDAR_MODE_GRAY,
	},
	{
		.work_mode = FS_SDK_WORK_MODE_RANGE,
		.lidar_mode = RX_LIDAR_MODE_RANGE_01,
	},
	{
		.work_mode = FS_SDK_WORK_MODE_PCL,
		.lidar_mode = RX_LIDAR_MODE_RANGE_01,
	},
	{
		.work_mode = FS_SDK_WORK_MODE_CUSTOM,
		.lidar_mode = RX_LIDAR_MODE_RANGE_1,
	},
	{
		.work_mode = FS_SDK_WORK_MODE_GRAY_CUSTOM,
		.lidar_mode = RX_LIDAR_MODE_GRAY_CUSTOM,
	},
};

int fs_sdk_boot_init(int biz_work_mode)
{
	int err = FS_SUCCESS;

	fs_sdk_work_mode_e sdk_mode = FS_SDK_WORK_MODE_PCL;
	if (biz_work_mode == TEST_POINT_CLOUD_MODE) {
		sdk_mode = FS_SDK_WORK_MODE_RANGE;
	}

	log_info("fs_sdk boot init, biz work_mode=%d -> sdk mode=%d", biz_work_mode, sdk_mode);

	do {
		err = fs_sdk_hw_init();
		if (err != FS_SUCCESS) {
			log_error("fs_sdk_hw_init failed, err=%d", err);
			break;
		}
		err = fs_sdk_mcu_upgrade(); /* 可选: MCU 固件升级, 首次开机自动烧录 MCU 固件 */
		if (err != FS_SUCCESS) {
			log_error("fs_sdk_mcu_upgrade failed, err=%d", err);
			break;
		}
		err = fs_sdk_mcu_init();
		if (err != FS_SUCCESS) {
			log_error("fs_sdk_mcu_init failed, err=%d", err);
			break;
		}
		err = fs_sdk_core_init(s_mode_map, NULL);
		if (err != FS_SUCCESS) {
			log_error("fs_sdk_core_init failed, err=%d", err);
			break;
		}
		err = fs_sdk_setup_mode(sdk_mode);
		if (err != FS_SUCCESS) {
			log_error("fs_sdk_setup_mode(%d) failed, err=%d", sdk_mode, err);
			break;
		}
		log_info("fs_sdk boot done, lidar mode=%s, version=%s",
			 sdk_mode == FS_SDK_WORK_MODE_RANGE ? "RANGE" : "PCL", fs_sdk_get_version());
	} while (0);

	return (err == FS_SUCCESS) ? RET_OK : RET_FAIL;
}

void fs_sdk_boot_exit(void)
{
	fs_sdk_cleanup();
	log_info("fs_sdk boot exit");
}

#endif /* MF150_FL6532 == 1 */
