#define _POSIX_C_SOURCE 200809L
#include "../../core/ys_management.h"
#include "../../manager/fpga_spi.h"
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

/* 只用于无雷达开发板：复用真实私有健康通道和退出处理，不打开任何硬件。 */
int main(int argc, char **argv)
{
	const char *value = getenv("LIDAR_WORK_MODE"), *flags = getenv("LIDAR_DEVBOARD_FLAGS");
	if (!value || strlen(value) != 1 || value[0] < '0' || value[0] > '0' + LIDAR_BETA_SPOT_MODE || !flags ||
	    flags[0] != '/')
		return 2;
	int mode = atoi(value);
	int spot = mode == LIDAR_BETA_SPOT_MODE;
	if (spot != (argc == 2 && strcmp(argv[1], "--fpga-spot") == 0))
		return 2;
	if (ys_management_init() || !ys_management_enabled())
		return 2;
	char timeout_flag[PATH_MAX];
	if (snprintf(timeout_flag, sizeof(timeout_flag), "%s/timeout-mode-%d", flags, mode) >=
	    (int)sizeof(timeout_flag))
		return 2;
	printf("【开发板模拟采集】work_mode=%d；不打开雷达设备，不产生真实点云\n", mode);
	fflush(stdout);
	ys_management_initialized(mode);
	struct timespec delay = { .tv_nsec = 20000000 };
	while (!ys_shutdown_requested()) {
		/* 标记存在时只完成初始化，不报告有效数据，用于验证超时和回滚。 */
		if (!spot)
			ys_management_progress(mode, access(timeout_flag, F_OK) != 0);
		nanosleep(&delay, NULL);
	}
	puts("开发板模拟采集已正常退出");
	ys_management_close();
	return 0;
}
