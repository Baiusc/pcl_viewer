/*
 * @Author       : yangkh kaihong.yang@vitalchem.com
 * @Date         : 2024-05-27 04:03:14
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2025-05-22 20:24:00
 * @FilePath     : /you_see_arm/ys_business/main.c
 * @Description  :
 * Copyright (c) 2024 by Vitalchem, All Rights Reserved.
 */

#include "ys_business_network.h"
#include "ys_business_sys.h"
#include "ys_msg_distribute.h"
#include "ys_ps_config.h"

#include "../common/common.h"

#include "../core/ys_core_system.h"
#include "../core/ys_version.h"
#include "../core/ys_management.h"

#include <signal.h> /* for signal */
#include <execinfo.h> /* for backtrace() */
#if CS96_BETA == 1
#include "../manager/fpga_spi.h"
#include <inttypes.h>
#include <unistd.h>
#endif

#define BACKTRACE_SIZE 16

static void dump(void)
{
	int j, nptrs;
	void *buffer[BACKTRACE_SIZE];
	char **strings;
	nptrs = backtrace(buffer, BACKTRACE_SIZE);
	printf("backtrace() returned %d addresses\n", nptrs);
	strings = backtrace_symbols(buffer, nptrs);

	if (strings == NULL) {
		perror("backtrace_symbols");
		exit(EXIT_FAILURE);
	}

	for (j = 0; j < nptrs; j++) {
		printf("  [%02d] %s\n", j, strings[j]);
	}

	free(strings);
}

static void signal_handler(int signo)
{
	printf("\ncatch signal %d \n", signo);
	printf("Dump stack start...\n");
	dump();
	printf("Dump stack end...\n");
	signal(signo, SIG_DFL); /* 恢复信号默认处理 */
	raise(signo); /* 重新发送信号 */
}

#if CS96_BETA == 1
static int run_fpga_spot(void)
{
	lidar_fpga_version_info_t info;
	if (!ys_management_enabled() || fpga_read_version_info(&info) ||
	    (info.version_num & 0xffffff00U) != LIDAR_BETA_SPOT_VERSION) {
		fputs("Beta 光斑模式的 FPGA 版本寄存器校验失败\n", stderr);
		return EXIT_FAILURE;
	}
	printf("Beta 光斑 FPGA：version_data=0x%08" PRIx32 "，version_time=0x%08" PRIx32 "，version_num=0x%08" PRIx32
	       "，hash_value=0x%08" PRIx32 "，mod_state=0x%08" PRIx32 "\n",
	       info.version_data, info.version_time, info.version_num, info.hash_value, info.mod_state);
	fflush(stdout);
	ys_management_initialized(LIDAR_BETA_SPOT_MODE);
	while (!ys_shutdown_requested())
		sleep(1);
	return EXIT_SUCCESS;
}
#endif

int main(int argc, char *argv[])
{
	char version_str[YS_VERSION_STRING_MAX] = { '\0' };
	int ret = RET_FAIL;

	if (ys_management_init())
		return EXIT_FAILURE;
#if CS96_BETA == 1
	/* 光斑固件从 SPI 读取版本信息，跳过 DMA、采集线程和业务网络。 */
	if (argc == 2 && strcmp(argv[1], "--fpga-spot") == 0) {
		ret = run_fpga_spot();
		ys_management_close();
		return ret;
	}
#endif
	signal(SIGSEGV, signal_handler);

	ys_version_string(version_str, sizeof(version_str));
	/* 保留历史输出格式, 兼容已有的日志解析习惯 */
	printf("Program version: %s, Model = %s\n", version_str, MODEL);

	/* 打印完整版本横幅。刻意放在 load_config() 之前: 日志等级由配置文件决定, 若现场把等级
	 * 调高, dirty 告警会被过滤掉, 而最需要看到它的人恰恰是测试人员 */
	ys_version_log_banner();

	if (!ys_management_enabled())
		check_system();

	check_network();

	ret = load_config();
	if (ret == RET_FAIL) {
		log_fatal("load config fail");
		goto end;
	}

	ret = load_cal_config();
	if (ret == RET_FAIL) {
		log_fatal("load calibration config fail");
		goto end;
	}

	if (config.set_spad == ON) {
		ret = sys_spad_init(config.work_mode);
		if (ret == RET_FAIL) {
			log_fatal("spad init fail");
			goto end;
		}
	}

	ret = sys_init();
	if (ret == RET_FAIL) {
		log_fatal("system init fail");
		goto end;
	}

	ret = ys_shutdown_requested() ? RET_OK : event_loop();

end:
	if (sys_uninit() != RET_OK)
		ret = RET_FAIL;
	destroy_cal_config();
	destroy_config();
	ys_management_close();
	return ret == RET_OK ? EXIT_SUCCESS : EXIT_FAILURE;
}
