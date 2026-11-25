/*
 * @Author       : kaihong.yang kaihong.yang@vitalchem.com
 * @Date         : 2025-01-07 02:53:52
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2025-06-02 22:58:36
 * @FilePath     : /you_see_arm/core/ys_core_spad.c
 * @Description  :
 * Copyright (c) 2025 by Vitalchem, All Rights Reserved.
 */
#include "ys_core_spad.h"

#include "../common/common.h"
#include "yousee_lidar_api_public.h"

#if CS96_BETA == 1
#include "ys_spad_reg.h" // IMX459 寄存器表, 仅 beta 使用

typedef enum {
	WAIT_INITIAL1, // 等待 COMREADY 信号，表示通信接口就绪
	WR_STANDBY, // 写入待机模式命令 (写入 0x2D20 寄存器)
	WR_STANDBY_WAIT, // 待机模式写入后的等待状态 (等待 COMREADY)
	CHECK_STANDBY1, // 检查是否进入 Standby 模式
	DIG_TEST,
	WAIT_INITIAL2, // 等待 COMREADY 信号，表示通信接口就绪
	CHECK_STANDBY2, // 检查是否进入 Standby 模式
	ANA_TEST,
	RD_STANDBY, // 读取待机状态确认 (读取 0x1A0C 寄存器)
	RD_STANDBY_CHECK, // 检查待机状态是否正确 (检查读取值是否为 0x01)
	WR_ORG_HISTOGRAM_REG, // 写入寄存器配置命令（原始直方图模式）
	WR_POINT_CLOUD_REG, // 写入寄存器配置命令（点云模式）
	WR_ACTIVE, // 写入激活模式命令 (写入 0x2D20 寄存器)
	WR_ACTIVE_WAIT, // 激活模式写入后的等待状态 (等待 COMREADY)
	RD_ACTIVE, // 读取激活状态确认 (读取 0x1A0C 寄存器)
	RD_ACTIVE_CHECK, // 检查激活状态是否正确 (检查读取值是否为 0x03)
	INI_DONE // 初始化完成状态
} imx459_state_t;

typedef enum {
	IN_INITIAL = 0x0,
	IN_STANDBY = 0x1,
	IN_INITIAL2 = 0x2,
	IN_ACTIVE = 0x3,
	IN_TERMINATE = 0x4,
	IN_DIG_TEST = 0x5,
	IN_ANA_TEST = 0x6,
} imx459_device_status_t;

typedef struct {
	imx459_state_t state;
	uint16_t reg_index;
	bool ini_done;
	uint32_t read_data;
} imx459_config_t;

imx459_config_t spad = { .state = WAIT_INITIAL1, .reg_index = 0, .ini_done = false, .read_data = 0 };

struct lidar_fpga_selftest fpga_state;

int spad_init(int work_mode)
{
#if LOCAL_TEST == 1
	return RET_OK;
#else
	if (fpga_status_get(&fpga_state) != RET_OK) {
		log_error("fpga_status_get error");
		return RET_FAIL;
	}

	switch (spad.state) {
	case WAIT_INITIAL1:
		log_debug("SPAD get in WAIT_INITIAL1");
		if (fpga_state.spi_status == COM_READY) {
			log_debug("FPGA: SPAD COMREADY");
			spad.state = WR_STANDBY;
		}
		break;

	case WR_STANDBY:
		log_debug("SPAD get in WR_STANDBY");
		if (spad_spi_write(STANDBY, 0x1, sizeof(uint32_t)) == RET_OK) {
			spad.state = WR_STANDBY_WAIT;
		}
		break;

	case WR_STANDBY_WAIT:
		log_debug("SPAD get in WR_STANDBY_WAIT");
		if (fpga_state.spi_status == COM_READY) {
			log_debug("FPGA: SPAD COMREADY");
			spad.state = CHECK_STANDBY1;
		}
		break;

	case CHECK_STANDBY1:
		log_debug("SPAD get in CHECK_STANDBY1");
		spad.read_data = spad_spi_read(DEVSTS, sizeof(uint32_t));
		log_debug("spad read_data %04x %08x", DEVSTS, spad.read_data);
		if (spad.read_data == ERR_FPGA_REG32) {
			break;
		}

		if (spad.read_data == IN_STANDBY) {
			spad.state = DIG_TEST;
		}
		break;

	case DIG_TEST:
		log_debug("SPAD get in DIG_TEST");
		if (spad_spi_write(STANDBY, 0x5, sizeof(uint32_t)) == RET_OK) {
			log_debug("spad_spi_write %04x %08x success", STANDBY, 0x5);
			spad.state = WAIT_INITIAL2;
		}
		break;

	case WAIT_INITIAL2:
		log_debug("SPAD get in WAIT_INITIAL2");
		if (fpga_state.spi_status == COM_READY) {
			log_debug("FPGA: SPAD COMREADY");
			spad.state = CHECK_STANDBY2;
		}
		break;

	case CHECK_STANDBY2:
		log_debug("SPAD get in CHECK_STANDBY2");
		spad.read_data = spad_spi_read(DEVSTS, sizeof(uint32_t));
		log_debug("spad read_data %04x %08x", DEVSTS, spad.read_data);
		if (spad.read_data == ERR_FPGA_REG32) {
			break;
		}

		if (spad.read_data == IN_STANDBY) {
			spad.state = ANA_TEST;
		}
		break;

	case ANA_TEST:
		log_debug("SPAD get in ANA_TEST");
		if (spad_spi_write(STANDBY, 0x9, sizeof(uint32_t)) == RET_OK) {
			log_debug("spad_spi_write %04x %08x success", STANDBY, 0x9);
			spad.state = RD_STANDBY;
		}
		break;

	case RD_STANDBY:
		log_debug("SPAD get in RD_STANDBY");
		if (fpga_state.spi_status != COM_READY) {
			break;
		}

		spad.read_data = spad_spi_read(DEVSTS, sizeof(uint32_t));
		log_debug("spad read_data %04x %08x", DEVSTS, spad.read_data);
		if (spad.read_data != ERR_FPGA_REG32) {
			spad.state = RD_STANDBY_CHECK;
		}
		break;

	case RD_STANDBY_CHECK:
		log_debug("SPAD get in RD_STANDBY_CHECK");
		if (spad.read_data == IN_STANDBY) {
			spad.reg_index = 0;
			spad.state = work_mode == RAW_MODE ? WR_ORG_HISTOGRAM_REG : WR_POINT_CLOUD_REG;
		} else {
			spad.state = RD_STANDBY;
		}
		break;

	case WR_ORG_HISTOGRAM_REG:
		if (spad.reg_index >= sizeof(spad_original_histogram_reg) / sizeof(spad_original_histogram_reg[0])) {
			spad.state = WR_ACTIVE;
		} else {
			const reg_config_t *cfg = &spad_original_histogram_reg[spad.reg_index];
			if (spad_spi_write(cfg->addr, cfg->data, sizeof(cfg->data)) == RET_OK) {
				log_debug("spad_spi_write %04x %08x success", cfg->addr, cfg->data);
				spad.reg_index++;
			} else {
				log_error("spad_spi_write %04x %08x failed", cfg->addr, cfg->data);
			}
		}
		break;

	case WR_POINT_CLOUD_REG:
		if (spad.reg_index >= sizeof(spad_point_cloud_reg) / sizeof(spad_point_cloud_reg[0])) {
			spad.state = WR_ACTIVE;
		} else {
			const reg_config_t *cfg = &spad_point_cloud_reg[spad.reg_index];
			if (spad_spi_write(cfg->addr, cfg->data, sizeof(cfg->data)) == RET_OK) {
				log_debug("spad_spi_write %04x %08x success", cfg->addr, cfg->data);
				spad.reg_index++;
			} else {
				log_error("spad_spi_write %04x %08x failed", cfg->addr, cfg->data);
			}
		}
		break;

	case WR_ACTIVE:
		log_debug("SPAD get in WR_ACTIVE");
		if (spad_spi_write(STANDBY, 0x0, sizeof(uint32_t)) == RET_OK) {
			log_debug("spad_spi_write %04x %08x success", STANDBY, 0x0);
			spad.state = WR_ACTIVE_WAIT;
		} else {
			log_error("spad_spi_write %04x %08x failed", STANDBY, 0x0);
		}
		break;

	case WR_ACTIVE_WAIT:
		log_debug("SPAD get in WR_ACTIVE_WAIT");

		if (fpga_state.spi_status == COM_READY) {
			log_debug("FPGA: SPAD COMREADY");
			spad.state = RD_ACTIVE;
		}

		break;

	case RD_ACTIVE:
		log_debug("SPAD get in RD_ACTIVE");
		spad.read_data = spad_spi_read(DEVSTS, sizeof(uint32_t));
		log_debug("spad read_data %04x %08x", DEVSTS, spad.read_data);
		if (spad.read_data != ERR_FPGA_REG32) {
			spad.state = RD_ACTIVE_CHECK;
		}
		break;

	case RD_ACTIVE_CHECK:
		log_debug("SPAD get in RD_ACTIVE_CHECK");
		if (spad.read_data == IN_ACTIVE) {
			spad.state = INI_DONE;
			spad.ini_done = true;
		} else {
			spad.state = RD_ACTIVE;
		}
		break;

	case INI_DONE:
		log_info("SPAD init done!");
		break;
	}

	if (spad.ini_done) {
		return RET_OK;
	} else {
		return RET_FAIL;
	}
#endif
}
#elif CS96_GAMMA == 1
/**
 * @description  : SPAD init
 * @param         {int} work_mode
 * @return        {int} result
 * SL100 由 FPGA 自动完成上电初始化, ARM 端不参与寄存器配置,
 * 此处保留接口以维持 sys_spad_init() 调用链一致。
 * 实机默认 config.set_spad = 0, 不会走到这里。
 */
int spad_init(int work_mode)
{
	(void)work_mode;
	return RET_OK;
}
#elif MF150_FL6031 == 1

typedef struct mf150_fl6031_reg_config {
	uint16_t addr;
	uint8_t val;
} mf150_fl6031_reg_config_t;

static const mf150_fl6031_reg_config_t reg_configs[] = {
	{ 0x0115, 0x08 }, { 0x0116, 0x0F }, { 0x0117, 0x00 }, { 0x0118, 0x08 }, { 0x0662, 0x14 }, { 0x0403, 0x80 },
	{ 0x0405, 0x2F }, { 0x080E, 0x1E }, { 0x0827, 0x03 }, { 0x0828, 0x03 }, { 0x0A11, 0x09 }, { 0x0A12, 0x04 },
	{ 0x081A, 0x09 }, { 0x011B, 0x04 }, { 0x0A13, 0x0E }, { 0x0420, 0x0F }, { 0x041F, 0x08 }, { 0x0580, 0x25 },
	{ 0x040F, 0x5A }, { 0x0302, 0xB4 }, { 0x0303, 0x4C }, { 0x0346, 0x5A }, { 0x0408, 0x09 }, { 0x0400, 0xB7 },
	{ 0x0406, 0x06 }, { 0x0407, 0xA3 }, { 0x0401, 0x01 }, { 0x0402, 0x28 }, { 0x040A, 0x03 }, { 0x0409, 0x22 },
	{ 0x0414, 0xAA }, { 0x0403, 0xC0 },
};

int spad_init()
{
	// 复位 SPAD
#if LOCAL_TEST == 1
	log_info("skip SPAD init in LOCAL_TEST mode");
	return RET_OK;
#else
#if 0
	log_info("使能 SPAD 触发...");
	SPAD_TRIG_MODE(1);

	log_info("复位 SPAD...");
	SPAD_RESET(0);
	usleep(30 * 1000);
	SPAD_RESET(1);

	// 寄存器配置，适配 800M 的 MIPI
	log_info("配置 SPAD 寄存器...");
	for (int i = 0; i < sizeof(reg_configs) / sizeof(mf150_reg_config_t); i++) {
		SPAD_REG_W(reg_configs[i].addr, reg_configs[i].val);
	}

	// 检查寄存器配置
	log_info("检查寄存器配置...");
	for (int i = 0; i < sizeof(reg_configs) / sizeof(mf150_reg_config_t); i++) {
		uint8_t val = SPAD_REG_R(reg_configs[i].addr);
		if (val != reg_configs[i].val && reg_configs[i].addr != 0x0403) {
			log_error("第 %02d 个寄存器 0x%04X 配置错误：期望 0x%02X, 读取到 0x%02X", i + 1,
				  reg_configs[i].addr, reg_configs[i].val, val);
		} else {
			log_info("第 %02d 个寄存器 0x%04X 配置正确：0x%02X", i + 1, reg_configs[i].addr, val);
		}
	}

	// enable SPAN_EN
	log_info("使能 SPAD 通知...\n");
	SPAD_NOTIFY_EN();
#endif
	return RET_OK;
#endif
}
#endif
