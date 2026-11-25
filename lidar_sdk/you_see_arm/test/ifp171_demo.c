/*
 * @Author       : yangkh kaihong.yang@vitalchem.com
 * @Date         : 2024-06-11 03:38:25
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2024-06-30 21:49:06
 * @FilePath     : /you_see_arm/test/ifp171_demo.c
 * @Description  :
 * Copyright (c) 2024 by Vitalchem, All Rights Reserved.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <getopt.h>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include "ifp171_api.h"

static unsigned char break_flag = 0;
extern unsigned char pexit_init;

/******************************************************************************************/

static void ifp171_signal_handler(int signal)
{
	printf("catch signal:%d pexit_init:%d\n", signal, ++pexit_init);
	if (pexit_init == 2) {
		break_flag = 1;
	}
}

void usage_help(void)
{
	printf("/******** usage help *******************************/\n");
	printf("/*   -h   help                                     */\n");
	printf("/*   -l   loop                                     */\n");
	printf("/*   -s   spi option                               */\n");
	printf("/*   -r   reg phy addr                             */\n");
	printf("/*   -v   value                                    */\n");
	printf("/*   -R   Read(defaule: read)                      */\n");
	printf("/*   -W   Write(defaule: read)                     */\n");
	printf("/*   -g   get status                               */\n");
	printf("/*   -f   save file path                           */\n");
	printf("/*   -d   select ddr data (defaule: bram data)     */\n");
	printf("/***************************************************/\n");
}

int main(int argc, char *argv[])
{
	int ret = 0;
	unsigned int ret_u32 = 0;
	short ret_s = 0;
	float ret_f = 0;
	int ch = 0;
	unsigned char loop_flag = 0;
	unsigned int reg_opt = 0;
	unsigned char spi_opt = 0;
	unsigned int reg = 0;
	unsigned int value = 0;
	unsigned char rw = 0;
	unsigned char gets = 0;
	struct ifp171_fpga_selftest fpga_status;
	unsigned char ddr_en = 0;
	char *file = NULL;
	char filename[20] = { 0 };
	unsigned int i;
	int fd_buf = 0;
	struct ifp171_buffer_list *node = NULL;
	int file_cnt = DEF_SLOT_PER_FRAME;
	int cnt;

	if (argc < 2) {
		usage_help();
		goto exit;
	}

	while ((ch = getopt(argc, argv, "hlsr:v:RWgf:d")) != -1) {
		switch (ch) {
		case 'h':
			usage_help();
			goto exit;
		case 'l':
			loop_flag = 1;
			break;
		case 's':
			spi_opt = 1;
			break;
		case 'r':
			reg = strtoull(optarg, NULL, 0);
			reg_opt = 1;
			break;
		case 'v':
			value = strtoull(optarg, NULL, 0);
			break;
		case 'R':
			rw = 0;
			break;
		case 'W':
			rw = 1;
			break;
		case 'g':
			gets = 1;
			break;
		case 'f':
			file = optarg;
			break;
		case 'd':
			ddr_en = 1;
			file_cnt = DEF_HISTOGRAM_PER_FRAME;
			break;
		default:
			printf("invalid option args!\n\n");
			usage_help();
			goto exit;
		}
	}

	signal(SIGINT, ifp171_signal_handler);

	if (ifp171_api_init()) {
		printf("ifp171_api_init() failed!\n");
		goto exit;
	}

	if (reg_opt) {
		if (spi_opt) {
			if (rw) {
				ret = spad_spi_write(reg, value, sizeof(value));
			} else {
				value = spad_spi_read(reg, sizeof(value));
			}
			printf("spi %c : 0x%x - 0x%x  ret=%d\n", (rw) ? 'W' : 'R', reg, value, ret);
		} else {
#ifdef DEF_DEMO_TEST
			if (rw) {
				regw32(reg, value);
			} else {
				value = regr32(reg);
			}
			printf("reg %c : 0x%x - 0x%x  ret=%d\n", (rw) ? 'W' : 'R', reg, value, ret);
#else
			printf("not support reg r/w\n");
#endif /* DEF_DEMO_TEST */
		}
	}

	if (gets) {
		ret = sys_workmode_get();
		switch (ret & 0xff) {
		case 0x00:
			printf("workmode : free\n");
			break;
		case 0x02:
			printf("workmode : run\n");
			break;
		case 0x04:
			printf("workmode : fault\n");
			break;
		case 0x08:
			printf("workmode : debug\n");
			break;
		default:
			printf("sys_workmode_get() failed!\n");
			break;
		}

		ret = sys_fps_get();
		printf("sys_fps : %d\n", ret);

		ret = calib_zero_offset_get(&ret_s);
		if (!ret) {
			printf("calib_zero_offset : %d\n", ret_s);
		}

		ret = fpga_status_get(&fpga_status);
		if (!ret) {
			printf("fpga_status:\n");
			printf("  done         : %d\n", fpga_status.done);
			printf("  workmode     : %d\n", fpga_status.workmode);
			printf("  motor_status : %d\n", fpga_status.motor_status);
			printf("  mirror_angle : %d\n", fpga_status.mirror_angle);
			printf("  spi_status   : %d\n", fpga_status.spi_status);
		}

		ret = ld_temp_get(&ret_s);
		if (!ret) {
			printf("ld_temp : %d\n", ret_s);
		}

		ret = spad_temp_get(&ret_s);
		if (!ret) {
			printf("spad_temp : %d\n", ret_s);
		}

		ret = fpga_temp_get(&ret_s);
		if (!ret) {
			printf("fpga_temp : %d\n", ret_s);
		}

		ret_u32 = motor_period_get();
		printf("motor_period : %d\n", ret);

		ret_u32 = vop_sample_get();
		printf("vop_sample : %d\n", ret);

		/* ret_u32 = a_current_get();
        printf("a_current : %d\n", ret);

        ret_u32 = b_current_get();
        printf("b_current : %d\n", ret);

        ret_u32 = c_current_get();
        printf("c_current : %d\n", ret); */

#ifdef DEF_TEST_FUN
		ret = motor_agle_get(&ret_f);
		if (!ret) {
			printf("motor_agle : %f\n", ret_f);
		}

		ret_u32 = motor_zero_time_get();
		printf("motor_zero_time : %d\n", ret_u32);
#endif /* DEF_TEST_FUN */
	}

	i = 0;
	cnt = 0;
	do {
		if (break_flag) {
			break;
		}

		if (file) {
			if (cnt >= file_cnt) {
				cnt = 0;
				if (fd_buf) {
					close(fd_buf);
				}
				i++;
			}

			if (!cnt) {
				sprintf(filename, "%s%d", file, i);
				fd_buf = open(filename, O_CREAT | O_RDWR, S_IRUSR | S_IWUSR);
				if (fd_buf < 0) {
					printf("open %s failed!\n", filename);
					continue;
				}
			}

get_node:
			node = ifp171_get_bufnode();
			if (!node) {
				if (!cnt && fd_buf) {
					close(fd_buf);
				}
				sleep(1);
				printf("bufnode empty!\n");
				continue;
			}
			if (ddr_en) {
				if (node->size != IFP171_HISTOGRAM_SIZE) {
					ifp171_del_bufnode(node);
					goto get_node;
				}
			} else {
				if (node->size != IFP171_POINT_SIZE) {
					ifp171_del_bufnode(node);
					goto get_node;
				}
			}

			write(fd_buf, node->addr, node->size);

			ifp171_del_bufnode(node);

			cnt++;
		}
	} while (loop_flag);

	ifp171_api_exit();
	sleep(3);

exit:
	return ret;
}
