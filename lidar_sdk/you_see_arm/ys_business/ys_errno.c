/*
 * @Author       : kaihong.yang kaihong.yang@vitalchem.com
 * @Date         : 2024-10-23 03:56:47
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2024-10-23 08:35:52
 * @FilePath     : /you_see_arm/ys_business/ys_errno.c
 * @Description  :
 * Copyright (c) 2024 by Vitalchem, All Rights Reserved.
 */

#include "ys_errno.h"

static error_info_t g_error_info;

int save_error_code(void *arg, uint8_t error_code)
{
	management_id_t *msg = (management_id_t *)arg;

	if (msg == NULL) {
		return RET_FAIL;
	}

	g_error_info.cmd_set = msg->code_set;
	g_error_info.cmd_id = msg->code_id;
	g_error_info.error_code = error_code;

	return RET_OK;
}

int get_error_code(uint8_t *data)
{
	int len = sizeof(error_info_t);
	memcpy(data, (uint8_t *)&g_error_info, len);

	return len;
}
