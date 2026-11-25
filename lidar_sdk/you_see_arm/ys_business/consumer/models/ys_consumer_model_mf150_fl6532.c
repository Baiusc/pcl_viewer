/*
 * @Author       : kaihong.yang kaihong.yang@vitalchem.com
 * @Date         : 2025-06-24
 * @FilePath     : /you_see_arm/ys_business/consumer/models/ys_consumer_model_mf150.c
 * @Description  : MF150 消费者模型实现
 * Copyright (c) 2025 by Vitalchem, All Rights Reserved.
 */

#include "ys_consumer_model.h"

#include "../../decode/ys_decode.h"
#include "../../decode/ys_decode_mf150_fl6532.h"
#include "../../ys_def.h"
#include "../../../core/ys_core_system.h"
#include "../../../third_party/fs_sdk/sdk/net_server/inc/fs_net_protocol.h"

int consumer_model_get_processed_len(int lines_v, int echo_mode)
{
	(void)lines_v;
	(void)echo_mode;
#if RANGE_TX_LZ4
	/* Range LZ4 压缩包透传: 输出缓冲需容纳整包(含包头) */
	return raw_point_cloud_full_size;
#else
	return FS_PROTOCOL_UDP_MTU_SIZE;
#endif
}

bool consumer_model_point_cloud_is_valid(const void *data)
{
	return mf150_fl6532_orgin_point_cloud_data_is_valid((const mf150_fl6532_raw_group_packet_t *)data);
}

int consumer_model_transform_point_cloud(const void *raw, void *dest)
{
	(void)dest;
#if RANGE_TX_LZ4
	/* Range LZ4 压缩包: 返回包头 data_len, consumer 按实际长度发送,
	 * 避免固定 1522 字节发送造成尾部填充占用带宽 */
	const mf150_fl6532_range_lz4_hdr_t *h = (const mf150_fl6532_range_lz4_hdr_t *)raw;

	if ((h->sof == MF150_FL6532_RANGE_LZ4_SOF) && (h->version == MF150_FL6532_RANGE_LZ4_VERSION) &&
	    (h->data_len >= sizeof(mf150_fl6532_range_lz4_hdr_t)) && ((int)h->data_len <= raw_point_cloud_full_size)) {
		return (int)h->data_len;
	}
#else
	(void)raw;
#endif
	return 0;
}

int consumer_model_parse_point_cloud(const void *raw, void *dest)
{
	return mf150_fl6532_parse_group_to_points((mf150_fl6532_raw_group_packet_t *)raw,
						  (mf150_fl6532_point_cloud_packet_t *)dest);
}
