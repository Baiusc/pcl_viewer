#include "ys_decode.h"
#include "ys_decode_mf150_fl6532.h"

#include "../algo/algo_parse_cloud.h"

#include "../../common/common.h"
#include "../../core/ys_base.h"
#include "../../core/ys_conf.h"
#include "../../core/ys_core_def.h"
#include "../../core/ys_core_system.h"

#include "../consumer/ys_pipeline_io.h"
#include "../../third_party/fs_sdk/sdk/net_server/inc/fs_net_protocol.h"
#include "../../third_party/fs_sdk/sdk/src/rx_cfg/rx_cfg_loader.h"

#include <math.h>

/******************************************************************************/
/*                         FS6532 专有解析函数                                  */
/******************************************************************************/

static int circl_cnt = 0;
int mf150_fl6532_parse_group_to_points(mf150_fl6532_raw_group_packet_t *raw_group,
				       mf150_fl6532_point_cloud_packet_t *dest_packet)
{
	if (raw_group == NULL || dest_packet == NULL) {
		log_error("input data is NULL");
		return 0;
	}

#if RANGE_TX_LZ4
	/* Range LZ4 压缩包已是最终线格式(自定义头+压缩流), 无需重组, 原样透传 */
	{
		const mf150_fl6532_range_lz4_hdr_t *lz4 = (const mf150_fl6532_range_lz4_hdr_t *)raw_group;

		if ((lz4->sof == MF150_FL6532_RANGE_LZ4_SOF) && (lz4->version == MF150_FL6532_RANGE_LZ4_VERSION) &&
		    (lz4->data_len <= raw_point_cloud_full_size)) {
			memcpy(dest_packet, raw_group, lz4->data_len);
			return (int)lz4->data_len;
		}
	}
#endif

	uint8_t *dest = (uint8_t *)dest_packet;
	fs_protocol_head_t *head = (fs_protocol_head_t *)dest;
	fs_lidar_pcl_echo_data_t *echo = (fs_lidar_pcl_echo_data_t *)(dest + sizeof(fs_protocol_head_t));
	get_head_pkg(head, raw_group->data_type);
	memset(echo, 0, sizeof(fs_lidar_pcl_echo_data_t));
	echo->protocol_ver_minor = raw_group->echo_protocol_ver_minor;
	echo->data_sequence = raw_group->echo_data_sequence;
	echo->frame_cnt = raw_group->echo_frame_cnt;
	echo->dot_num = raw_group->echo_dot_num;
	echo->type = raw_group->echo_type;
	echo->peak_type_2_1 = raw_group->echo_peak_type_2_1;
	echo->frame_flag = raw_group->echo_frame_flag;
	echo->width = raw_group->echo_width;
	echo->height = raw_group->echo_height;
	echo->max_bin_cnt = raw_group->echo_max_bin_cnt;
	echo->ab_frame_idx = raw_group->echo_ab_frame_idx;
	echo->max_sub_frame_cnt = raw_group->echo_max_sub_frame_cnt;
	echo->roi_index = raw_group->echo_roi_index;
	echo->pixel_len = raw_group->echo_pixel_len;
	echo->data_offset = raw_group->echo_data_offset;
	echo->data_len = raw_group->echo_data_len;
	echo->peak_type_4_3 = raw_group->echo_peak_type_4_3;
	echo->timestamp_base = raw_group->echo_timestamp_base;

	uint16_t payload_len = raw_group->echo_dot_num * raw_group->echo_pixel_len;
	memcpy(echo->data, raw_group->raw_pixels, payload_len);

	return (int)(sizeof(fs_protocol_head_t) + sizeof(fs_lidar_pcl_echo_data_t) + payload_len);
}

bool mf150_fl6532_orgin_point_cloud_data_is_valid(const mf150_fl6532_raw_group_packet_t *raw_data)
{
	(void)raw_data;
	return (cal_param.side_light >> 0) & 1;
}
