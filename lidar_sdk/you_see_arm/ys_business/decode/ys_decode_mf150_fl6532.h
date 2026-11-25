#ifndef YS_DECODE_MF150_FL6532_H
#define YS_DECODE_MF150_FL6532_H

#include "../ys_def.h"
#include "../../core/ys_core_def.h"
#include "../consumer/ys_pipeline_io.h"

#include <stdint.h>
#include <stdbool.h>

int mf150_fl6532_parse_group_to_points(mf150_fl6532_raw_group_packet_t *raw_group,
				       mf150_fl6532_point_cloud_packet_t *dest_packet);
bool mf150_fl6532_orgin_point_cloud_data_is_valid(const mf150_fl6532_raw_group_packet_t *raw_data);

#endif // YS_DECODE_MF150_FL6532_H
