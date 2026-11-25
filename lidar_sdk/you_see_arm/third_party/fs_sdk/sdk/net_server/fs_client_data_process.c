#include <fcntl.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

#include "utils/fs_err.h"
#include "utils/fs_log.h"
#include "utils/fs_util.h"
#include "fs_udp_server.h"
#include "fs_net_protocol.h"
#include "fs_client_data_process.h"
#include "rx_cfg_loader.h"
#include "time_sync_eval.h"
#include "soc_system_manager.h"

#ifdef LOG_TAG
#undef LOG_TAG
#endif
#define LOG_TAG "fs_client_data_process"

#define DEFAULT_TARGET_IP "192.168.137.100"

#define FS_PCL_DATA_UDP_PORT            6001
#define FS_PCL_DATA_UDP_TARGET_PORT     6002
#define FS_IMU_DATA_UDP_PORT            6301
#define FS_IMU_DATA_UDP_TARGET_PORT     6002

static fs_udp_param g_pcldata_udp = {
    .sockfd = -1,
    .strTargetIp = DEFAULT_TARGET_IP,
};

static fs_udp_param g_imudata_udp = {
    .sockfd = -1,
    .strTargetIp = DEFAULT_TARGET_IP,
};

static int pixel_udp_blocked_by_type(uint8_t data_type)
{
    if (!soc_mgr_pcl_udp_allowed()) {
        if (data_type == DATA_TYPE_POINTCLOUD
            || data_type == DATA_TYPE_CICL_DATA
            || data_type == DATA_TYPE_GL_DATA) {
            return 1;
        }
    }
    return 0;
}

int32_t fs_udp_server_init(void)
{
    int32_t err = FS_SUCCESS;

    snprintf(g_pcldata_udp.strTargetIp, sizeof(g_pcldata_udp.strTargetIp), "%s", UDP_DATA_TARGET_IP);
    g_pcldata_udp.targetPort = FS_PCL_DATA_UDP_TARGET_PORT;

    err = fs_udp_init(&g_pcldata_udp, FS_PCL_DATA_UDP_PORT);
    if (err < 0) {
        FS_LOGE(" udp server init failed...");
    }

    snprintf(g_imudata_udp.strTargetIp, sizeof(g_imudata_udp.strTargetIp), "%s", UDP_DATA_TARGET_IP);
    g_imudata_udp.targetPort = FS_IMU_DATA_UDP_TARGET_PORT;

    err = fs_udp_init(&g_imudata_udp, FS_IMU_DATA_UDP_PORT);
    if (err < 0) {
        FS_LOGE(" udp server init failed...");
    }

    return err;
}

int32_t fs_udp_server_uninit(void)
{
    if (g_pcldata_udp.sockfd > 0) {
        close(g_pcldata_udp.sockfd);
        g_pcldata_udp.sockfd = -1;
    }

    if (g_imudata_udp.sockfd > 0) {
        close(g_imudata_udp.sockfd);
        g_imudata_udp.sockfd = -1;
    }

    return FS_SUCCESS;
}

int fs_send_imu_data(imu_sensor_data_t *imu_data, uint32_t count)
{
    fs_protocol_head_t    *head_pkg;
    fs_lidar_imu_header_t *imu_header;
    uint8_t data_buf[sizeof(*head_pkg) + sizeof(*imu_header) + sizeof(imu_header->data[0]) * 32]; // max count: 32
    uint8_t crc8 = 0;
    uint32_t dlen = 0;

    if (imu_data == NULL) {
        FS_LOGE("imu_data is NULL.");
        return FS_ERR_NULL_PTR;
    }
    if (count > (sizeof(data_buf) - sizeof(*head_pkg) - sizeof(*imu_header)) / sizeof(imu_header->data[0])) {
        FS_LOGE("invalid count(%u) > %zu", count,
                (sizeof(data_buf) - sizeof(*head_pkg) - sizeof(*imu_header)) / sizeof(imu_header->data[0]));
        return FS_ERR_BUF_OVERFLOW;
    }
    if (!soc_mgr_imu_udp_allowed()) {
        return FS_SUCCESS;
    }

    memset(data_buf, 0, sizeof(data_buf));
    head_pkg = (fs_protocol_head_t *)data_buf;
    imu_header = (fs_lidar_imu_header_t *)&data_buf[sizeof(*head_pkg)];
    get_head_pkg(head_pkg, DATA_TYPE_IMU_DATA);
    imu_header->crc8_header = fs_util_crc8(head_pkg, sizeof(*head_pkg), 0);
    imu_header->version = 0x02;
    imu_header->len = sizeof(*imu_header) + count * sizeof(imu_header->data[0]);
    imu_header->time_type = 0;

    for (uint32_t i = 0; i < count; i++) {
        imu_header->data[i].timestamp = imu_data[i].timestamp;
        imu_header->data[i].temperature = imu_data[i].temperature;
        imu_header->data[i].gyro_x = imu_data[i].gyro_x;
        imu_header->data[i].gyro_y = imu_data[i].gyro_y;
        imu_header->data[i].gyro_z = imu_data[i].gyro_z;
        imu_header->data[i].acc_x = imu_data[i].acc_x;
        imu_header->data[i].acc_y = imu_data[i].acc_y;
        imu_header->data[i].acc_z = imu_data[i].acc_z;
        crc8 = fs_util_crc8(&imu_header->data[i], sizeof(imu_header->data[i]), crc8);
    }

    imu_header->crc8_imu_dat = crc8;
    dlen = sizeof(*head_pkg) + sizeof(*imu_header) + count * sizeof(imu_header->data[0]);

    return fs_udp_send_data(&g_imudata_udp, data_buf, dlen, __FUNCTION__);
}


static int circl_cnt = 0;

int32_t fs_send_pixel_data_one_group(fs_lidar_data_info_t *info)
{
    uint16_t dot_num = 0;
    uint16_t max_dot_num = 0;
    int32_t  remain_dot_num;
    int32_t  j = 0;
    int32_t  pkt_num;
    uint8_t  data_buf[FS_PROTOCOL_UDP_MTU_SIZE] = { 0 };
    uint16_t pixel_len  = info->group_info.pixel_len;
    uint32_t data_count = info->group_info.pixel_count;
    uint8_t  roi_index  = info->group_info.roi_index & 0x03;
    fs_lidar_pcl_echo_data_t *echo_data;
    fs_protocol_head_t *head_pkg;
    static uint16_t next_start_sequence = 0;
    static int32_t max_group_num = 0;
    static uint16_t max_sequence = 0;

    if (pixel_udp_blocked_by_type(info->data_type)) {
        return FS_SUCCESS;
    }

    if (max_group_num == 0) {
        int32_t all_group_num = 0;
        const rx_parse_cfg_t *parse_cfg;
        rx_cfg_get_rx_parse(RX_LIDAR_MODE_RANGE_01, &parse_cfg);
        uint8_t frame_cfg_cnt = (uint8_t)parse_cfg->is_ab_frame + 1;

        for (int32_t ab_idx = 0; ab_idx < frame_cfg_cnt; ab_idx++) {
            if (!parse_cfg->frm_cfg[ab_idx].is_b_frame && ab_idx == 1) {
                continue;
            }
            else if (parse_cfg->frm_cfg[ab_idx].is_b_frame && ab_idx == 0) {
                continue;
            }

            all_group_num += parse_cfg->frm_cfg[ab_idx].group_cnt;
        }

        max_group_num = all_group_num;
    }

    if (info->group_info.vir_group_idx == 0) {
        next_start_sequence = 0;
    }

    head_pkg  = (fs_protocol_head_t *)(data_buf);
    echo_data = (fs_lidar_pcl_echo_data_t *)(data_buf + sizeof(fs_protocol_head_t));
    get_head_pkg(head_pkg, info->data_type);
    remain_dot_num = data_count;
    max_dot_num = (FS_PROTOCOL_UDP_MTU_SIZE - sizeof(fs_protocol_head_t) - sizeof(fs_lidar_pcl_echo_data_t)) / pixel_len;

    echo_data->protocol_ver_minor = PROTOCOL_PCL_VER_MINOR;
    echo_data->frame_cnt          = info->frm_id;
    echo_data->width              = info->ROI_info[roi_index].width;
    echo_data->height             = info->ROI_info[roi_index].height;
    echo_data->max_bin_cnt        = info->max_bin_cnt;
    echo_data->ab_frame_idx       = ((info->ROI_info[roi_index].pcl_unit & 0x7F) << 1) | (info->ab_frame_idx & 0x01);
    echo_data->max_sub_frame_cnt  = info->max_sub_frame_cnt & 0x0F;
    echo_data->roi_index          = roi_index;
    echo_data->pixel_len          = pixel_len;
    echo_data->timestamp_base     = info->timestamp_base;

    switch ((uint32_t)info->data_type) {
        case DATA_TYPE_POINTCLOUD: {
            echo_data->type = FS_PCL_DIS_XYZ;
            break;
        }

        case DATA_TYPE_CICL_DATA: {
            echo_data->type = FS_CICL_DATA;
            break;
        }

        case DATA_TYPE_GL_DATA: {
            echo_data->type = FS_GL_DATA;
            break;
        }

        default: {
            echo_data->type = FS_CICL_DATA;
            break;
        }
    }

    echo_data->peak_type_2_1 = ((info->peak_type[1] & 0x0F) << 4) | (info->peak_type[0] & 0x0F);
    echo_data->peak_type_4_3 = ((info->peak_type[3] & 0x0F) << 4) | (info->peak_type[2] & 0x0F);

    if (max_dot_num == 0) {
        FS_LOGE("invalid max_dot_num=0: pixel_len=%u, type=0x%02X", pixel_len, info->data_type);
        return FS_ERR_BAD_PARAMS;
    }

    echo_data->data_offset = 0;
    echo_data->data_len = 0;
    pkt_num = (data_count % max_dot_num) ? (data_count / max_dot_num + 1) : (data_count / max_dot_num);

    for (j = 0; j < pkt_num; j++) {
        uint16_t data_len = 0;
        echo_data->data_sequence = next_start_sequence + j;

        if ((0 == j) && (info->group_info.vir_group_idx == 0)) {
            echo_data->frame_flag = FS_PCL_FRAME_FLAG_START;
        }
        else if ((j == (pkt_num - 1)) && (echo_data->data_sequence == max_sequence)) {
            echo_data->frame_flag = FS_PCL_FRAME_FLAG_END;
        }
        else {
            echo_data->frame_flag = FS_PCL_FRAME_FLAG_NORMAL;
        }

        if (remain_dot_num >= max_dot_num) {
            dot_num = max_dot_num;
        }
        else {
            dot_num = remain_dot_num;
        }

        remain_dot_num -= dot_num;
        echo_data->dot_num = dot_num;

        for (int k = 0; k < dot_num; k++) {
            uint32_t offset = (j * max_dot_num + k) * pixel_len;
            memcpy(echo_data->data + data_len, &info->data[offset], pixel_len);
            data_len += pixel_len;
        }

        if(circl_cnt > 3){
            fs_udp_send_data(&g_pcldata_udp, data_buf, data_len + sizeof(fs_protocol_head_t) + sizeof(fs_lidar_pcl_echo_data_t), __FUNCTION__);
        }
    }

    next_start_sequence = echo_data->data_sequence + 1;
    circl_cnt++;

    if (circl_cnt > 4) {
        circl_cnt = 4;
    }

    if (max_sequence < echo_data->data_sequence) {
        max_sequence = echo_data->data_sequence;
    }

    return FS_SUCCESS;
}

int32_t fs_send_gray_data_one_group(fs_lidar_data_info_t *info)
{
    uint16_t dot_num = 0;
    uint16_t max_dot_num = 0;
    int32_t  remain_dot_num;
    int32_t  j = 0;
    int32_t  pkt_num;
    uint8_t data_buf[FS_PROTOCOL_UDP_MTU_SIZE] = {0};
    uint16_t pixel_len  = info->group_info.pixel_len;
    uint32_t data_count = info->group_info.pixel_count;
    uint8_t  roi_index  = info->group_info.roi_index & 0x03;
    fs_lidar_pcl_echo_data_t *echo_data;
    fs_protocol_head_t *head_pkg;
    static uint16_t next_start_sequence = 0;

    if (pixel_udp_blocked_by_type(info->data_type)) {
        return FS_SUCCESS;
    }

    if (info->group_info.vir_group_idx == 0) {
        next_start_sequence = 0;
    }

    head_pkg  = (fs_protocol_head_t *)(data_buf);
    echo_data = (fs_lidar_pcl_echo_data_t *)(data_buf + sizeof(fs_protocol_head_t));
    get_head_pkg(head_pkg, info->data_type);
    remain_dot_num = data_count;
    max_dot_num = (FS_PROTOCOL_UDP_MTU_SIZE - sizeof(fs_protocol_head_t) - sizeof(fs_lidar_pcl_echo_data_t)) / pixel_len;

    echo_data->protocol_ver_minor = PROTOCOL_PCL_VER_MINOR;
    echo_data->frame_cnt          = info->frm_id;
    echo_data->width              = info->ROI_info[roi_index].width;
    echo_data->height             = info->ROI_info[roi_index].height;
    echo_data->max_bin_cnt        = info->max_bin_cnt;
    echo_data->ab_frame_idx       = ((info->ROI_info[roi_index].pcl_unit & 0x7F) << 1) | (info->ab_frame_idx & 0x01);
    echo_data->max_sub_frame_cnt  = info->max_sub_frame_cnt & 0x0F;
    echo_data->roi_index          = roi_index;
    echo_data->pixel_len          = pixel_len;
    echo_data->timestamp_base     = info->timestamp_base;

    switch ((uint32_t)info->data_type) {
        case DATA_TYPE_CICL_DATA: {
            echo_data->type = FS_CICL_DATA;
            break;
        }

        case DATA_TYPE_GL_DATA: {
            echo_data->type = FS_GL_DATA;
            break;
        }

        default: {
            echo_data->type = FS_CICL_DATA;
            break;
        }
    }

    echo_data->peak_type_2_1 = ((info->peak_type[1] & 0x0F) << 4) | (info->peak_type[0] & 0x0F);
    echo_data->peak_type_4_3 = ((info->peak_type[3] & 0x0F) << 4) | (info->peak_type[2] & 0x0F);

    if (max_dot_num == 0) {
        FS_LOGE("invalid max_dot_num=0: pixel_len=%u, type=0x%02X", pixel_len, info->data_type);
        return FS_ERR_BAD_PARAMS;
    }

    echo_data->data_offset = 0;
    echo_data->data_len = 0;
    pkt_num = (data_count % max_dot_num) ? (data_count / max_dot_num + 1) : (data_count / max_dot_num);

    for (j = 0; j < pkt_num; j++) {
        uint16_t data_len = 0;
        echo_data->data_sequence = next_start_sequence + j;

        if ((0 == j) && (info->group_info.vir_group_idx == 0)) {
            echo_data->frame_flag = FS_PCL_FRAME_FLAG_START;
        }
        else if ((j == (pkt_num - 1)) && (info->group_info.vir_group_idx == (info->group_cnt - 1))) {
            echo_data->frame_flag = FS_PCL_FRAME_FLAG_END;
        }
        else {
            echo_data->frame_flag = FS_PCL_FRAME_FLAG_NORMAL;
        }

        if (remain_dot_num >= max_dot_num) {
            dot_num = max_dot_num;
        }
        else {
            dot_num = remain_dot_num;
        }

        remain_dot_num -= dot_num;
        echo_data->dot_num = dot_num;

        for (int k = 0; k < dot_num; k++) {
            uint32_t offset = (j * max_dot_num + k) * pixel_len;
            memcpy(echo_data->data + data_len, &info->data[offset], pixel_len);
            data_len += pixel_len;
        }

        fs_udp_send_data(&g_pcldata_udp, data_buf, data_len + sizeof(fs_protocol_head_t) + sizeof(fs_lidar_pcl_echo_data_t), __FUNCTION__);
    }

    next_start_sequence = echo_data->data_sequence + 1;
    return FS_SUCCESS;
}
