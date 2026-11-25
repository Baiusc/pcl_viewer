#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "fs_err.h"
#include "fs_util.h"
#include "fs_hw.h"
#include "mcu_protocol.h"
#include "soc_sdk_internal.h"
#include "soc_system_manager.h"
#include "FL6X32.h"

#define FL6X32_REG_SCAN_TRIGGER_MODE  0x00009004u
#define FL6X32_SCAN_TRIGGER_EXT_BIT   0x00000001u

#undef  LOG_TAG
#define LOG_TAG "soc.FL6X32"

#ifndef FS_PROTO_BUF_LEN_MAX
// 与 sensor 通信的单个包缓存长度
#define FS_PROTO_BUF_LEN_MAX (sizeof(FL6X32_rbuf_t) + sizeof(FL6X32_rbuf_tail_t) + 256)
#endif
#define FL6X32_MUL_WRITE_SAFTY_CNT   8  // 寄存器连写安全最大支持地址个数

#define FS_PROTO_HEAD(cmd)               (uint16_t)(((cmd & 0xFF) << 8) | ((~cmd) & 0xFF))

// 协议命令头
typedef enum {
    FL6X32_PROTO_HEAD_WRITE     = 0x61, // 单次写(安全, CRC 校验错误时, 数据不会写入寄存器)
    // FL6X32_PROTO_HEAD_MUL_WRITE = 0x71, // 连续写(不安全, 即使 CRC 校验错误, 数据仍然会被写入寄存器)
    FL6X32_PROTO_HEAD_MUL_WRITE = 0x5A, // 连续写(8 个地址内安全)
    FL6X32_PROTO_HEAD_READ      = 0x81, // 单次读
    FL6X32_PROTO_HEAD_MUL_READ  = 0x91, // 连续读
} ENUM_FL6X32_PROTO_HEAD;

#pragma pack(push)
#pragma pack(1) // one byte-aligned

typedef struct {
    uint16_t hdr;     // protocol header
    uint32_t addr;    // register address
    uint16_t dlen;    // len of data that need to read
    uint16_t wcrc_o;  // crc of hdr to dlen
    uint16_t wcrc_i;  // readback crc to compare with wcrc_o
    uint32_t data[0]; // register data our read
} FL6X32_rbuf_t;

typedef struct {
    uint16_t rcrc; // crc of data
} FL6X32_rbuf_tail_t;

typedef struct {
    uint16_t hdr;     // protocol header
    uint32_t addr;    // register address
    uint16_t dlen;    // len of data that need to write
    uint32_t data[0]; // register data then need to write
} FL6X32_wbuf_t;

typedef struct {
    uint16_t wcrc_o; // crc of hdr to data
    uint16_t wcrc_i; // readback crc to compare with wcrc_o
} FL6X32_wbuf_tail_t;

typedef struct {
    uint16_t hdr;    // protocol header
    uint32_t addr;   // register address
    uint16_t wcrc_o; // crc of hdr to addr
} FL6X32_rreg_t;

typedef struct {
    uint16_t wcrc_i; // readback crc to compare with wcrc_o
    uint32_t data;   // register data
    uint16_t rcrc;   // crc of data
} FL6X32_rreg_tail_t;

typedef struct {
    uint16_t hdr;    // protocol header
    uint32_t addr;   // register address
    uint32_t data;   // write data
    uint16_t wcrc_o; // crc of hdr to data
} FL6X32_wreg_t;

typedef struct {
    uint16_t wcrc_i; // readback crc to compare with wcrc_o
} FL6X32_wreg_tail_t;

#pragma pack(pop)

static int16_t m_dev_id[FS_SENSOR_RX_MAX_DEV_CNT];

int32_t FL6X32_read_register(fs_sensor_tx_rx_e idx, uint32_t addr, uint32_t *data, uint32_t count)
{
    int32_t err = FS_SUCCESS;
    int32_t i;

    for (i = 0; i < (int32_t)count; i++) {
        err = mcu_prot_get_lidarchip_reg(addr + i * sizeof(uint32_t), &data[i]);

        if (err != FS_SUCCESS) {
            break;
        }
    }

    return err;
}

int32_t FL6X32_write_register(fs_sensor_tx_rx_e idx, uint32_t addr, const uint32_t *data, uint32_t count)
{
    int32_t err = FS_SUCCESS;
    int32_t i;

    for (i = 0; i < (int32_t)count; i++) {
        err = mcu_prot_set_lidarchip_reg(addr + i * sizeof(uint32_t), data[i]);

        if (err != FS_SUCCESS) {
            break;
        }
    }

    return err;
}

static int32_t FL6X32_read_buffer(fs_sensor_tx_rx_e idx, uint32_t addr, void *data, uint32_t *dlen)
{
    int32_t err = FS_ERR_UNSUPPORT;
    return err;
}

static int32_t FL6X32_write_buffer(fs_sensor_tx_rx_e idx, uint32_t addr, const void *data, uint32_t dlen)
{
    int32_t err = FS_ERR_UNSUPPORT;
    return err;
}

static int32_t FL6X32_write_cfg(const rx_reg_t *cfg, uint16_t cfg_cnt)
{
    int32_t err = FS_SUCCESS;
    int32_t i;

    for (i = 0; i < cfg_cnt; i++) {
        if (cfg[i].addr == RX_REG_DELAY_US) {
            // FS_LOGI("Delay %u us", cfg[i].data);
            fs_hw_usleep(cfg[i].data);
            continue;
        }

        // FS_LOGI("Write Reg: addr=0x%08X, val=0x%08X", cfg[i].addr, cfg[i].data);
        err = FL6X32_write_register(FS_RX_FL6X32_1, cfg[i].addr, &cfg[i].data, 1);

        if (err != FS_SUCCESS) {
            FS_LOGE("%d - wreg(addr=0x%08x, val=0x%08x): err=%d", i, cfg[i].addr, cfg[i].data, err);
            break;
        }

        if (cfg[i].delay > 0) {
            fs_hw_usleep(cfg[i].delay);
        }
    }

    return err;
}

static int32_t FL6X32_load_trim(fs_sensor_tx_rx_e idx, fs_rx_trim_t *trim, char *sn_buf, uint32_t buf_len)
{
    int32_t err = FS_SUCCESS;
    uint32_t val, SN[4];
    char string[16] = {'\0'};
    uint8_t num;
    uint16_t temp_vbd_code;
    float    temp_vbd_radio;
    float    temp_cali_tj;
    float    temp_vbd_V, cali_vbd_25C;
    // CHECK_PTR(trim);

    val = 0x00000012;
    CHECK_ERR(FL6X32_write_register(idx, 0x0000D000, &val, 1));
    val = 0x00000000;
    CHECK_ERR(FL6X32_write_register(idx, 0x0000D008, &val, 1));
    val = 0x00000001;
    CHECK_ERR(FL6X32_write_register(idx, 0x0000D004, &val, 1));
    fs_hw_msleep(1); // wait for load done
    CHECK_ERR(FL6X32_read_register(idx, 0x0000D010, &val, 1)); // read load status

    if ((val & 0x00000001) != 1) {
        FS_LOGE("load trim failed");
        return FS_ERR_INTERNAL;
    }

    CHECK_ERR(FL6X32_read_register(idx, 0x0000E000, &val, 1));

    if ((val & 0xFF) != 0x5A) {
        FS_LOGE("load trim failed");
        return FS_ERR_INTERNAL;
    }

    SN[0] = val;
    val = 0x00000006;
    CHECK_ERR(FL6X32_write_register(idx, 0x0000D004, &val, 1)); // clear load status
    CHECK_ERR(FL6X32_read_register(idx, 0x0000E004, &SN[1], 3));

    if (((SN[1] >> 8) & 0xFF) == 0x01) {
        FS_LOGD("0x0000E004 bit[15:8] = 0x01...");
        val = 0x00000007;
        CHECK_ERR(FL6X32_write_register(idx, 0x0000E02C, &val, 1));
        val = 0x00000715;
        CHECK_ERR(FL6X32_write_register(idx, 0x0000E03C, &val, 1));
    }

    if (sn_buf && buf_len > 0) {
        snprintf(sn_buf, buf_len, "%c%c%c%c%c%c-%02u_%02u-%02u(%c-%02X-%01X-%01X-%02X-%01X)",
                ( SN[1] >> 16) & 0xFF, (SN[1] >> 24) & 0xFF, (SN[2] >>  0) & 0xFF, (SN[2] >>  8) & 0xFF, (SN[2] >> 16) & 0xFF, (SN[2] >> 24) & 0xFF,
                ( SN[3] >>  0) & 0xFF, (SN[3] >>  8) & 0xFF, (SN[3] >> 16) & 0xFFFF,
                ((SN[1] >>  0) & 0xFF) - 1 + 'A', (SN[1] >> 8) & 0xFF, (SN[0] >> 16) & 0x03, (SN[0] >> 18) & 0x01,
                ( SN[0] >> 19) & 0x1F, (SN[0] >> 24) & 0x0F);
        FS_LOGD("load trim success: SN=%s", sn_buf);
    }

    num = SN[3] & 0xFF;
    snprintf(string, sizeof(string), "%c%c%c%c%c%c", (SN[1] >> 16) & 0xFF, (SN[1] >> 24) & 0xFF, (SN[2] >> 0) & 0xFF, 
                (SN[2] >> 8) & 0xFF, (SN[2] >> 16) & 0xFF, (SN[2] >> 24) & 0xFF);

    if (     (strcmp(string, "U0RU21") == 0)
          || (strcmp(string, "U0RU22") == 0)
          || (strcmp(string, "U0RU23") == 0)
          || (strcmp(string, "U0RS50") == 0)
          || (strcmp(string, "U0RS51") == 0)
          || (strcmp(string, "U0RS62") == 0)
          || (strcmp(string, "U0NU55") == 0)
          || (strcmp(string, "UPG372") == 0)) {
        temp_vbd_radio = -0.0365f;
    }
    else if ((strcmp(string, "U0NU56") == 0)
          || (strcmp(string, "U10L16") == 0)
          || (strcmp(string, "U10L18") == 0)
          || (strcmp(string, "U10L19") == 0)
          || (strcmp(string, "U10L21") == 0)
          || (strcmp(string, "UPG302") == 0)) {
        temp_vbd_radio = -0.0280f;
    }
    else if (strcmp(string, "UPF900") == 0) {
        if (((num >= 3) && (num <= 8)) || ((num >= 12) && (num <= 14))) {
            temp_vbd_radio = -0.0365f;
        }
        else {
            temp_vbd_radio = -0.0280f;
        }
    }
    else {
        temp_vbd_radio = -0.0365f;
    }

    // 获取芯片校准 vbd 时的环境温度
    CHECK_ERR(FL6X32_read_register(idx, 0x0000E010, &val, 1));
    // bit[6:0]为温度值，bit[7]为温度值符号位(0表示正温，1表示负温)，20.0 是基准温度
    temp_cali_tj = ((float)(val & 0x7F) * (((val >> 7) & 0x01) ? (-1) : (1))) / 10 + 20.0f;
    // 获取芯片校准的 vbd code
    CHECK_ERR(FL6X32_read_register(idx, 0x0000E040, &val, 1));
    temp_vbd_code = val & 0xFFFF;
    // 转换单位为伏特(V), 带符号(VBD 是负压)
    temp_vbd_V = -(temp_vbd_code * 0.05f + 16.6f + 1.0f);
    cali_vbd_25C = temp_vbd_V + temp_vbd_radio * (temp_cali_tj - 25.0f);
    FS_LOGD("temp_vbd_radio=%.4fV/C, temp_cali_tj=%.2fC, cali_vbd_25C=%.4fV(temp_vbd_code=0x%04X)", 
            temp_vbd_radio, temp_cali_tj, cali_vbd_25C, temp_vbd_code);

    if (trim != NULL) {
        trim->temp_vbd_k   = temp_vbd_radio;
        trim->cali_vbd_25C = cali_vbd_25C;
    }

    return err;
}

static int32_t FL6X32_check_alive(fs_rx_t *rx)
{
    int32_t err = FS_SUCCESS;
#if 0
    const rx_reg_t *check_alive_cfg;
    uint16_t cfg_cnt = 0;
    CHECK_PTR(rx);
    CHECK_PTR(rx->cfg);
    CHECK_ERR(rx_cfg_get_stage_cfg(rx->cfg, FS_CFG_STAGE_CHECK_ALIVE, &check_alive_cfg, &cfg_cnt));
    CHECK_ERR(FL6X32_write_cfg(check_alive_cfg, cfg_cnt, NULL, NULL));
#endif
    return err;
}

static int32_t FL6X32_get_status(fs_rx_t *rx, void *status)
{
    int32_t err = FS_SUCCESS;
#if 0
    const rx_reg_t *get_status_cfg;
    uint16_t cfg_cnt = 0;
    uint32_t buffer[64];
    uint32_t buf_len = sizeof(buffer);
    CHECK_PTR(rx);
    CHECK_PTR(rx->cfg);
    CHECK_ERR(rx_cfg_get_stage_cfg(rx->cfg, FS_CFG_STAGE_READ_STATUS, &get_status_cfg, &cfg_cnt));
    CHECK_ERR(FL6X32_write_cfg(get_status_cfg, cfg_cnt, buffer, &buf_len));
#endif
    return err;
}

static int32_t FL6X32_reset(fs_rx_t *rx, bool soft_reset)
{
    int32_t err = FS_SUCCESS;

    if (soft_reset) {
        const rx_reg_t *reset_cfg;
        uint16_t cfg_cnt = 0;
        CHECK_PTR(rx);
        CHECK_PTR(rx->cfg);
        CHECK_ERR(rx_cfg_get_stage_cfg(rx->cfg, RX_STAGE_INIT, &reset_cfg, &cfg_cnt));
        FS_LOGI("FL6X32 soft reset: writing %u regs", cfg_cnt);
        CHECK_ERR(FL6X32_write_cfg(reset_cfg, cfg_cnt));
    }
    else {
        err = fs_hw_reset(FS_RX_FL6X32_1);
    }

    return err;
}

static int32_t FL6X32_wakeup(fs_rx_t *rx)
{
    int32_t err = FS_SUCCESS;
    const rx_reg_t *wakeup_cfg;
    uint16_t cfg_cnt = 0;
    CHECK_PTR(rx);
    CHECK_PTR(rx->cfg);
    CHECK_ERR(rx_cfg_get_stage_cfg(rx->cfg, RX_STAGE_WAKEUP, &wakeup_cfg, &cfg_cnt));
    FS_LOGI("FL6X32 wakeup: writing %u regs", cfg_cnt);
    CHECK_ERR(FL6X32_write_cfg(wakeup_cfg, cfg_cnt));
    return err;
}

static int32_t FL6X32_sleep(fs_rx_t *rx)
{
    int32_t err = FS_SUCCESS;
    const rx_reg_t *sleep_cfg;
    uint16_t cfg_cnt = 0;
    CHECK_PTR(rx);
    CHECK_PTR(rx->cfg);
    CHECK_ERR(rx_cfg_get_stage_cfg(rx->cfg, RX_STAGE_SLEEP, &sleep_cfg, &cfg_cnt));
    FS_LOGI("FL6X32 sleep: writing %u regs", cfg_cnt);
    CHECK_ERR(FL6X32_write_cfg(sleep_cfg, cfg_cnt));
    return err;
}

static int32_t FL6X32_apply_scan_trigger_mode(void)
{
    uint32_t reg_val = 0u;
    fs_sdk_work_mode_e work_mode = fs_sdk_get_work_mode();
    fs_pcl_trigger_mode_t trig_mode = soc_mgr_get_pcl_trigger_mode();
    int32_t err;

    err = FL6X32_read_register(FS_RX_FL6X32_1, FL6X32_REG_SCAN_TRIGGER_MODE, &reg_val, 1);
    if (err != FS_SUCCESS) {
        FS_LOGE("Failed to read scan trigger mode reg 0x%08x: err=%d",
                FL6X32_REG_SCAN_TRIGGER_MODE, err);
        return err;
    }

    if (work_mode == FS_SDK_WORK_MODE_PCL && trig_mode == FS_PCL_TRIGGER_EXTERNAL) {
        reg_val |= FL6X32_SCAN_TRIGGER_EXT_BIT;
    } else {
        reg_val &= ~FL6X32_SCAN_TRIGGER_EXT_BIT;
    }

    err = FL6X32_write_register(FS_RX_FL6X32_1, FL6X32_REG_SCAN_TRIGGER_MODE, &reg_val, 1);
    if (err != FS_SUCCESS) {
        FS_LOGE("Failed to write scan trigger mode reg 0x%08x val=0x%08x: err=%d",
                FL6X32_REG_SCAN_TRIGGER_MODE, reg_val, err);
        return err;
    }

    return FS_SUCCESS;
}

static int32_t FL6X32_setup_cfg(fs_rx_t *rx)
{
    int32_t err = FS_SUCCESS;
    const rx_reg_t *setup_cfg;
    uint16_t cfg_cnt = 0;
    CHECK_PTR(rx);
    CHECK_PTR(rx->cfg);
    CHECK_ERR(rx_cfg_get_stage_cfg(rx->cfg, RX_STAGE_CONFIG, &setup_cfg, &cfg_cnt));
    FS_LOGI("FL6X32 setup cfg: writing %u regs", cfg_cnt);
    CHECK_ERR(FL6X32_write_cfg(setup_cfg, cfg_cnt));
    FL6X32_load_trim(FS_RX_FL6X32_1, NULL, NULL, 0); // allow to fail
    /* PCL: external RX_FRAME_SYNC input; non-PCL: internal trigger (FR-006 / arch §3.1). */
    CHECK_ERR(FL6X32_apply_scan_trigger_mode());
    return err;
}

static int32_t FL6X32_start_scan(fs_rx_t *rx)
{
    int32_t err = FS_SUCCESS;
    const rx_reg_t *start_cfg;
    uint16_t cfg_cnt = 0;
    CHECK_PTR(rx);
    CHECK_PTR(rx->cfg);
    CHECK_ERR(rx_cfg_get_stage_cfg(rx->cfg, RX_STAGE_START, &start_cfg, &cfg_cnt));
    FS_LOGI("FL6X32 start scan: writing %u regs", cfg_cnt);
    CHECK_ERR(FL6X32_write_cfg(start_cfg, cfg_cnt));
    return err;
}

static int32_t FL6X32_stop_scan(fs_rx_t *rx)
{
    int32_t err = FS_SUCCESS;
    const rx_reg_t *stop_cfg;
    uint16_t cfg_cnt = 0;
    CHECK_PTR(rx);
    CHECK_PTR(rx->cfg);
    CHECK_ERR(rx_cfg_get_stage_cfg(rx->cfg, RX_STAGE_STOP, &stop_cfg, &cfg_cnt));
    FS_LOGI("FL6X32 stop scan: writing %u regs", cfg_cnt);
    CHECK_ERR(FL6X32_write_cfg(stop_cfg, cfg_cnt));
    return err;
}

static int32_t FL6X32_detect_device(fs_sensor_tx_rx_e idx)
{
    int32_t err = FS_SUCCESS;
    int32_t try_cnt = 3;

    do {
        uint32_t chip_id = 0x00;
        CONTINUE_ON_ERR(err = FL6X32_read_register(idx, 0x2000, &chip_id, 1));

        if (((chip_id >> 8) & 0xFFFF) == 30981) {
            err = FS_SUCCESS;
            break;
        }

        err = FS_ERR_NO_DEVICE;
    }
    while (--try_cnt > 0);

    return err;
}

int32_t FL6X32_deinit(fs_rx_t *rx)
{
    int32_t err = FS_SUCCESS;
    return err;
}

int32_t FL6X32_init(fs_rx_t *rx, const int16_t *dev_id, uint8_t dev_cnt)
{
    int32_t err = FS_SUCCESS;
    const rx_reg_t *init_cfg;
    uint16_t cfg_cnt = 0;
    CHECK_PTR(rx);
    // CHECK_PTR(rx->cfg);
    rx->has_load_cfg = false;
    err = rx_cfg_get_stage_cfg(rx->cfg, RX_STAGE_INIT, &init_cfg, &cfg_cnt);

    if (err) {
        // FS_LOGE("Failed to get init cfg: %d, skip!", err);
        err = FS_SUCCESS;
    }
    else {
        // CHECK_ERR(FL6X32_write_cfg(init_cfg, cfg_cnt));
        rx->has_load_cfg = true;
    }

    // FL6X32 寄存器的位宽
    rx->reg_width = FS_SENSOR_REG_WIDTH_32;
    // 对外提供的函数接口
    rx->op.check_alive      = FL6X32_check_alive;
    rx->op.get_status       = FL6X32_get_status;
    rx->op.reset            = FL6X32_reset;
    rx->op.wakeup           = FL6X32_wakeup;
    rx->op.sleep            = FL6X32_sleep;
    rx->op.setup_cfg        = FL6X32_setup_cfg;
    rx->op.start_scan       = FL6X32_start_scan;
    rx->op.stop_scan        = FL6X32_stop_scan;
    rx->op.read_register    = FL6X32_read_register;
    rx->op.write_register   = FL6X32_write_register;
    rx->op.read_buffer      = FL6X32_read_buffer;
    rx->op.write_buffer     = FL6X32_write_buffer;

    for (int32_t i = 0; i < FS_SENSOR_RX_MAX_DEV_CNT; i++) {
        rx->trim[i].cali_vbd_25C = -20.0f;
        rx->trim[i].temp_vbd_k   = -0.0365f;
        rx->dev_id[i] = -1;
        snprintf(rx->SN[i], sizeof(rx->SN[i]), "%s", "unknown");
    }

    if ((dev_id != NULL) && (dev_cnt > 0)) {
        int32_t detected = 0;

        if (dev_cnt > sizeof(rx->dev_id) / sizeof(rx->dev_id[0])) {
            FS_LOGE("dev_cnt is too large: dev_cnt(%d) > %d", dev_cnt, (int32_t)(sizeof(rx->dev_id) / sizeof(rx->dev_id[0])));
            return FS_ERR_BAD_PARAMS;
        }

        for (int32_t i = 0; i < dev_cnt; i++) {
            int32_t ret;

            if (dev_id[i] < 0) {
                FS_LOGE("invalid dev_id: dev_id[%d] = %d", i, dev_id[i]);
                return FS_ERR_BAD_PARAMS;
            }

            ret = FL6X32_detect_device(FS_RX_FL6X32_1 + dev_id[i]);

            if (ret != FS_SUCCESS) {
                FS_LOGE("Failed to detect FL6X32: dev_id[%d] = %d", i, dev_id[i]);
                continue;
            }

            FS_LOGD("Successfully detected FL6X32: dev_id[%d] = %d", i, dev_id[i]);
            FL6X32_load_trim(FS_RX_FL6X32_1, &rx->trim[detected], rx->SN[detected], sizeof(rx->SN[0])); // allow to fail
            rx->dev_id[detected++] = dev_id[i];
        }

        if (detected == 0) {
            FS_LOGE("No valid FL6X32 devices found");
            return FS_ERR_NO_DEVICE;
        }
        // else {
        //     FS_LOGD("Successfully detected FL6X32 devices count: %d", detected);
        // }
    }

    for (int32_t i = 0; i < (int32_t)(sizeof(m_dev_id) / sizeof(m_dev_id[0])); i++) {
        if (i < (int32_t)(sizeof(rx->dev_id) / sizeof(rx->dev_id[0]))) {
            m_dev_id[i] = rx->dev_id[i];
        }
        else {
            m_dev_id[i] = -1;
        }
    }

    return err;
}
