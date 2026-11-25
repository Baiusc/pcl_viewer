#include <stdint.h>
#include <sys/unistd.h>

#include "fs_err.h"
#include "fs_hw.h"
#include "mcu_protocol.h"
#include "soc_system_manager.h"

#undef LOG_TAG
#define LOG_TAG "soc.hw"

void fs_hw_usleep(int32_t us)
{
    usleep(us);
    return;
}

void fs_hw_msleep(int32_t ms)
{
    usleep(ms * 1000);
    return;
}

int32_t fs_hw_reset(fs_sensor_tx_rx_e dev_id)
{
    int err = FS_SUCCESS;

    switch (dev_id) {
        case FS_RX_FL6X32_1:
        case FS_RX_FL6X32_ALL: {
            err = mcu_prot_lidarchip_hw_reset(MCU_PROT_DEV_FL6X32_1 + (dev_id - FS_RX_FL6X32_1));
            if (err != FS_SUCCESS) {
                FS_LOGE("FL6X32 hardware reset failed: %d", err);
            }
            break;
        }

        case FS_TX_FL9003_1:
        case FS_TX_FL9003_2:
        case FS_TX_FL9003_3:
        case FS_TX_FL9003_ALL:{
            err = mcu_prot_lidarchip_hw_reset(MCU_PROT_DEV_FL9003_1 + (dev_id - FS_TX_FL9003_1));
            if (err != FS_SUCCESS) {
                FS_LOGE("FL9003 hardware reset failed: %d", err);
            }
            break;
        }

        case FS_TX_FL9103_1:
        case FS_TX_FL9103_2:
        case FS_TX_FL9103_ALL: {
            err = mcu_prot_lidarchip_hw_reset(MCU_PROT_DEV_FL9103_1 + (dev_id - FS_TX_FL9103_1));
            if (err != FS_SUCCESS) {
                FS_LOGE("FL9103 hardware reset failed: %d", err);
            }
            break;
        }

        default: {
            FS_LOGE("Unsupported device for hardware reset: %d", dev_id);
            err = FS_ERR_BAD_PARAMS;
            break;
        }
    }

    return err;
}

int32_t fs_hw_get_adc(adc_type_e type, adc_data_e fmt, int32_t *io_value, uint8_t *io_cnt, uint32_t *o_raw_adc_mV)
{
    int err = FS_SUCCESS;

    if (!io_value || !io_cnt) {
        return FS_ERR_BAD_PARAMS;
    }

    err = mcu_prot_get_adc((uint8_t)type, (uint8_t)fmt, io_value, io_cnt, o_raw_adc_mV);
    if (err != FS_SUCCESS) {
        FS_LOGE("Failed to get ADC data: type=%d, fmt=%d, err=%d", type, fmt, err);
    }

    return err;
}

int32_t fs_hw_set_vpp_volt(float volt, uint32_t *o_dac_code)
{
    return mcu_prot_set_vpp(volt, o_dac_code);
}

int32_t fs_hw_set_vbd_volt(float volt, uint32_t *o_dac_code)
{
    return mcu_prot_set_vbd(volt, o_dac_code);
}

int32_t fs_hw_enable_vbd(bool enable)
{
    int32_t err = FS_SUCCESS;

    FS_LOGD("fs_hw_enable_vbd: %s", enable ? "enable" : "disable");
    err = mcu_prot_set_enable_vbd(enable, false);
    if (err != FS_SUCCESS) {
        FS_LOGE("set vbd state failed: %d", err);
    }

    return err;
}

int32_t fs_hw_enable_vpp(bool enable)
{
    int32_t err = FS_SUCCESS;

    FS_LOGD("fs_hw_enable_vpp: %s", enable ? "enable" : "disable");
    err = mcu_prot_set_enable_vpp(enable, false);
    if (err != FS_SUCCESS) {
        FS_LOGE("set vpp state failed: %d", err);
    }

    return err;
}

int32_t fs_hw_enable_temp_comp(bool enable)
{
    int32_t err = FS_SUCCESS;
    err = mcu_prot_set_temp_comp(enable);

    if (err != FS_SUCCESS) {
        FS_LOGE("set temp compensation state failed: %d", err);
    }

    return err;
}

int32_t fs_hw_enable_voltage_calib(bool enable)
{
    int32_t err = FS_SUCCESS;
    err = mcu_prot_set_voltage_calib(enable);

    if (err != FS_SUCCESS) {
        FS_LOGE("set voltage calibration state failed: %d", err);
    }

    return err;
}

int32_t fs_hw_store_adc_calib(adc_type_e type, float k, float b)
{
    int err = mcu_prot_store_adc_calib((uint8_t)type, k, b);

    if (err != FS_SUCCESS) {
        FS_LOGE("store adc calib failed: type=%d, err=%d", type, err);
    }

    return err;
}

int32_t fs_hw_read_adc_calib(adc_type_e type, float *k, float *b)
{
    int err;

    if (!k || !b) {
        return FS_ERR_BAD_PARAMS;
    }

    err = mcu_prot_read_adc_calib((uint8_t)type, k, b);
    if (err != FS_SUCCESS) {
        FS_LOGE("read adc calib failed: type=%d, err=%d", type, err);
    }

    return err;
}

int32_t fs_hw_remove_adc_calib(adc_type_e type)
{
    int err = mcu_prot_remove_adc_calib((uint8_t)type);

    if (err != FS_SUCCESS) {
        FS_LOGE("remove adc calib failed: type=%d, err=%d", type, err);
    }

    return err;
}

int32_t fs_hw_store_dac_calib(dac_type_e type, float k, float b)
{
    int err = mcu_prot_store_dac_calib((uint8_t)type, k, b);

    if (err != FS_SUCCESS) {
        FS_LOGE("store dac calib failed: type=%d, err=%d", type, err);
    }

    return err;
}

int32_t fs_hw_read_dac_calib(dac_type_e type, float *k, float *b)
{
    int err;

    if (!k || !b) {
        return FS_ERR_BAD_PARAMS;
    }

    err = mcu_prot_read_dac_calib((uint8_t)type, k, b);
    if (err != FS_SUCCESS) {
        FS_LOGE("read dac calib failed: type=%d, err=%d", type, err);
    }

    return err;
}

int32_t fs_hw_remove_dac_calib(dac_type_e type)
{
    int err = mcu_prot_remove_dac_calib((uint8_t)type);

    if (err != FS_SUCCESS) {
        FS_LOGE("remove dac calib failed: type=%d, err=%d", type, err);
    }

    return err;
}

int32_t fs_hw_init(void)
{
    int32_t err = FS_SUCCESS;
    return err;
}

int32_t fs_hw_calib_volt_save_begin(void)
{
    fw_if_reject_reason_t reason = FW_IF_REJECT_NONE;

    return soc_mgr_program_token_acquire(&reason);
}

int32_t fs_hw_calib_volt_save_end(void)
{
    fw_if_reject_reason_t reason = FW_IF_REJECT_NONE;

    return soc_mgr_program_token_release(&reason);
}
