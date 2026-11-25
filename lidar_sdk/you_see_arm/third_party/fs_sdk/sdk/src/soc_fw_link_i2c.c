#include "soc_fw_link_i2c.h"

#include <stddef.h>

#include "board_cfg.h"
#include "drivers/i2c/i2c_master.h"
#include "fs_top_module_cfg.h"
#include "utils/fs_err.h"

int soc_fw_link_i2c_send_timesync_push(const uint8_t *payload, uint16_t payload_len)
{
    int ret;

    if ((payload_len != 0u) && (payload == NULL)) {
        return FS_ERR_BAD_PARAMS;
    }

    ret = i2c_master_init();
    if (ret != FS_SUCCESS) {
        return ret;
    }

    return i2c_master_write((uint8_t)MCU_CMD_I2C_BUS,
                            (uint8_t)FS_CFG_I2C_TIME_SYNC_SLAVE_ADDR,
                            (uint16_t)FS_CFG_I2C_TIME_SYNC_REG_PUSH,
                            1u,
                            (uint8_t *)payload,
                            payload_len);
}
