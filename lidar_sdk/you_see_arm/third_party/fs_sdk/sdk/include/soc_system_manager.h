#ifndef SOC_SYSTEM_MANAGER_H
#define SOC_SYSTEM_MANAGER_H

#include <stdbool.h>
#include <stdint.h>

#include "fs_sdk_api.h"
#include "fs_pointcloud_timesync_caps.h"
#include "fw_if_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SOC_MGR_STATE_INIT = 0,
    SOC_MGR_STATE_IDLE,
    SOC_MGR_STATE_NORMAL_RUN,
    SOC_MGR_STATE_PROGRAM_ENTERING,
    SOC_MGR_STATE_PROGRAM,
    SOC_MGR_STATE_PROGRAM_EXITING,
    SOC_MGR_STATE_UPDATE_PREPARE,
    SOC_MGR_STATE_FAULT
} soc_mgr_state_t;

typedef enum {
    SOC_MGR_OP_NONE = 0,
    SOC_MGR_OP_MODE_TRANSITION,
    SOC_MGR_OP_PROGRAM_WINDOW,
    SOC_MGR_OP_CFG_CALI_TRANSFER,
    SOC_MGR_OP_RUNTIME_MUTATION,
    SOC_MGR_OP_UPGRADE_PREPARE
} soc_mgr_operation_t;

void soc_mgr_init(void);

soc_mgr_state_t soc_mgr_get_state(void);
soc_mgr_operation_t soc_mgr_get_active_operation(void);

bool soc_mgr_program_token_held(void);
bool soc_mgr_mode_transition_busy(void);
bool soc_mgr_cfg_cali_transfer_busy(void);

int32_t soc_mgr_time_sync_stop(void);
int32_t soc_mgr_time_sync_start_for_mode(fs_sdk_work_mode_e work_mode);
bool soc_mgr_time_sync_is_running(void);

int32_t soc_mgr_setup_mode(fs_sdk_work_mode_e work_mode);

fs_pcl_trigger_mode_t soc_mgr_get_pcl_trigger_mode(void);
int32_t soc_mgr_set_pcl_trigger_mode(fs_pcl_trigger_mode_t mode);

int32_t soc_mgr_runtime_mutation_begin(void);
void soc_mgr_runtime_mutation_end(void);

int32_t soc_mgr_program_token_acquire(fw_if_reject_reason_t *out_reason);
int32_t soc_mgr_program_token_release(fw_if_reject_reason_t *out_reason);

int32_t soc_mgr_check_cfg_cali_write(int flash_data_type);
int32_t soc_mgr_cfg_cali_transfer_begin(int flash_data_type);
void soc_mgr_cfg_cali_transfer_end(void);

int32_t soc_mgr_upgrade_enter(void);
void soc_mgr_upgrade_leave(int32_t upgrade_err);

/* ---- Module output gates (sole authority — no external switches) ---- */

bool soc_mgr_pcl_udp_allowed(void);
bool soc_mgr_imu_udp_allowed(void);
bool soc_mgr_imu_forward_allowed(void);

#ifdef __cplusplus
}
#endif

#endif /* SOC_SYSTEM_MANAGER_H */
