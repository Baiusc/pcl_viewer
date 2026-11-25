#include "soc_system_manager.h"

#include <pthread.h>

#include "soc_sdk_internal.h"
#include "mcu_protocol.h"
#include "fw_if_system_state.h"
#include "utils/fs_err.h"
#include "utils/fs_log.h"

#undef LOG_TAG
#define LOG_TAG "soc.mgr"

#include "soc_time_sync_module.h"
#include "fs_timesync_ctrl.h"
#include "fs_pointcloud_timesync_caps.h"
#include "fs_top_module_cfg.h"

#include <errno.h>
#include <fcntl.h>
#include <unistd.h>

#include "drivers/fs_gpio_trig_ioctl.h"
#include "drivers/fs_frame_sync_ioctl.h"

static int32_t soc_mgr_send_timesync_ctrl_and_wait_ack(uint8_t ctrl)
{
    int32_t ret;

    ret = mcu_prot_time_sync_ctrl(ctrl);
    if (ret != FS_SUCCESS) {
        FS_LOGE("TimeSync ctrl ACK failed: ctrl=%u ret=%d", ctrl, ret);
        return ret;
    }

    return FS_SUCCESS;
}

/* ---- pointcloud helper ---- */
static bool soc_mgr_is_pointcloud_mode(fs_sdk_work_mode_e work_mode)
{
    return (work_mode == FS_SDK_WORK_MODE_PCL);
}

static pthread_mutex_t g_mgr_mutex = PTHREAD_MUTEX_INITIALIZER;
static soc_mgr_state_t g_mgr_state = SOC_MGR_STATE_INIT;
static soc_mgr_operation_t g_active_op = SOC_MGR_OP_NONE;
static bool g_program_token = false;
static bool g_cfg_cali_transfer = false;
static fs_sdk_work_mode_e g_program_saved_mode = FS_SDK_WORK_MODE_IDLE;

static pthread_t g_time_sync_thread;
static volatile int g_time_sync_running = 0;

/* ---- caps (lazy init by open-then-close probe) ---- */
static fs_pointcloud_timesync_caps_t g_pointcloud_caps;
static bool g_pointcloud_caps_initialized = false;
static fs_pcl_trigger_mode_t g_pcl_trigger_mode = FS_PCL_TRIGGER_EXTERNAL;

static bool soc_mgr_pointcloud_uses_internal_trigger(void)
{
    return g_pointcloud_caps.trigger_mode == FS_PCL_TRIGGER_INTERNAL;
}

fs_pcl_trigger_mode_t soc_mgr_get_pcl_trigger_mode(void)
{
    fs_pcl_trigger_mode_t mode;

    pthread_mutex_lock(&g_mgr_mutex);
    mode = g_pcl_trigger_mode;
    pthread_mutex_unlock(&g_mgr_mutex);
    return mode;
}

int32_t soc_mgr_set_pcl_trigger_mode(fs_pcl_trigger_mode_t mode)
{
    if (mode != FS_PCL_TRIGGER_EXTERNAL && mode != FS_PCL_TRIGGER_INTERNAL) {
        return FS_ERR_BAD_PARAMS;
    }

    pthread_mutex_lock(&g_mgr_mutex);
    if (g_active_op != SOC_MGR_OP_NONE || g_program_token || g_cfg_cali_transfer) {
        pthread_mutex_unlock(&g_mgr_mutex);
        return FS_ERR_BUSY;
    }
    g_pcl_trigger_mode = mode;
    g_pointcloud_caps_initialized = false;
    pthread_mutex_unlock(&g_mgr_mutex);
    return FS_SUCCESS;
}

static void soc_mgr_init_pointcloud_caps(void)
{
    fs_pcl_trigger_mode_t trigger_mode;

    if (g_pointcloud_caps_initialized) return;

    trigger_mode = soc_mgr_get_pcl_trigger_mode();
    g_pointcloud_caps.pointcloud_timesync_supported = false;
    g_pointcloud_caps.pointcloud_true_fsync_anchor_supported = false;
    g_pointcloud_caps.trigger_to_spad_fsync_delay_ns = FS_CFG_POINTCLOUD_TRIGGER_TO_SPAD_FSYNC_DELAY_NS;
    g_pointcloud_caps.trigger_mode = trigger_mode;

    int fsync_fd = open(FS_FRAME_SYNC_DEVICE_PATH, O_RDWR | O_NONBLOCK);
    if (fsync_fd >= 0) { g_pointcloud_caps.pointcloud_true_fsync_anchor_supported = true; close(fsync_fd); }
    else { FS_LOGI("soc_mgr: %s unavailable, using trigger fallback", FS_FRAME_SYNC_DEVICE_PATH); }

    if (trigger_mode == FS_PCL_TRIGGER_INTERNAL) {
        g_pointcloud_caps.pointcloud_timesync_supported =
            g_pointcloud_caps.pointcloud_true_fsync_anchor_supported;
        if (!g_pointcloud_caps.pointcloud_timesync_supported) {
            FS_LOGE("soc_mgr: internal PCL trigger requires %s", FS_FRAME_SYNC_DEVICE_PATH);
        }
    } else {
        int trig_fd = open(FS_GPIO_TRIG_DEVICE_PATH, O_RDWR);

        if (trig_fd >= 0) {
            g_pointcloud_caps.pointcloud_timesync_supported = true;
            close(trig_fd);
        } else {
            FS_LOGW("soc_mgr: %s unavailable (errno=%d)", FS_GPIO_TRIG_DEVICE_PATH, errno);
        }
    }

    g_pointcloud_caps_initialized = true;
}

static void *time_sync_pub_thread(void *arg)
{
    (void)arg;
    FS_LOGI("time sync pub thread started");
    while (g_time_sync_running) {
        int ret = soc_time_sync_module_pub_poll_once();
        if (ret == SOC_TIME_SYNC_MODULE_PUB_STOPPED) {
            FS_LOGI("time sync pub session stopped");
            break;
        }
        if (ret != FS_SUCCESS && ret != FS_ERR_IO && ret != FS_ERR_AGAIN && ret != FS_ERR_BUSY) {
            FS_LOGW("time_sync_pub_poll_once: %d", ret);
        }
    }
    g_time_sync_running = 0;
    FS_LOGI("time sync pub thread exiting");
    return NULL;
}

static int32_t reject_to_fs_err(fw_if_reject_reason_t reason)
{
    switch (reason) {
    case FW_IF_REJECT_BUSY:
        return FS_ERR_BUSY;
    case FW_IF_REJECT_INVALID_STATE:
        return FS_ERR_UNSUPPORT;
    case FW_IF_REJECT_INVALID_ARGUMENT:
        return FS_ERR_BAD_PARAMS;
    case FW_IF_REJECT_TIMEOUT:
        return FS_ERR_TIME_OUT;
    default:
        return FS_ERR_INTERNAL;
    }
}

static fw_if_reject_reason_t fs_err_to_reject(int32_t err)
{
    if (err == FS_ERR_BUSY) {
        return FW_IF_REJECT_BUSY;
    }
    if (err == FS_ERR_BAD_PARAMS) {
        return FW_IF_REJECT_INVALID_ARGUMENT;
    }
    if (err == FS_ERR_TIME_OUT || err == FS_ERR_MCU_PROT_TIMEOUT) {
        return FW_IF_REJECT_TIMEOUT;
    }
    if (err == FS_SUCCESS) {
        return FW_IF_REJECT_OK;
    }
    return FW_IF_REJECT_INTERNAL;
}

static void set_state_locked(soc_mgr_state_t state)
{
    g_mgr_state = state;
}

static int32_t soc_mgr_mcu_program_enter(void)
{
    mcu_prot_program_status_t status;
    int32_t err;

    err = mcu_prot_program_enter(&status);
    if (err != FS_SUCCESS) {
        return err;
    }
    if (status.mcu_state != (uint8_t)FW_IF_MCU_STATE_PROGRAM) {
        return FS_ERR_UNSUPPORT;
    }
    return FS_SUCCESS;
}

static int32_t soc_mgr_mcu_program_exit(void)
{
    mcu_prot_program_status_t status;
    int32_t err;

    err = mcu_prot_program_exit(&status);
    if (err != FS_SUCCESS) {
        return err;
    }
    if (status.mcu_state != (uint8_t)FW_IF_MCU_STATE_IDLE) {
        return FS_ERR_UNSUPPORT;
    }
    return FS_SUCCESS;
}

void soc_mgr_init(void)
{
    pthread_mutex_lock(&g_mgr_mutex);
    g_mgr_state = SOC_MGR_STATE_IDLE;
    g_active_op = SOC_MGR_OP_NONE;
    g_program_token = false;
    g_cfg_cali_transfer = false;
    g_program_saved_mode = FS_SDK_WORK_MODE_IDLE;
    g_pcl_trigger_mode =
        (FS_CFG_PCL_TRIGGER_MODE == FS_CFG_PCL_TRIGGER_MODE_INTERNAL)
            ? FS_PCL_TRIGGER_INTERNAL
            : FS_PCL_TRIGGER_EXTERNAL;
    g_pointcloud_caps_initialized = false;
    pthread_mutex_unlock(&g_mgr_mutex);
}

soc_mgr_state_t soc_mgr_get_state(void)
{
    soc_mgr_state_t state;

    pthread_mutex_lock(&g_mgr_mutex);
    state = g_mgr_state;
    pthread_mutex_unlock(&g_mgr_mutex);
    return state;
}

soc_mgr_operation_t soc_mgr_get_active_operation(void)
{
    soc_mgr_operation_t op;

    pthread_mutex_lock(&g_mgr_mutex);
    op = g_active_op;
    pthread_mutex_unlock(&g_mgr_mutex);
    return op;
}

bool soc_mgr_program_token_held(void)
{
    bool held;

    pthread_mutex_lock(&g_mgr_mutex);
    held = g_program_token;
    pthread_mutex_unlock(&g_mgr_mutex);
    return held;
}

bool soc_mgr_mode_transition_busy(void)
{
    bool busy;

    pthread_mutex_lock(&g_mgr_mutex);
    busy = (g_active_op == SOC_MGR_OP_MODE_TRANSITION);
    pthread_mutex_unlock(&g_mgr_mutex);
    return busy;
}

bool soc_mgr_cfg_cali_transfer_busy(void)
{
    bool busy;

    pthread_mutex_lock(&g_mgr_mutex);
    busy = g_cfg_cali_transfer;
    pthread_mutex_unlock(&g_mgr_mutex);
    return busy;
}

int32_t soc_mgr_time_sync_stop(void)
{
    if (!soc_time_sync_module_enabled()) {
        g_time_sync_running = 0;
        g_time_sync_thread = 0;
        return FS_SUCCESS;
    }

    g_time_sync_running = 0;
    soc_time_sync_module_pub_stop();  /* STOP trigger + anchor, wake poll() */

    if (g_time_sync_thread != 0) {
        pthread_join(g_time_sync_thread, NULL);
        g_time_sync_thread = 0;
    }
    return FS_SUCCESS;
}

int32_t soc_mgr_time_sync_start_for_mode(fs_sdk_work_mode_e work_mode)
{
    int32_t err;

    if (!soc_mgr_is_pointcloud_mode(work_mode)) return FS_SUCCESS;
    if (!soc_time_sync_module_enabled()) return FS_SUCCESS;

    soc_mgr_init_pointcloud_caps();
    if (!g_pointcloud_caps.pointcloud_timesync_supported) {
        FS_LOGE("pointcloud TimeSync not supported on this hardware");
        return FS_ERR_IO;
    }
    if (g_time_sync_running) return FS_SUCCESS;

    err = soc_time_sync_module_pub_prepare(&g_pointcloud_caps);
    if (err) return err;

    err = soc_time_sync_module_pub_start_anchor();
    if (err) FS_LOGW("pub_start_anchor failed: %d (non-fatal)", err);

    /* does NOT set g_time_sync_running — that happens after ENABLE ACK + start_trigger */
    return FS_SUCCESS;
}

bool soc_mgr_time_sync_is_running(void)
{
    if (!soc_time_sync_module_enabled()) return false;
    return (g_time_sync_running != 0);
}

/* ---- exit pointcloud helper ---- */
static int32_t soc_mgr_exit_pointcloud_timesync_if_needed(fs_sdk_work_mode_e old_mode)
{
    if (!soc_mgr_is_pointcloud_mode(old_mode)) return FS_SUCCESS;
    if (!soc_time_sync_module_enabled()) return FS_SUCCESS;

    (void)soc_mgr_time_sync_stop();

    if (!soc_mgr_pointcloud_uses_internal_trigger()) {
        int32_t err = soc_mgr_send_timesync_ctrl_and_wait_ack(FS_TIMESYNC_CTRL_DISABLE);
        if (err != FS_SUCCESS) {
            FS_LOGW("TIME_SYNC_DISABLE ACK failed: %d; trigger already stopped, continue mode switch", err);
        }
    }
    return FS_SUCCESS;
}

int32_t soc_mgr_setup_mode(fs_sdk_work_mode_e work_mode)
{
    int32_t err = FS_SUCCESS;
    int32_t mode_err;
    soc_mgr_state_t next_state;
    fs_sdk_work_mode_e old_mode;

    pthread_mutex_lock(&g_mgr_mutex);
    if (g_program_token || g_cfg_cali_transfer ||
        g_active_op != SOC_MGR_OP_NONE) {
        pthread_mutex_unlock(&g_mgr_mutex);
        return FS_ERR_BUSY;
    }
    g_active_op = SOC_MGR_OP_MODE_TRANSITION;
    pthread_mutex_unlock(&g_mgr_mutex);

    /* exit old pointcloud session (PCL->PCL also restarts fully) */
    old_mode = fs_sdk_get_work_mode();
    (void)soc_mgr_exit_pointcloud_timesync_if_needed(old_mode);

    if (soc_mgr_is_pointcloud_mode(work_mode) && soc_time_sync_module_enabled()) {
        soc_mgr_init_pointcloud_caps();
        if (!g_pointcloud_caps.pointcloud_timesync_supported) {
            if (soc_mgr_pointcloud_uses_internal_trigger()) {
                FS_LOGE("pointcloud mode blocked: %s unavailable for internal trigger",
                        FS_FRAME_SYNC_DEVICE_PATH);
            } else {
                FS_LOGE("pointcloud mode blocked: %s unavailable", FS_GPIO_TRIG_DEVICE_PATH);
            }
            pthread_mutex_lock(&g_mgr_mutex);
            g_active_op = SOC_MGR_OP_NONE;
            pthread_mutex_unlock(&g_mgr_mutex);
            return FS_ERR_IO;
        }
    }

    /* apply new mode */
    err = fs_sdk_setup_mode_impl(work_mode);
    mode_err = err;
    /* NOTE: for PCL, fs_sdk_setup_mode_impl() configures SPAD/RX for external trigger
     *       and MCU calls app_board_fsync_set_scan_mode(POINTCLOUD_EXTERNAL).
     *       Actual scan frames start only after ENABLE ACK + gpio_trig START below.
     *       If mode setup fails, system stays stopped/fault — do NOT auto-recover old trigger. */

    next_state = (work_mode == FS_SDK_WORK_MODE_IDLE)
                     ? SOC_MGR_STATE_IDLE : SOC_MGR_STATE_NORMAL_RUN;

    if (mode_err == FS_SUCCESS && soc_mgr_is_pointcloud_mode(work_mode)) {

        if (!soc_time_sync_module_enabled()) {
            FS_LOGW("pointcloud mode running without TimeSync module "
                    "(FS_CFG_FEATURE_MCU_TIMESYNC / FS_CFG_ROUTE_TIME_SYNC);");
            goto out;
        }

        int32_t ts_err = soc_mgr_time_sync_start_for_mode(work_mode);
        if (ts_err != FS_SUCCESS) { err = ts_err; goto out; }

        if (!soc_mgr_pointcloud_uses_internal_trigger()) {
            int32_t ack_err = soc_mgr_send_timesync_ctrl_and_wait_ack(FS_TIMESYNC_CTRL_ENABLE_POINTCLOUD);
            if (ack_err != FS_SUCCESS) {
                FS_LOGE("ENABLE_POINTCLOUD ACK failed: %d", ack_err);
                soc_mgr_time_sync_stop();
                err = ack_err;
                goto out;
            }
        }

        int32_t trig_err = soc_time_sync_module_pub_start_trigger();
        if (trig_err != FS_SUCCESS) {
            FS_LOGE("pub_start_trigger failed: %d", trig_err);
            soc_mgr_time_sync_stop();
            err = trig_err;
            goto out;
        }

        g_time_sync_running = 1;
        if (pthread_create(&g_time_sync_thread, NULL, time_sync_pub_thread, NULL) != 0) {
            FS_LOGE("failed to create time sync pub thread");
            g_time_sync_running = 0;
            soc_time_sync_module_pub_stop();
            err = FS_ERR_IO;
            goto out;
        }
    }

out:
    pthread_mutex_lock(&g_mgr_mutex);
    g_active_op = SOC_MGR_OP_NONE;
    if (mode_err == FS_SUCCESS && err == FS_SUCCESS) {
        set_state_locked(next_state);
    }
    pthread_mutex_unlock(&g_mgr_mutex);

    return err;
}

int32_t soc_mgr_runtime_mutation_begin(void)
{
    int32_t err = FS_SUCCESS;

    pthread_mutex_lock(&g_mgr_mutex);
    if (g_program_token || g_cfg_cali_transfer ||
        g_active_op != SOC_MGR_OP_NONE) {
        err = FS_ERR_BUSY;
    }
    else {
        g_active_op = SOC_MGR_OP_RUNTIME_MUTATION;
    }
    pthread_mutex_unlock(&g_mgr_mutex);
    return err;
}

void soc_mgr_runtime_mutation_end(void)
{
    pthread_mutex_lock(&g_mgr_mutex);
    if (g_active_op == SOC_MGR_OP_RUNTIME_MUTATION) {
        g_active_op = SOC_MGR_OP_NONE;
    }
    pthread_mutex_unlock(&g_mgr_mutex);
}

int32_t soc_mgr_program_token_acquire(fw_if_reject_reason_t *out_reason)
{
    int32_t err = FS_SUCCESS;
    fw_if_reject_reason_t reason = FW_IF_REJECT_OK;
    fs_sdk_work_mode_e saved_mode;

    pthread_mutex_lock(&g_mgr_mutex);
    if (g_program_token) {
        reason = FW_IF_REJECT_BUSY;
        err = reject_to_fs_err(reason);
        goto out_unlock;
    }
    if (g_cfg_cali_transfer || g_active_op != SOC_MGR_OP_NONE) {
        reason = FW_IF_REJECT_BUSY;
        err = reject_to_fs_err(reason);
        goto out_unlock;
    }
    saved_mode = fs_sdk_get_work_mode();
    g_program_saved_mode = saved_mode;
    g_active_op = SOC_MGR_OP_PROGRAM_WINDOW;
    set_state_locked(SOC_MGR_STATE_PROGRAM_ENTERING);
    pthread_mutex_unlock(&g_mgr_mutex);

    err = soc_mgr_time_sync_stop();
    if (err != FS_SUCCESS) {
        reason = fs_err_to_reject(err);
        goto rollback;
    }

    err = fs_sdk_setup_mode_impl(FS_SDK_WORK_MODE_IDLE);

    if (err != FS_SUCCESS) {
        reason = fs_err_to_reject(err);
        goto rollback;
    }

    err = soc_mgr_mcu_program_enter();
    if (err != FS_SUCCESS) {
        reason = fs_err_to_reject(err);
        goto rollback;
    }

    pthread_mutex_lock(&g_mgr_mutex);
    g_program_token = true;
    set_state_locked(SOC_MGR_STATE_PROGRAM);
    g_active_op = SOC_MGR_OP_NONE;
    pthread_mutex_unlock(&g_mgr_mutex);
    goto out;

rollback:
    pthread_mutex_lock(&g_mgr_mutex);
    g_program_token = false;
    g_active_op = SOC_MGR_OP_NONE;
    set_state_locked(SOC_MGR_STATE_IDLE);
    pthread_mutex_unlock(&g_mgr_mutex);
    if (saved_mode != FS_SDK_WORK_MODE_IDLE) {
        /* Do not auto-restore old pointcloud trigger here. Caller shall setup mode explicitly. */
    }
    goto out;

out_unlock:
    pthread_mutex_unlock(&g_mgr_mutex);
out:
    if (out_reason != NULL) {
        *out_reason = reason;
    }
    return err;
}

int32_t soc_mgr_program_token_release(fw_if_reject_reason_t *out_reason)
{
    int32_t err = FS_SUCCESS;
    fw_if_reject_reason_t reason = FW_IF_REJECT_OK;

    pthread_mutex_lock(&g_mgr_mutex);
    if (!g_program_token) {
        reason = FW_IF_REJECT_INVALID_STATE;
        err = reject_to_fs_err(reason);
        pthread_mutex_unlock(&g_mgr_mutex);
        goto out;
    }
    if (g_cfg_cali_transfer) {
        reason = FW_IF_REJECT_BUSY;
        err = reject_to_fs_err(reason);
        pthread_mutex_unlock(&g_mgr_mutex);
        goto out;
    }
    g_active_op = SOC_MGR_OP_PROGRAM_WINDOW;
    set_state_locked(SOC_MGR_STATE_PROGRAM_EXITING);
    pthread_mutex_unlock(&g_mgr_mutex);

    err = soc_mgr_mcu_program_exit();
    if (err != FS_SUCCESS) {
        reason = fs_err_to_reject(err);
        pthread_mutex_lock(&g_mgr_mutex);
        g_active_op = SOC_MGR_OP_NONE;
        set_state_locked(SOC_MGR_STATE_PROGRAM);
        pthread_mutex_unlock(&g_mgr_mutex);
        goto out;
    }

    /* DEC-20260515-002: Program exit returns to IDLE; caller must explicitly
     * start NORMAL_RUN again — do not restore g_program_saved_mode here.
     * Skip setup_mode_impl if already in IDLE: acquire already placed the
     * system in IDLE, so a second full teardown+reinit cycle is unnecessary. */
    if (fs_sdk_get_work_mode() != FS_SDK_WORK_MODE_IDLE) {
        err = fs_sdk_setup_mode_impl(FS_SDK_WORK_MODE_IDLE);
    }

    pthread_mutex_lock(&g_mgr_mutex);
    g_program_token = false;
    g_active_op = SOC_MGR_OP_NONE;
    set_state_locked(SOC_MGR_STATE_IDLE);
    pthread_mutex_unlock(&g_mgr_mutex);

out:
    if (out_reason != NULL) {
        *out_reason = reason;
    }
    return err;
}

int32_t soc_mgr_check_cfg_cali_write(int flash_data_type)
{
    (void)flash_data_type;

    pthread_mutex_lock(&g_mgr_mutex);
    if (!g_program_token) {
        pthread_mutex_unlock(&g_mgr_mutex);
        return FS_ERR_BUSY;
    }
    if (g_cfg_cali_transfer) {
        pthread_mutex_unlock(&g_mgr_mutex);
        return FS_ERR_BUSY;
    }
    if (g_active_op != SOC_MGR_OP_NONE) {
        pthread_mutex_unlock(&g_mgr_mutex);
        return FS_ERR_BUSY;
    }
    pthread_mutex_unlock(&g_mgr_mutex);
    return FS_SUCCESS;
}

int32_t soc_mgr_cfg_cali_transfer_begin(int flash_data_type)
{
    int32_t err = FS_SUCCESS;

    (void)flash_data_type;

    pthread_mutex_lock(&g_mgr_mutex);
    if (!g_program_token) {
        err = FS_ERR_BUSY;
    }
    else if (g_cfg_cali_transfer) {
        err = FS_ERR_BUSY;
    }
    else if (g_active_op != SOC_MGR_OP_NONE) {
        err = FS_ERR_BUSY;
    }
    else {
        g_cfg_cali_transfer = true;
        g_active_op = SOC_MGR_OP_CFG_CALI_TRANSFER;
    }
    pthread_mutex_unlock(&g_mgr_mutex);
    return err;
}

void soc_mgr_cfg_cali_transfer_end(void)
{
    pthread_mutex_lock(&g_mgr_mutex);
    g_cfg_cali_transfer = false;
    if (g_active_op == SOC_MGR_OP_CFG_CALI_TRANSFER) {
        g_active_op = SOC_MGR_OP_NONE;
    }
    pthread_mutex_unlock(&g_mgr_mutex);
}

int32_t soc_mgr_upgrade_enter(void)
{
    pthread_mutex_lock(&g_mgr_mutex);
    if (g_program_token || g_cfg_cali_transfer ||
        g_active_op != SOC_MGR_OP_NONE) {
        pthread_mutex_unlock(&g_mgr_mutex);
        return FS_ERR_BUSY;
    }
    g_active_op = SOC_MGR_OP_UPGRADE_PREPARE;
    set_state_locked(SOC_MGR_STATE_UPDATE_PREPARE);
    pthread_mutex_unlock(&g_mgr_mutex);
    return FS_SUCCESS;
}

void soc_mgr_upgrade_leave(int32_t upgrade_err)
{
    (void)upgrade_err;

    pthread_mutex_lock(&g_mgr_mutex);
    g_active_op = SOC_MGR_OP_NONE;
    if (g_mgr_state == SOC_MGR_STATE_UPDATE_PREPARE) {
        set_state_locked((fs_sdk_get_work_mode() == FS_SDK_WORK_MODE_IDLE) ?
                             SOC_MGR_STATE_IDLE :
                             SOC_MGR_STATE_NORMAL_RUN);
    }
    pthread_mutex_unlock(&g_mgr_mutex);
}

/* ---- Module output gates (sole authority — no external switches) ---- */

bool soc_mgr_pcl_udp_allowed(void)
{
    bool busy;
    soc_mgr_state_t state;

    pthread_mutex_lock(&g_mgr_mutex);
    busy  = g_cfg_cali_transfer;
    state = g_mgr_state;
    pthread_mutex_unlock(&g_mgr_mutex);

    if (busy) return false;
    return (state == SOC_MGR_STATE_NORMAL_RUN);
}

bool soc_mgr_imu_udp_allowed(void)
{
    bool busy;
    soc_mgr_state_t state;

    pthread_mutex_lock(&g_mgr_mutex);
    busy  = g_cfg_cali_transfer;
    state = g_mgr_state;
    pthread_mutex_unlock(&g_mgr_mutex);

    if (busy) return false;
    return (state == SOC_MGR_STATE_NORMAL_RUN);
}

bool soc_mgr_imu_forward_allowed(void)
{
    soc_mgr_state_t state;

    pthread_mutex_lock(&g_mgr_mutex);
    state = g_mgr_state;
    pthread_mutex_unlock(&g_mgr_mutex);

    return (state == SOC_MGR_STATE_NORMAL_RUN);
}
