#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <pthread.h>

#include "drivers/imu/ImuSensor.h"
#include "fs_err.h"
#include "fs_log.h"
#include "fs_top_module_cfg.h"
#include "mcu_protocol.h"
#include "fs_client_data_process.h"
#include "soc_system_manager.h"
#include "time_sync_eval.h"

#ifdef LOG_TAG
#undef LOG_TAG
#endif
#define LOG_TAG "imu_data_service"

/* MCU LSM6DS3TR is configured as accel +/-2g and gyro +/-250dps. */
#define MCU_IMU_ACC_2G_SCALE_G          (0.000061f)
#define MCU_IMU_GYRO_250DPS_SCALE_RAD_S (0.00015271631f)

#if FS_CFG_FEATURE_IMU_SERVICE && FS_CFG_FEATURE_MCU_IMU_PUSH && \
    (FS_CFG_ROUTE_IMU_PUSH != FS_SOC_MCU_LINK_DISABLED)
#define IMU_SERVICE_MCU_FORWARD_BACKEND_ACTIVE 1
#else
#define IMU_SERVICE_MCU_FORWARD_BACKEND_ACTIVE 0
#endif

#if IMU_SERVICE_MCU_FORWARD_BACKEND_ACTIVE
static bool imu_service_run_flag = false;
static pthread_mutex_t imu_service_lock = PTHREAD_MUTEX_INITIALIZER;

static bool imu_service_output_allowed(void)
{
    return soc_mgr_imu_forward_allowed();
}

static void imu_service_mcu_forward_cb(const imu_sample_msg_t *samples, uint16_t count, void *user)
{
    imu_sensor_data_t imu_data[MCU_IMU_PUSH_MAX_SAMPLES];
    uint16_t i;
    int ret;
    bool running;

    (void)user;
    if (samples == NULL || count == 0u) {
        return;
    }
    if (count > MCU_IMU_PUSH_MAX_SAMPLES) {
        count = MCU_IMU_PUSH_MAX_SAMPLES;
    }

    pthread_mutex_lock(&imu_service_lock);
    running = imu_service_run_flag;
    pthread_mutex_unlock(&imu_service_lock);
    if (!running) {
        return;
    }
    if (!imu_service_output_allowed()) {
        return;
    }

    time_sync_eval_record_imu_batch(count);
    memset(imu_data, 0, sizeof(imu_data));
    for (i = 0u; i < count; i++) {
        time_sync_eval_record_imu_timestamp(samples[i].t_imu_ns, samples[i].timestamp_quality);
        imu_data[i].timestamp = (int64_t)samples[i].t_imu_ns;
        imu_data[i].acc_x = (float)samples[i].acc_x * MCU_IMU_ACC_2G_SCALE_G;
        imu_data[i].acc_y = (float)samples[i].acc_y * MCU_IMU_ACC_2G_SCALE_G;
        imu_data[i].acc_z = (float)samples[i].acc_z * MCU_IMU_ACC_2G_SCALE_G;
        imu_data[i].gyro_x = (float)samples[i].gyro_x * MCU_IMU_GYRO_250DPS_SCALE_RAD_S;
        imu_data[i].gyro_y = (float)samples[i].gyro_y * MCU_IMU_GYRO_250DPS_SCALE_RAD_S;
        imu_data[i].gyro_z = (float)samples[i].gyro_z * MCU_IMU_GYRO_250DPS_SCALE_RAD_S;
        imu_data[i].temperature = 0;
    }

    for (i = 0u; i < count; i++) {
        ret = fs_send_imu_data(&imu_data[i], 1u);
        if (ret < 0) {
            FS_LOGE("send MCU IMU sample failed: %d, batch_count=%u, sample_idx=%u", ret, count, i);
            break;
        }
    }
}
#endif

int imu_service_init(void)
{
#if !FS_CFG_FEATURE_IMU_SERVICE
    return FS_SUCCESS;
#elif !IMU_SERVICE_MCU_FORWARD_BACKEND_ACTIVE
    /*
     * Default product topology: SoC-owned IMU. The native entry points
     * init_imu_sensor() and getImuData() remain the intended backend surface;
     * this service does not register the optional MCU IMU Push callback.
     */
    return FS_SUCCESS;
#else
    int ret;

    pthread_mutex_lock(&imu_service_lock);
    imu_service_run_flag = true;
    pthread_mutex_unlock(&imu_service_lock);

    ret = mcu_prot_register_imu_forward_cb(imu_service_mcu_forward_cb, NULL);
    if (ret != FS_SUCCESS) {
        pthread_mutex_lock(&imu_service_lock);
        imu_service_run_flag = false;
        pthread_mutex_unlock(&imu_service_lock);
        FS_LOGE("register MCU IMU forward callback failed: %d", ret);
        return ret;
    }

    FS_LOGI("MCU IMU data service initialized");
    return FS_SUCCESS;
#endif
}

int imu_service_deinit(void)
{
#if !FS_CFG_FEATURE_IMU_SERVICE || !IMU_SERVICE_MCU_FORWARD_BACKEND_ACTIVE
    return FS_SUCCESS;
#else
    pthread_mutex_lock(&imu_service_lock);
    if (!imu_service_run_flag) {
        pthread_mutex_unlock(&imu_service_lock);
        return 0;
    }
    imu_service_run_flag = false;
    pthread_mutex_unlock(&imu_service_lock);

    (void)mcu_prot_register_imu_forward_cb(NULL, NULL);
    FS_LOGI("MCU IMU data service deinitialized");
    return FS_SUCCESS;
#endif
}
