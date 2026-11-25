#ifndef __FS_HW_H__
#define __FS_HW_H__

#include <stdint.h>
#include <stdbool.h>
// #include "fs_sensor_hub.h"

typedef enum {
    FS_COMM_INTERFACE_SPI = 0,
    FS_COMM_INTERFACE_I2C = 1,
} fs_comm_interface_e;

typedef enum {
    FS_RX_FL6X32_1,
    FS_RX_FL6X32_ALL,

    FS_TX_FL9003_1,
    FS_TX_FL9003_2,
    FS_TX_FL9003_3,
    FS_TX_FL9003_ALL,

    FS_TX_FL9103_1,
    FS_TX_FL9103_2,
    FS_TX_FL9103_ALL,
} fs_sensor_tx_rx_e;

typedef enum {
    ADC_TYPE_TEMP_TX1 = 0, // TX1 板的温度
    ADC_TYPE_TEMP_TX2,     // TX2 板的温度
    ADC_TYPE_TEMP_RX,      // RX  板的温度
    ADC_TYPE_TEMP_PWR,     // PWR 板的温度
    ADC_TYPE_TEMP_MCU,     // MCU 内部温度, 不是通过 ADC 获取的

    ADC_TYPE_VEX,          // VEX 电压
    ADC_TYPE_VBD,          // VBD 电压
    ADC_TYPE_VPP,          // VPP 电压

    ADC_TYPE_VER_RX,       // RX  板的硬件版本号
    ADC_TYPE_VER_TX1,      // TX1 板的硬件版本号, 未接为 V02~V03
    ADC_TYPE_VER_TX2,      // TX2 板的硬件版本号, 未接为 V02~V03

    ADC_TYPE_ALL,
    ADC_TYPE_INIT,         // 获取初始值
} adc_type_e;

typedef enum {
    ADC_DATA_RAW = 0,      // 原始 ADC 数据，单位 mV
    ADC_DATA_DEF_PARA,     // 标准参数校准数据
    ADC_DATA_CALIB_PARA,   // 后校准参数计算的数据

    ADC_DATA_CNT,
} adc_data_e;

typedef enum {
    DAC_TYPE_VBD = 0,
    DAC_TYPE_VPP = 1,
} dac_type_e;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 微秒级延时函数
 * @param us 延时时长，单位为微秒
 */
void fs_hw_usleep(int32_t us);

/**
 * @brief 毫秒级延时函数
 * @param ms 延时时长，单位为毫秒
 */
void fs_hw_msleep(int32_t ms);

/**
 * @brief 为 sensor 提供硬件上下电功能
 * @param dev_id sensor 编号
 * @param power_on 是否上电 1 - power on  0 - power off
 * @return 成功返回 FS_SUCCESS, 失败返回其他值
 */
int32_t fs_hw_power_ctrl(fs_sensor_tx_rx_e dev_id, uint8_t power_on);

/**
 * @brief 选择传感器通信接口
 * @param interface 通信接口类型
 * @return 成功返回 FS_SUCCESS，失败返回负值错误码
 */
int32_t fs_hw_rx_comm_interface_select(fs_comm_interface_e interface);

/**
 * @brief 为 sensor 提供硬件复位功能
 * @param dev_id sensor 编号
 * @return 成功返回 FS_SUCCESS, 失败返回其他值
 */
int32_t fs_hw_reset(fs_sensor_tx_rx_e dev_id);

/**
 * @brief 硬件初始化函数
 * @return 成功返回 FS_SUCCESS，失败返回负值错误码
 */
int32_t fs_hw_init(void);

/**
 * @brief 硬件复位 FL6X32 传感器
 * @return 成功返回 FS_SUCCESS，失败返回负值错误码
 */
int32_t fs_hw_reset_rx(void);

/**
 * @brief 控制 FL6X32 传感器 下电
 * @return 成功返回 FS_SUCCESS，失败返回负值错误码
 */
int32_t fs_hw_rx_power_off(void);

/**
 * @brief 控制 FL6X32 传感器电源
 * @param enable 是否使能
 * @return 成功返回 FS_SUCCESS，失败返回负值错误码
 */
int32_t fs_hw_rx_power_ctrl(bool enable);

/**
 * @brief 设置VPP电压
 *
 * @param volt 电压值，单位V
 * @param o_dac_code 输出参数，返回实际设置的DAC码值，可为NULL
 * @return int32_t 错误码，0表示成功
 */
int32_t fs_hw_set_vpp_volt(float volt, uint32_t *o_dac_code);

/**
 * @brief 设置VBD电压
 *
 * @param volt 电压值，单位V
 * @param o_dac_code 输出参数，返回实际设置的DAC码值，可为NULL
 * @return int32_t 错误码，0表示成功
 */
int32_t fs_hw_set_vbd_volt(float volt, uint32_t *o_dac_code);

/**
 * @brief 初始化DAC
 *
 * @param vpp_volt VPP电压值，单位V
 * @param vbd_volt VBD电压值，单位V
 * @return int32_t 错误码，0表示成功
 */
int32_t fs_hw_init_dac(float vpp_volt, float vbd_volt);

/**
 * @brief 获取 ADC 数值, 支持批量获取, 单位: mV 或 摄氏度 或 版本号的十分之一
 * @param type ADC 类型
 * @param fmt 数据格式
 * @param io_value 输入输出参数, 输入时表示传入的缓冲区指针, 输出时表示获取到的数据缓冲区指针
 * @param io_cnt 输入输出参数, 输入时表示传入的缓冲区长度, 输出时表示实际获取到的数据个数
 * @param o_raw_adc_mV 输出参数, 返回原始电压值(mV)，可为 NULL
 * @return 成功返回 FS_SUCCESS, 失败返回其他值
 */
int32_t fs_hw_get_adc(adc_type_e type, adc_data_e fmt, int32_t *io_value, uint8_t *io_cnt, uint32_t *o_raw_adc_mV);

/**
 * @brief 初始化 ADC
 *
 * @return int32_t 错误码，0表示成功
 */
int32_t fs_hw_init_adc(void);

/**
 * @brief 使能TX电源
 *
 * @param enable 是否使能
 * @return int32_t 错误码，0表示成功
 */
int32_t fs_hw_tx_power_ctrl(bool enable);

/**
 * @brief 重置博尔芯
 * @return int32_t 错误码，0表示成功
 */
int32_t fs_hw_reset_powerx(void);

/**
 * @brief 使能或关闭VBD电源
 *
 * @param enable 是否使能
 * @return int32_t 错误码，0表示成功
 */
int32_t fs_hw_enable_vbd(bool enable);

/**
 * @brief 使能或关闭VPP电源
 *
 * @param enable 是否使能
 * @return int32_t 错误码，0表示成功
 */
int32_t fs_hw_enable_vpp(bool enable);

/**
 * @brief 使能或关闭温度补偿功能
 *
 * @param enable 是否使能
 * @return int32_t 错误码，0表示成功
 */
int32_t fs_hw_enable_temp_comp(bool enable);

/**
 * @brief 使能或关闭电压校准功能
 *
 * @param enable 是否使能
 * @return int32_t 错误码，0表示成功
 */
int32_t fs_hw_enable_voltage_calib(bool enable);

int32_t fs_hw_store_adc_calib(adc_type_e type, float k, float b);
int32_t fs_hw_read_adc_calib(adc_type_e type, float *k, float *b);
int32_t fs_hw_remove_adc_calib(adc_type_e type);

int32_t fs_hw_store_dac_calib(dac_type_e type, float k, float b);
int32_t fs_hw_read_dac_calib(dac_type_e type, float *k, float *b);
int32_t fs_hw_remove_dac_calib(dac_type_e type);

/** Enter MCU program window for voltage-calibration flash save (PC 0x13 SAVE). */
int32_t fs_hw_calib_volt_save_begin(void);
/** Leave program window after voltage-calibration flash save. */
int32_t fs_hw_calib_volt_save_end(void);

#ifdef __cplusplus
}
#endif

#endif /* __FS_HW_H__ */
