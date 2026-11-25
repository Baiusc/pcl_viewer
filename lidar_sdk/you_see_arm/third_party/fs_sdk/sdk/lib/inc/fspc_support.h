#ifndef __FSPC_SUPPORT_H__
#define __FSPC_SUPPORT_H__

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    FS_SDK_WORK_MODE_IDLE   = 0,  // SDK 工作模式: 待机模式
    FS_SDK_WORK_MODE_GRAY   = 1,  // SDK 工作模式: 灰度图模式, 用于调焦
    FS_SDK_WORK_MODE_RANGE  = 2,  // SDK 工作模式: Range 模式, 用于标定
    FS_SDK_WORK_MODE_PCL    = 3,  // SDK 工作模式: 点云(Point Cloud)模式, 用于导航与避障
    FS_SDK_WORK_MODE_CUSTOM = 4,  // SDK 工作模式: 自定义模式, 由用户根据需求定义具体功能
    FS_SDK_WORK_MODE_GRAY_CUSTOM = 5,  // SDK 工作模式: 灰度图模式(自定义), 用于调焦

    FS_SDK_WORK_MODE_CNT,
} fs_sdk_work_mode_e;

typedef enum {
    LIDAR_WORK_MODE_IDLE  = 0,  // 雷达工作模式: 待机模式
    LIDAR_WORK_MODE_GRAY  = 1,  // 雷达工作模式: 灰度图模式, 用于调焦
    LIDAR_WORK_MODE_RANGE = 2,  // 雷达工作模式: Range 模式, 用于标定
    LIDAR_WORK_MODE_PCL   = 3,  // 雷达工作模式: 点云(Point Cloud)模式, 用于导航与避障
} lidar_work_mode_e;

typedef enum {
    CHIP_DEVICE_RX       = 0,  // 激光雷达 RX 芯片

    CHIP_DEVICE_TX_H_ALL = 10, // 激光雷达 TX high side 芯片: 所有 TX high side 芯片
    CHIP_DEVICE_TX_H0    = 11, // 激光雷达 TX high side 芯片: 编号 0
    CHIP_DEVICE_TX_H1    = 12, // 激光雷达 TX high side 芯片: 编号 1(option)
    CHIP_DEVICE_TX_H2    = 13, // 激光雷达 TX high side 芯片: 编号 2(option)

    CHIP_DEVICE_TX_L_ALL = 20, // 激光雷达 TX low side 芯片: 所有 TX low side 芯片(option)
    CHIP_DEVICE_TX_L0    = 21, // 激光雷达 TX low side 芯片: 编号 0(option)
    CHIP_DEVICE_TX_L1    = 22, // 激光雷达 TX low side 芯片: 编号 1(option)
} chip_device_e;

typedef enum {
    TX_SEQ_MODE_REVERSE       = 0,    // 全扫反序点亮
    TX_SEQ_MODE_FORWARD       = 1,    // 全扫正序点亮
    TX_SEQ_MODE_SINGLE_Y_CHN  = 2,    // 单通道(Y)点亮
    TX_SEQ_MODE_MULTI_Y_CHN   = 3,    // 多通道(Y)点亮

    TX_SEQ_MODE_SINGLE_X_CHN  = 4,    // 单通道(X)点亮
    TX_SEQ_MODE_BLOCK_CHN     = 5,    // 块(X,Y)扫描点亮
    TX_SEQ_MODE_MULTI_X_CHN   = 6,    // 多通道(X)点亮
    TX_SEQ_MODE_CTRL_TX       = 7,    // TX 光斑控制
    TX_SEQ_MODE_TX_RX_MERGE   = 8,    // TX & RX 融合

    TX_SEQ_MODE_TX_SUSPEND    = 50,   // TX 暂停: 关闭
    TX_SEQ_MODE_TX_RESUME     = 51,   // TX 恢复: 开启
} tx_seq_mode_e;

typedef enum {
    DAC_DEVICE_VBD = 0,  // 控制 VBD 电压的 DAC 设备
    DAC_DEVICE_VPP = 1,  // 控制 VPP 电压的 DAC 设备
} dac_device_e;

typedef enum {
    ADC_DEVICE_VBD = 0,  // 获取 VBD 电压的 ADC 设备
    ADC_DEVICE_VPP = 1,  // 获取 VPP 电压的 ADC 设备
    ADC_DEVICE_VEX = 2,  // 获取 VEX 电压的 ADC 设备
} adc_device_e;

typedef enum {
    TEST_DEVICE_ALL       = 0,  // 自测: 所有设备
    TEST_DEVICE_SOC_BOARD = 1,  // 自测: SOC 板
    TEST_DEVICE_MCU_BOARD = 2,  // 自测: MCU 板
    TEST_DEVICE_TX_BOARD  = 3,  // 自测: TX  板
    TEST_DEVICE_RX_BOARD  = 4,  // 自测: RX  板
} test_device_e;

typedef enum {
    PC_CFGDATA_TYPE_CFG          = 0,
    PC_CFGDATA_TYPE_CALI         = 1,
    PC_CFGDATA_TYPE_PC_TOOL_CALI = 0x11,
} pc_cfgdata_type_e;

typedef struct {
    uint8_t        data_type;
    uint32_t       total_len;
    uint16_t       cur_pkt;
    uint32_t       offset;
    uint8_t        reserved;
    const uint8_t *data;
    uint32_t       data_len;
} pc_cfgdata_write_req_t;

typedef struct {
    uint8_t  data_type;
    uint32_t request_total_len;
    uint16_t cur_pkt;
    uint32_t offset;
    uint8_t  reserved;
    uint8_t *data;
    uint32_t data_cap;
    uint32_t data_len;
    uint32_t actual_total_len;
} pc_cfgdata_read_req_t;

typedef struct {
    float temp_tx;
    float init_temp_tx;
    float temp_rx;
    float init_temp_rx;
} fs_temp_cmd_t;

typedef struct {
    /**
     * @brief 获取版本信息: 支持 PC 工具获取雷达模组的版本信息
     * @param ver_buf 输出参数, 用于返回版本信息字符串
     * @param len 输入输出参数, 输入时表示 ver_buf 的缓冲区长度, 输出时表示实际写入的版本信息字符串长度
     * @return 0 表示成功，非 0 表示失败
     */
    int32_t (*get_version_inf)(char *ver_buf, uint32_t *len);

    /**
     * @brief 设置工作模式: 支持 PC 工具切换雷达模组的工作模式
     * @param mode 工作模式, 详见 fs_sdk_work_mode_e 枚举类型
     * @param cfg_idx 配置索引, 用于指定某些工作模式下的特定配置项
     * @return 0 表示成功，非 0 表示失败
     */
    int32_t (*set_work_mode)(fs_sdk_work_mode_e mode, uint32_t cfg_idx);

    /**
     * @brief 获取工作模式: 支持 PC 工具获取雷达模组当前的工作模式
     * @param mode 输出参数, 用于返回当前的工作模式, 详见 fs_sdk_work_mode_e 枚举类型
     * @param cfg_idx 输出参数, 用于返回当前工作模式下的配置索引
     * @return 0 表示成功，非 0 表示失败
     */
    int32_t (*get_work_mode)(fs_sdk_work_mode_e *mode, uint32_t *cfg_idx);

    /**
     * @brief 读取设备寄存器: 支持 PC 工具读取设备寄存器的值(支持连续读取多个寄存器)
     * @param dev_id 设备 ID, 详见 chip_device_e 枚举类型
     * @param reg_addr 寄存器地址
     * @param reg_val 输出参数, 用于返回寄存器值的缓冲区指针
     * @param reg_cnt 输入输出参数, 输入时表示 reg_val 的缓冲区长度, 输出时表示实际读到的寄存器个数
     * @return 0 表示成功，非 0 表示失败
     */
    int32_t (*get_device_reg)(chip_device_e dev_id, uint32_t reg_addr, uint32_t *reg_val, uint32_t *reg_cnt);

    /**
     * @brief 写入设备寄存器: 支持 PC 工具写入设备寄存器的值(支持连续写入多个寄存器)
     * @param dev_id 设备 ID, 详见 chip_device_e 枚举类型
     * @param reg_addr 寄存器地址
     * @param reg_val 寄存器值的缓冲区指针
     * @param reg_cnt 寄存器个数, 表示 reg_val 缓冲区中寄存器值的数量
     * @return 0 表示成功，非 0 表示失败
     */
    int32_t (*set_device_reg)(chip_device_e dev_id, uint32_t reg_addr, const uint32_t *reg_val, uint32_t reg_cnt);

    /**
     * @brief 控制温飘补偿功能开关, 以支持 PC 工具开关温飘补偿功能
     * @param enable 是否开启温飘补偿功能: false 关闭, true 开启
     * @return 0 表示成功，非 0 表示失败
     */
    int32_t (*enable_temp_comp)(bool enable);

    /**
     * @brief 获取温度值: 支持 PC 工具获取 TX 和 RX 端温度值,TX RX共板时，这两个温度相同
     * @param p_temp 输出参数,详见 fs_temp_cmd_t 结构体
     * @return 0 表示成功，非 0 表示失败
     */
    int32_t (*get_txrx_temp)(fs_temp_cmd_t *p_temp);

    /**
     * @brief 控制电压校准功能开关, 以支持 PC 工具开关电压校准功能
     * @param enable 是否开启电压校准功能: false 关闭(使用默认校准参数), true 开启(使用后期校准参数)
     * @return 0 表示成功，非 0 表示失败
     */
    int32_t (*enable_voltage_calib)(bool enable);

    /**
     * @brief 配置 TX 工作序列: 支持 PC 工具配置 TX 工作序列以支持 AA 调焦
     * @param mode TX 序列模式
     * @param chns 多通道模式时的通道列表
     * @param chn_num 多通道模式时的通道个数
     * @return 0 表示成功，非 0 表示失败
     */
    int32_t (*set_tx_seq_mode)(tx_seq_mode_e mode, const uint8_t *chns, uint16_t chn_num);

    /**
     * @brief 获取 TX 工作序列: 支持 PC 工具获取 TX 工作序列以了解当前 TX 的工作状态
     * @param mode TX 序列模式
     * @param chns 多通道模式时的通道列表
     * @param chn_num 多通道模式时的通道个数
     * @return 0 表示成功，非 0 表示失败
     */
    int32_t (*get_tx_seq_mode)(tx_seq_mode_e *mode, uint8_t *chns, uint16_t *chn_num);

    /**
     * @brief 设置 DAC 电压: 支持 PC 工具设置 DAC 电压
     * @param dac_dev DAC 设备类型
     * @param volt_mV 电压值(单位: mV)
     * @param dac_code 输出参数, 用于返回 DAC 电压对应的 DAC code 值(可传 NULL 表示不关心 DAC code)
     * @return 0 表示成功，非 0 表示失败
     */
    int32_t (*set_dac)(dac_device_e dac_dev, int32_t volt_mV, uint32_t *dac_code);

    /**
     * @brief 获取 ADC 电压: 支持 PC 工具获取 ADC 电压
     * @param adc_dev ADC 设备类型
     * @param volt_mV 输出参数, 用于返回 ADC 电压值(单位: mV)
     * @param raw_adc_mV 输出参数, 用于返回原始 ADC 电压值(可传 NULL 表示不关心 raw ADC), 单位: mV
     * @return 0 表示成功，非 0 表示失败
    */
    int32_t (*get_adc)(adc_device_e adc_dev, int32_t *volt_mV, uint32_t *raw_adc_mV);

    /**
     * @brief 存储 ADC 校准数据: 支持 PC 工具存储 ADC 校准数据
     * @param adc_dev ADC 设备类型
     * @param k 斜率参数
     * @param b 截距参数
     * @return 0 表示成功，非 0 表示失败
     */
    int32_t (*store_adc_calib)(adc_device_e adc_dev, float k, float b);

    /**
     * @brief 读取 ADC 校准数据: 支持 PC 工具读取 ADC 校准数据
     * @param adc_dev ADC 设备类型
     * @param k 输出参数, 用于返回斜率参数
     * @param b 输出参数, 用于返回截距参数
     * @return 0 表示成功，非 0 表示失败
     */
    int32_t (*read_adc_calib)(adc_device_e adc_dev, float *k, float *b);

    /**
     * @brief 移除 ADC 校准数据: 支持 PC 工具移除 ADC 校准数据
     * @param adc_dev ADC 设备类型
     * @return 0 表示成功，非 0 表示失败
     */
    int32_t (*remove_adc_calib)(adc_device_e adc_dev);

    /**
     * @brief 存储 DAC 校准数据: 支持 PC 工具存储 DAC 校准数据
     * @param dac_dev DAC 设备类型
     * @param k 斜率参数
     * @param b 截距参数
     * @return 0 表示成功，非 0 表示失败
     */
    int32_t (*store_dac_calib)(dac_device_e dac_dev, float k, float b);

    /**
     * @brief 读取 DAC 校准数据: 支持 PC 工具读取 DAC 校准数据
     * @param dac_dev DAC 设备类型
     * @param k 输出参数, 用于返回斜率参数
     * @param b 输出参数, 用于返回截距参数
     * @return 0 表示成功，非 0 表示失败
     */
    int32_t (*read_dac_calib)(dac_device_e dac_dev, float *k, float *b);

    /**
     * @brief 移除 DAC 校准数据: 支持 PC 工具移除 DAC 校准数据
     * @param adc_dev DAC 设备类型
     * @return 0 表示成功，非 0 表示失败
     */
    int32_t (*remove_dac_calib)(dac_device_e dac_dev);

    /**
     * @brief 存储算法校准数据: 支持 PC 工具存储算法校准数据
     * @param data 校准数据缓冲区指针
     * @param dlen 校准数据长度
     * @return 0 表示成功，非 0 表示失败
     */
    int32_t (*store_algo_calib)(const void *data, uint32_t dlen);

    /**
     * @brief 读取算法校准数据: 支持 PC 工具读取算法校准数据
     * @param data 输出参数, 用于返回校准数据缓冲区指针, 调用方申请的缓冲区最大尺寸由 dlen 参数传入
     * @param dlen 输入输出参数, 输入时表示传入的缓冲区长度, 输出时表示实际获取到的数据长度
     * @return 0 表示成功，非 0 表示失败
     */
    int32_t (*read_algo_calib)(void *data, uint32_t *dlen);

    /**
     * @brief 移除算法校准数据: 支持 PC 工具移除算法校准数据
     * @return 0 表示成功，非 0 表示失败
     */
    int32_t (*remove_algo_calib)(void);

    /**
     * @brief 存储工站数据: 支持 PC 工具存储量程工站数据
     * @param idx 工站编号, 用于区分不同的工站数据
     * @param data 工站数据缓冲区指针
     * @param dlen 工站数据长度
     * @return 0 表示成功，非 0 表示失败
     */
    int32_t (*store_factory_data)(uint32_t idx, const void *data, uint32_t dlen);

    /**
     * @brief 读取工站数据: 支持 PC 工具读取工站数据
     * @param idx 工站编号
     * @param data 输出参数, 用于返回工站数据缓冲区指针, 调用方申请的缓冲区最大尺寸由 dlen 参数传入
     * @param dlen 输入输出参数, 输入时表示传入的缓冲区长度, 输出时表示实际获取到的数据长度
     * @return 0 表示成功，非 0 表示失败
     */
    int32_t (*read_factory_data)(uint32_t idx, void *data, uint32_t *dlen);

    /**
     * @brief 移除工站数据: 支持 PC 工具移除工站数据
     * @param idx 工站编号
     * @return 0 表示成功，非 0 表示失败
     */
    int32_t (*remove_factory_data)(uint32_t idx);

    /**
     * @brief 写入 CFGDATA/CALIDATA: 支持 PC 工具通过 0x110 分片写入
     * @param req 写请求, 包含 data_type/total_len/offset/data 等信息
     * @return 0 表示成功，非 0 表示失败
     */
    int32_t (*write_cfgdata)(const pc_cfgdata_write_req_t *req);

    /**
     * @brief 读取 CFGDATA/CALIDATA: 支持 PC 工具通过 0x111 分片读取
     * @param req 读请求, 回调应填充 actual_total_len/data_len/data
     * @return 0 表示成功，非 0 表示失败
     */
    int32_t (*read_cfgdata)(pc_cfgdata_read_req_t *req);

    /**
     * @brief 中止当前 CFGDATA/CALIDATA 会话: TCP 断开或上层需要清理时调用
     */
    void (*abort_cfgdata)(void);

    /**
     * @brief 设备自测接口, 支持 PC 工具对指定设备进行自测
     * @param test_dev 待测对象
     * @param param1 预留参数1, 用于传入自测所需的额外参数
     * @param param2 预留参数2, 用于传入自测所需的额外参数
     * @return 0 表示成功，非 0 表示失败
     */
    int32_t (*self_test)(test_device_e test_dev, uint32_t param1, uint32_t param2);
} pc_callbacks;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 注册服务以支持阜时 PC 工具调用
 * @param callbacks 回调函数集合
 * @return 0表示成功，非0表示失败
 */
int32_t fspc_support_init(const pc_callbacks *callbacks);

/**
 * @brief 清理阜时 PC 工具支持相关资源
 */
void    fspc_cleanup(void);

/**
 * @brief 发送点云(point cloud)数据给 PC 工具
 * @return 0表示成功，非0表示失败
 */
int32_t fspc_send_pcl(void);

/**
 * @brief 发送芯片直出测距数据给 PC 工具
 * @return 0表示成功，非0表示失败
 */
int32_t fspc_send_range(void);

/**
 * @brief 发送芯片直出灰度图数据给 PC 工具
 * @return 0表示成功，非0表示失败
 */
int32_t fspc_send_gray(void);

/**
 * @brief 发送状态监控数据给 PC 工具
 * @return 0表示成功，非0表示失败
 */
int32_t fspc_send_monitor(void);

#ifdef __cplusplus
}
#endif

#endif /* __FSPC_SUPPORT_H__ */
