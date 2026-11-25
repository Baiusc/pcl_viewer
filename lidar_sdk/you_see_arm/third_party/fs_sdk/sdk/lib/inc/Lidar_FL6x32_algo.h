#pragma once
#include <string.h>
#include <fcntl.h>

#ifdef _WIN32
#include <io.h>
#include <process.h>
#pragma warning( disable : 4996 )
#define EXPORT_INTERFACE __declspec (dllexport)
#define INNER_HIDDEN
#endif   // WIN32
#ifdef __linux__
#include <unistd.h>
#include <getopt.h>
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#define EXPORT_INTERFACE
#define INNER_HIDDEN __attribute__((visibility("hidden")))
#endif // __linux__

#ifdef __cplusplus
extern "C" {
#endif
#pragma pack(push)
#pragma pack(1)  // one byte-aligned
    typedef enum {
        ALGO_PEAK_NUM_1 = 0,    // peak num = 1
        ALGO_PEAK_NUM_2,        // peak num = 2
        ALGO_PEAK_NUM_3,        // peak num = 3
        ALGO_PEAK_NUM_4,        // peak num = 4
        ALGO_PEAK_NUM_CNT,
    } algo_peak_num_e;

    typedef struct {
        uint16_t th_peak_addr;              // 寻峰结果 peak 地址(也就是 peak, 12bit)
        bool     crosstalk;                 // th_peak_addr 是否为串扰峰: false -> 否; true -> 是
        uint32_t dist_rp;                   // 信号峰上升沿位置对应的距离(16bit+8bit, 整数部分单位 cm)
        uint16_t fwhm;                      // 信号峰峰宽(5bit+8bit, 整数部分单位 bin)
        uint32_t conv_intensity;            // 信号峰卷积峰高(18bit)
        uint32_t dist_cent;                 // 信号峰质心结果对应的距离(16bit+8bit, 整数部分单位 cm)
        uint16_t hist_peak_max;             // 信号峰原始 bin 最大值(信号峰高, 12bit)
    } FL6x32_algo_result_t;

    EXPORT_INTERFACE typedef struct {
        uint16_t        pixel_id;           // 本Pixel在当前Slot内的编号，从0开始
        uint32_t        noise_amb;          // 寻峰算出的噪声阈值
        uint16_t        amb_num;            // laness统计的环境光总计数
        uint32_t        det_sum;            // laness统计的直方图总计数
        uint16_t        real_tx_num;        // laness统计的真实tx次数。
        uint16_t        lpwr_real_tx_num;   // 低功率的统计tx次数
        algo_peak_num_e peak_num;           // peak 数量;
        uint16_t row;                       // 物理行坐标
        uint16_t col;                       // 物理列坐标
        uint8_t  data[0];                   // 用于存放 peak_num * FL6x32_algo_result_t
    } FL6x32_range_pix_info_t;

    EXPORT_INTERFACE typedef struct {
        unsigned short  group_start;                        // roi起始group
        unsigned short  piece_start;                        // roi起始piece
        unsigned short  row_start;                          // roi起始行号
        unsigned short  col_start;                          // roi起始列号
        unsigned short  row_size;                           // roi行size
        unsigned short  col_size;                           // roi列size
        unsigned char   peak_num;                           // 峰数
        unsigned short  range_data_size;                    // rw数据size（目前直方图长度rw+4）
        unsigned short  binning_model[2];                   // [0]列binning-size,[1]行binning-size
        unsigned char   roll_mode;                          // 子帧模式: 0 -> 1 个子帧; 2/4: 2 或 4 个子帧
        unsigned char   roll_idx;                           // 1 个子帧：0, 2 个子帧：1/2, 4 个子帧：1/2/3/4
    } FL6x32_ROI_INFO;

    EXPORT_INTERFACE typedef struct {
        unsigned char   input_data_version;             // 0 range，1 ECHO
        unsigned char   calib_version;                  // dist校准版本
        unsigned char   output_data_size;               // 5 dist + xyz + refl
        unsigned char   ROI_cnt;                        // roi个数
        unsigned char   ROI_idx;                        // 当前roi-id
        FL6x32_ROI_INFO  ROI_info[4];                   // roi信息
        float           TX_temperature;                 // 温漂校准TX
        float           RX_temperature;                 // 温漂校准RX
        unsigned char   multithread_num;                // denoise开关
        unsigned char   neighb_denoise_switch;          // denoise开关
        unsigned char   backfill_switch;                // denoise开关
        unsigned char   dist_smooth_switch;             // denoise开关
        unsigned char   refl_smooth_switch;             // denoise开关
    } FL6x32_para;

#pragma pack(pop)

    EXPORT_INTERFACE char* Obtain_Version();
    /******************************
    * 初始化算法运行环境
    ******************************/
    EXPORT_INTERFACE int Init_Algo_Env(FL6x32_para* para_data, float* calib_coeff, const int calibDataLength);

    /******************************
    * 释放算法运行环境
    ******************************/
    EXPORT_INTERFACE int Clean_Algo_Env();

    /********************************************************************************************
    * 直方图-原始点云，算法接口
    * IC_input: 输入range数据,
    * rawCloudData: 输出原始点云（距离，反射率，峰形数据等）
    *********************************************************************************************/
    EXPORT_INTERFACE int Algo_GetRawCloudData(FL6x32_para* para_data, unsigned char* input_rangeData, const int inputDataLength,
                                              float* calib_coeff, const int calibDataLength);
    /********************************************************************************************
    * 原始点云-优化点云，算法接口
    * rawCloudData: 输入原始点云（距离，反射率，峰形数据等）,
    * outputCloudData: 输出最终点云（距离，反射率，坐标等）
    *********************************************************************************************/
    EXPORT_INTERFACE int Algo_GetOutputCloudData(FL6x32_para* para_data, unsigned char* input_rangeData, const int inputDataLength,
                                                 float* outputCloudData, int* outputDataLength);

    /********************************************************************************************
    * range—>点云，算法接口(多线程版本)
    * input_rangeData: 输入range数据,
    * outputCloudData: 输出最终点云（距离，反射率，坐标等）
    *********************************************************************************************/
    EXPORT_INTERFACE int Algo_Get_CloudData_multithread(FL6x32_para* para_data, unsigned short multithread_idx,
                                            unsigned char* input_rangeData, const int inputDataLength,
                                            float* outputCloudData, int* outputDataLength);

    int Obtain_algo_result(unsigned char* uint8_data, short* uint16_data, unsigned int* uint32_data, float* float32_data);

#ifdef __cplusplus
}
#endif
