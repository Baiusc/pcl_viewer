#ifndef __IMU_SENSOR_H__
#define __IMU_SENSOR_H__
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
//校准时长 单位s
#define ADJUST_COST 3

/*
#define IIO_G_TO_M_S_2        (                 g        )           ((g) * 980665ULL / 100000ULL)
	2g  0.061 61	980665	100000	598.20565
	4g  0.122 122	980665	100000	1196.4113
	8g  0.244 244	980665	100000	2392.8226
	16g 0.488 488	980665	100000	4785.6452

#define IIO_DEGREE_TO_RAD        (                 deg        )           (((deg) * 314159ULL + 9000000ULL) / 18000000ULL)
	250  8.75 8750	314159	9000000	18000000	153.2161806
	500  17.5 17500	314159	9000000	18000000	305.9323611
	1000 35   35000	314159	9000000	18000000	611.3647222
	2000 70   70000	314159	9000000	18000000	1222.229444
*/

#define FS_IMU_MSG_10Hz   0
#define IMU_DEFAULT_SLAVE_ADDR 0x60
typedef struct {
    int64_t timestamp; // Unit: ns
    int16_t temperature; // Unit: 1/256 ℃
    float gyro_x;   // Unit: rad/s
    float gyro_y;   // Unit: rad/s
    float gyro_z;   // Unit: rad/s
    float acc_x;    // Unit: g
    float acc_y;    // Unit: g
    float acc_z;    // Unit: g
}imu_sensor_data_t;
typedef struct {
    float offset_gyro_x;
    float offset_gyro_y;
    float offset_gyro_z;
}imu_offset_t;

int init_imu_sensor(char *devname);
bool isOk();
bool isInitParam(void);
int setImuParam(void);
int getImuData(imu_sensor_data_t * imu_data, uint32_t count);
int getImuAvailableList(const char *node, int32_t dev_idx, float *data, uint32_t *count);
int getImuIntList(const char *node, int32_t dev_idx, int32_t *data, uint32_t *count);
int setImuFloatData(const char *node, int32_t dev_idx, float dat);
int setImuIntData(const char *node, int32_t dev_idx, int32_t data);
int setImuScaleAcc(float scale); // only 0.000598 0.001196 0.002392 0.004785
int getImuScaleAcc(float * scale);
int getImuScaleAvailableAcc(float *scale, uint32_t *count);
int setImuScaleGyr(float scale); // only 0.000153 0.000305 0.000611 0.001222
int getImuScaleGyr(float * scale);
int getImuScaleAvailableGyr(float *scale, uint32_t *count);
int setImuFreqAcc(int32_t  freq); // 13 208 833 1660 3330 6660
int getImuFreqAcc(int32_t *freq);
int getImuFreqAvailableAcc(int32_t *freq, uint32_t *count);
int setImuFreqGyr(int32_t  freq);
int getImuFreqGyr(int32_t *freq);
int getImuFreqAvailableGyr(int32_t *freq, uint32_t *count);
void setIsAdjusting(bool isAdjusting);
int read_imu_offset(imu_offset_t *offset);
int write_imu_offset(imu_offset_t *offset);
bool is_imu_offset_exist();
int start_adjust_until_end();
int i2c_read(uint8_t regaddr, uint8_t *buff, uint16_t len);
int i2c_write(uint8_t regaddr, uint8_t *buff, uint16_t len);
int32_t getNodeValue(const char *node, int32_t dev_idx,       void *buff, uint32_t len);
int32_t setNodeValue(const char *node, int32_t dev_idx, const void *buff, uint32_t len);


#endif
