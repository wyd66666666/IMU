#ifndef ANONY_PROTOCOL_H
#define ANONY_PROTOCOL_H

#include <stdint.h>

// 字节操作宏
#define BYTE0(x) ((uint8_t)((x) & 0xFF))
#define BYTE1(x) ((uint8_t)(((x) >> 8) & 0xFF))
#define BYTE2(x) ((uint8_t)(((x) >> 16) & 0xFF))
#define BYTE3(x) ((uint8_t)(((x) >> 24) & 0xFF))

// 匿名协议V8帧头定义
#define ANONY_FRAME_HEAD1 0xAB
#define ANONY_FRAME_HEAD2 0xFF

// 设备地址定义
#define DEVICE_ADDR_IMU   0xDD  // 匿名IMU设备地址
#define DEVICE_ADDR_HOST  0xFE  // 上位机地址

// 功能码定义 - 基本信息类帧
#define FUNC_INERTIAL_SENSOR   0x01  // 惯性传感器数据
#define FUNC_MAG_TEMP_SENSOR   0x02  // 罗盘、温度传感器数据  
#define FUNC_ATTITUDE_EULER    0x03  // 飞控姿态：欧拉角格式
#define FUNC_ATTITUDE_QUAT     0x04  // 飞控姿态：四元数格式
#define FUNC_ALTITUDE_DATA     0x05  // 高度数据
#define FUNC_FLIGHT_MODE       0x06  // 飞控运行模式
#define FUNC_FLIGHT_SPEED      0x07  // 飞行速度数据
#define FUNC_POSITION_OFFSET   0x08  // 位置偏移数据
#define FUNC_WIND_ESTIMATE     0x09  // 风速估计
#define FUNC_TARGET_ATTITUDE   0x0A  // 目标姿态数据
#define FUNC_TARGET_SPEED      0x0B  // 目标速度数据
#define FUNC_RTH_INFO          0x0C  // 回航信息
#define FUNC_POWER_INFO        0x0D  // 电压电流数据
#define FUNC_MODULE_STATUS     0x0E  // 外接模块工作状态
#define FUNC_RGB_BRIGHTNESS    0x0F  // RGB亮度信息输出

// 功能码定义 - 灵活格式帧
#define FUNC_USER_DATA_F1      0xF1  // 用户自定义数据帧1
#define FUNC_USER_DATA_F2      0xF2  // 用户自定义数据帧2
#define FUNC_USER_DATA_F3      0xF3  // 用户自定义数据帧3
#define FUNC_USER_DATA_F4      0xF4  // 用户自定义数据帧4
#define FUNC_USER_DATA_F5      0xF5  // 用户自定义数据帧5

// 函数声明
void ANO_Send_Attitude(float roll, float pitch, float yaw);
void ANO_Send_InertialSensor(int16_t acc_x, int16_t acc_y, int16_t acc_z,
                             int16_t gyro_x, int16_t gyro_y, int16_t gyro_z,
                             uint8_t shock_sta);
void ANO_Send_MagTempSensor(int16_t mag_x, int16_t mag_y, int16_t mag_z,
                            int16_t temp, uint8_t mag_sta);
void ANO_Send_Sensor(int16_t acc_x, int16_t acc_y, int16_t acc_z,
                     int16_t gyro_x, int16_t gyro_y, int16_t gyro_z,
                     int16_t mag_x, int16_t mag_y, int16_t mag_z);
void ANO_Send_SensorRaw(int16_t acc_x, int16_t acc_y, int16_t acc_z,
                        int16_t gyro_x, int16_t gyro_y, int16_t gyro_z,
                        int16_t mag_x, int16_t mag_y, int16_t mag_z);
void ANO_Send_UserData(float d1, float d2, float d3, float d4, float d5);
void ANO_Send_LinkData(uint8_t state);
void ANO_Send_Test(void);

// 校验和计算函数
uint8_t ANO_Calculate_SumCheck(uint8_t *data, uint16_t len);
uint8_t ANO_Calculate_AddCheck(uint8_t *data, uint16_t len);

// 测试函数
void ANO_Protocol_Test(void);
void ANO_Checksum_Test(void);

#endif

