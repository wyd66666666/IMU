#include "anony_protocol.h"
#include "usart.h"
#include <string.h>

// 基础协议构建函数
static void ANO_Build_Frame(uint8_t s_addr, uint8_t d_addr, uint8_t func_id, uint8_t *data, uint16_t data_len, uint8_t *frame);
static uint16_t ANO_Calculate_Checksum(uint8_t *data, uint16_t len, uint8_t *sum_check, uint8_t *add_check);

/**
 * @brief 计算和校验
 * @param data 数据指针  
 * @param len 数据长度
 * @return 和校验值
 */
uint8_t ANO_Calculate_SumCheck(uint8_t *data, uint16_t len)
{
    uint8_t sum = 0;
    for(uint16_t i = 0; i < len; i++)
    {
        sum += data[i];
    }
    return sum;
}

/**
 * @brief 计算附加校验
 * @param data 数据指针
 * @param len 数据长度  
 * @return 附加校验值
 */
uint8_t ANO_Calculate_AddCheck(uint8_t *data, uint16_t len)
{
    uint8_t sum = 0;
    uint8_t add = 0;
    for(uint16_t i = 0; i < len; i++)
    {
        sum += data[i];
        add += sum;
    }
    return add;
}

/**
 * @brief 计算校验和
 * @param data 数据指针
 * @param len 数据长度
 * @param sum_check 和校验值输出
 * @param add_check 附加校验值输出
 * @return 总长度
 */
static uint16_t ANO_Calculate_Checksum(uint8_t *data, uint16_t len, uint8_t *sum_check, uint8_t *add_check)
{
    uint8_t sum = 0;
    uint8_t add = 0;
    
    for(uint16_t i = 0; i < len; i++)
    {
        sum += data[i];
        add += sum;
    }
    
    *sum_check = sum;
    *add_check = add;
    
    return len;
}

/**
 * @brief 构建完整的匿名协议数据帧
 * @param s_addr 源地址
 * @param d_addr 目标地址
 * @param func_id 功能码
 * @param data 数据内容
 * @param data_len 数据长度
 * @param frame 输出帧缓冲区
 */
static void ANO_Build_Frame(uint8_t s_addr, uint8_t d_addr, uint8_t func_id, uint8_t *data, uint16_t data_len, uint8_t *frame)
{
    uint8_t sum_check, add_check;
    uint16_t frame_len = 6 + data_len; // 帧头(1) + 源地址(1) + 目标地址(1) + 功能码(1) + 数据长度(2) + 数据(n)
    
    // 构建帧头部
    frame[0] = ANONY_FRAME_HEAD1;    // 帧头
    frame[1] = s_addr;               // 源地址
    frame[2] = d_addr;               // 目标地址
    frame[3] = func_id;              // 功能码
    frame[4] = BYTE0(data_len);      // 数据长度低字节
    frame[5] = BYTE1(data_len);      // 数据长度高字节
    
    // 复制数据
    if(data != NULL && data_len > 0)
    {
        memcpy(&frame[6], data, data_len);
    }
    
    // 计算校验和
    ANO_Calculate_Checksum(frame, frame_len, &sum_check, &add_check);
    
    // 添加校验和
    frame[frame_len] = sum_check;
    frame[frame_len + 1] = add_check;
    
    // 发送数据帧
    HAL_UART_Transmit(&huart1, frame, frame_len + 2, 100);
}

/**
 * @brief 发送姿态角数据 - ID: 0x03：飞控姿态：欧拉角格式
 * @param roll 横滚角(度)
 * @param pitch 俯仰角(度)  
 * @param yaw 偏航角(度)
 */
void ANO_Send_Attitude(float roll, float pitch, float yaw)
{
    uint8_t data[7];
    int16_t temp;
    
    // Roll角度，精确到0.01度
    temp = (int16_t)(roll * 100);
    data[0] = BYTE0(temp);
    data[1] = BYTE1(temp);
    
    // Pitch角度，精确到0.01度
    temp = (int16_t)(pitch * 100);
    data[2] = BYTE0(temp);
    data[3] = BYTE1(temp);
    
    // Yaw角度，精确到0.01度
    temp = (int16_t)(yaw * 100);
    data[4] = BYTE0(temp);
    data[5] = BYTE1(temp);
    
    // 融合状态
    data[6] = 1;  // 假设融合正常
    
    uint8_t frame[16];
    ANO_Build_Frame(DEVICE_ADDR_IMU, DEVICE_ADDR_HOST, FUNC_ATTITUDE_EULER, data, 7, frame);
}

/**
 * @brief 发送惯性传感器数据 - ID: 0x01：惯性传感器数据
 * @param acc_x, acc_y, acc_z 加速度计数据，单位cm/s²
 * @param gyro_x, gyro_y, gyro_z 陀螺仪数据，32768对应2000deg/s
 * @param shock_sta 震动状态
 */
void ANO_Send_InertialSensor(int16_t acc_x, int16_t acc_y, int16_t acc_z,
                             int16_t gyro_x, int16_t gyro_y, int16_t gyro_z,
                             uint8_t shock_sta)
{
    uint8_t data[13];
    
    // 加速度计数据
    data[0] = BYTE0(acc_x);
    data[1] = BYTE1(acc_x);
    data[2] = BYTE0(acc_y);
    data[3] = BYTE1(acc_y);
    data[4] = BYTE0(acc_z);
    data[5] = BYTE1(acc_z);
    
    // 陀螺仪数据
    data[6] = BYTE0(gyro_x);
    data[7] = BYTE1(gyro_x);
    data[8] = BYTE0(gyro_y);
    data[9] = BYTE1(gyro_y);
    data[10] = BYTE0(gyro_z);
    data[11] = BYTE1(gyro_z);
    
    // 震动状态
    data[12] = shock_sta;
    
    uint8_t frame[22];
    ANO_Build_Frame(DEVICE_ADDR_IMU, DEVICE_ADDR_HOST, FUNC_INERTIAL_SENSOR, data, 13, frame);
}

/**
 * @brief 发送磁力计温度传感器数据 - ID: 0x02：罗盘、温度传感器数据
 * @param mag_x, mag_y, mag_z 磁力计数据
 * @param temp 温度，放大10倍传输，0.1摄氏度
 * @param mag_sta 罗盘状态
 */
void ANO_Send_MagTempSensor(int16_t mag_x, int16_t mag_y, int16_t mag_z,
                            int16_t temp, uint8_t mag_sta)
{
    uint8_t data[9];
    
    // 磁力计数据
    data[0] = BYTE0(mag_x);
    data[1] = BYTE1(mag_x);
    data[2] = BYTE0(mag_y);
    data[3] = BYTE1(mag_y);
    data[4] = BYTE0(mag_z);
    data[5] = BYTE1(mag_z);
    
    // 温度数据
    data[6] = BYTE0(temp);
    data[7] = BYTE1(temp);
    
    // 磁力计状态
    data[8] = mag_sta;
    
    uint8_t frame[18];
    ANO_Build_Frame(DEVICE_ADDR_IMU, DEVICE_ADDR_HOST, FUNC_MAG_TEMP_SENSOR, data, 9, frame);
}

/**
 * @brief 发送传感器数据(兼容旧版本)
 */
void ANO_Send_Sensor(int16_t acc_x, int16_t acc_y, int16_t acc_z,
                     int16_t gyro_x, int16_t gyro_y, int16_t gyro_z,
                     int16_t mag_x, int16_t mag_y, int16_t mag_z)
{
    // 发送惯性传感器数据
    ANO_Send_InertialSensor(acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, 0);
    
    // 发送磁力计数据
    ANO_Send_MagTempSensor(mag_x, mag_y, mag_z, 250, 1); // 假设温度25度，状态正常
}

/**
 * @brief SensorRaw函数(与Sensor相同)
 */
void ANO_Send_SensorRaw(int16_t acc_x, int16_t acc_y, int16_t acc_z,
                        int16_t gyro_x, int16_t gyro_y, int16_t gyro_z,
                        int16_t mag_x, int16_t mag_y, int16_t mag_z)
{
    ANO_Send_Sensor(acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z);
}

/**
 * @brief 发送用户自定义数据 - ID: 0xF1：灵活格式帧
 * @param d1-d5 5个float值，转为int16_t发送
 */
void ANO_Send_UserData(float d1, float d2, float d3, float d4, float d5)
{
    uint8_t data[10];
    int16_t temp;
    
    // 5个float值，转为int16_t发送
    temp = (int16_t)(d1 * 100);
    data[0] = BYTE0(temp);
    data[1] = BYTE1(temp);
    
    temp = (int16_t)(d2 * 100);
    data[2] = BYTE0(temp);
    data[3] = BYTE1(temp);
    
    temp = (int16_t)(d3 * 100);
    data[4] = BYTE0(temp);
    data[5] = BYTE1(temp);
    
    temp = (int16_t)(d4 * 100);
    data[6] = BYTE0(temp);
    data[7] = BYTE1(temp);
    
    temp = (int16_t)(d5 * 100);
    data[8] = BYTE0(temp);
    data[9] = BYTE1(temp);
    
    uint8_t frame[19];
    ANO_Build_Frame(DEVICE_ADDR_IMU, DEVICE_ADDR_HOST, FUNC_USER_DATA_F1, data, 10, frame);
}

/**
 * @brief 发送连接指令 - 用于调试和状态指示
 * @param state 连接状态
 */
void ANO_Send_LinkData(uint8_t state)
{
    uint8_t data[1];
    data[0] = state;
    
    uint8_t frame[10];
    ANO_Build_Frame(DEVICE_ADDR_IMU, DEVICE_ADDR_HOST, 0xA0, data, 1, frame);
}

/**
 * @brief 发送测试数据包
 */
void ANO_Send_Test(void)
{
    // 固定姿态数据 - Roll=10°, Pitch=20°, Yaw=30°
    ANO_Send_Attitude(10.0f, 20.0f, 30.0f);
}