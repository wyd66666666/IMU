#ifndef FUSION_FILTER_H
#define FUSION_FILTER_H

#include <stdint.h>

// 姿态数据结构体
typedef struct {
    float roll;         // 横滚角(度)
    float pitch;        // 俯仰角(度)
    float yaw;          // 偏航角(度)
    float altitude;     // 高度(米)
    float accel[3];     // 校准后的加速度
    float gyro[3];      // 校准后的角速度
    float mag[3];       // 校准后的磁场
    float confidence;   // 姿态置信度
} attitude_data_t;

// 初始化融合滤波器
void fusion_init(void);

// 设置参数
void fusion_set_params(float gyro_weight, float accel_weight, float mag_weight);

// 更新姿态估计
void fusion_update(float acc[3], float gyro[3], float mag[3], float baro_alt, float dt);

// 获取当前姿态
attitude_data_t fusion_get_attitude(void);

// 陀螺仪校准
void fusion_set_gyro_bias(float bias[3]);

// 启用/禁用磁力计
void fusion_use_magnetometer(int enable);

// 获取调试信息
const char* fusion_get_debug_info(void);

#endif /* FUSION_FILTER_H */