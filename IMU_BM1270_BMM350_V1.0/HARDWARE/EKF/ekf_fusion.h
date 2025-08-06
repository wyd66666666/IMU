#ifndef EKF_FUSION_H
#define EKF_FUSION_H

#include <stdint.h>
#include <math.h>

// 姿态结构体
typedef struct {
    float roll;        // 横滚角(度)
    float pitch;       // 俯仰角(度)
    float yaw;         // 偏航角(度)
    float altitude;    // 高度(米)
    float q[4];        // 四元数
    float bias[3];     // 陀螺仪偏差
    float confidence;  // 姿态置信度(0-1)
} ekf_attitude_t;

// 初始化EKF
void ekf_init(void);

// 设置初始状态
void ekf_set_initial_state(float q[4], float gyro_bias[3]);

// 传感器数据融合
void ekf_update(float acc[3], float gyro[3], float mag[3], float baro_alt, float dt);

// 获取当前姿态
ekf_attitude_t ekf_get_attitude(void);

// 设置EKF参数
void ekf_set_process_noise(float gyro_noise, float gyro_bias_noise);
void ekf_set_measurement_noise(float acc_noise, float mag_noise, float baro_noise);

// 检测并处理磁力计干扰
void ekf_set_mag_disturbance_detection(float threshold);

// 动态增益调整
void ekf_enable_adaptive_gain(int enable);

#endif /* EKF_FUSION_H */