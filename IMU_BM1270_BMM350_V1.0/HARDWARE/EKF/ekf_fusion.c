#include "ekf_fusion.h"
#include <string.h>
#include <stdio.h>

#define DEG_TO_RAD (0.017453292519943295f)
#define RAD_TO_DEG (57.29577951308232f)

// EKF 状态变量
static float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};  // 四元数
static float bias[3] = {0.0f, 0.0f, 0.0f};     // 陀螺仪偏差
static float P[7][7] = {{0}};                  // 协方差矩阵

// EKF 参数
static float process_noise_gyro = 0.001f;       // 陀螺仪过程噪声
static float process_noise_bias = 0.0001f;      // 偏差过程噪声
static float meas_noise_acc = 0.1f;            // 加速度计测量噪声
static float meas_noise_mag = 10.0f;           // 磁力计测量噪声
static float meas_noise_baro = 1.0f;           // 气压计测量噪声
static float mag_disturbance_threshold = 0.1f;  // 磁干扰阈值
static int adaptive_gain_enabled = 1;           // 自适应增益使能

// 上一次的气压高度
static float last_baro_alt = 0.0f;
static int baro_initialized = 0;

// 姿态输出
static ekf_attitude_t current_attitude = {0};

// 内部函数声明
static void quaternion_normalize(float q[4]);
static void quaternion_to_euler(const float q[4], float* roll, float* pitch, float* yaw);
static float vector_norm(const float v[3]);
static float vector_dot(const float a[3], const float b[3]);
static void vector_cross(const float a[3], const float b[3], float result[3]);
static void update_step(float gyro[3], float dt);
static void correct_step_acc(float acc[3]);
static void correct_step_mag(float mag[3]);
static void correct_step_baro(float baro_alt, float dt);
static float calculate_acceleration_confidence(float acc[3]);
static float calculate_magnetic_confidence(float mag[3], float acc[3]);

// 初始化EKF
void ekf_init(void)
{
    // 初始化四元数为单位四元数
    q[0] = 1.0f;
    q[1] = q[2] = q[3] = 0.0f;
    
    // 陀螺仪偏差初始化为0
    bias[0] = bias[1] = bias[2] = 0.0f;
    
    // 初始化协方差矩阵
    memset(P, 0, sizeof(P));
    for (int i = 0; i < 7; i++) {
        P[i][i] = 0.1f;  // 对角线元素初始化
    }
    
    // 重置姿态输出
    memset(&current_attitude, 0, sizeof(ekf_attitude_t));
    current_attitude.q[0] = 1.0f;
    current_attitude.confidence = 0.5f;
    
    printf("EKF initialized\n");
}

// 设置初始状态
void ekf_set_initial_state(float init_q[4], float gyro_bias[3])
{
    if (init_q) {
        for (int i = 0; i < 4; i++) {
            q[i] = init_q[i];
        }
        quaternion_normalize(q);
    }
    
    if (gyro_bias) {
        for (int i = 0; i < 3; i++) {
            bias[i] = gyro_bias[i];
        }
    }
    
    // 更新当前姿态
    quaternion_to_euler(q, &current_attitude.roll, &current_attitude.pitch, &current_attitude.yaw);
    memcpy(current_attitude.q, q, sizeof(float) * 4);
    memcpy(current_attitude.bias, bias, sizeof(float) * 3);
    
    printf("EKF initial state set\n");
}

// 主要融合更新函数
void ekf_update(float acc[3], float gyro[3], float mag[3], float baro_alt, float dt)
{
    // 1. 预测步骤（时间更新）
    update_step(gyro, dt);
    
    // 2. 校正步骤（测量更新）
    
    // 加速度计校正（用于Roll和Pitch）
    float acc_confidence = calculate_acceleration_confidence(acc);
    if (acc_confidence > 0.1f) {
        correct_step_acc(acc);
    }
    
    // 磁力计校正（用于Yaw）
    if (mag && mag[0] != 0.0f && mag[1] != 0.0f && mag[2] != 0.0f) {
        float mag_confidence = calculate_magnetic_confidence(mag, acc);
        if (mag_confidence > 0.1f) {
            correct_step_mag(mag);
        }
    }
    
    // 气压计校正（用于高度）
    if (baro_alt != 0.0f) {
        if (!baro_initialized) {
            last_baro_alt = baro_alt;
            baro_initialized = 1;
        } else {
            correct_step_baro(baro_alt, dt);
        }
        current_attitude.altitude = baro_alt;
    }
    
    // 更新姿态输出
    quaternion_to_euler(q, &current_attitude.roll, &current_attitude.pitch, &current_attitude.yaw);
    memcpy(current_attitude.q, q, sizeof(float) * 4);
    memcpy(current_attitude.bias, bias, sizeof(float) * 3);
    
    // 计算总体置信度
    current_attitude.confidence = 0.5f * acc_confidence;
    if (mag && mag[0] != 0.0f && mag[1] != 0.0f && mag[2] != 0.0f) {
        current_attitude.confidence += 0.5f * calculate_magnetic_confidence(mag, acc);
    }
}

// 获取当前姿态
ekf_attitude_t ekf_get_attitude(void)
{
    return current_attitude;
}

// 设置EKF过程噪声参数
void ekf_set_process_noise(float gyro_noise, float gyro_bias_noise)
{
    process_noise_gyro = gyro_noise;
    process_noise_bias = gyro_bias_noise;
}

// 设置EKF测量噪声参数
void ekf_set_measurement_noise(float acc_noise, float mag_noise, float baro_noise)
{
    meas_noise_acc = acc_noise;
    meas_noise_mag = mag_noise;
    meas_noise_baro = baro_noise;
}

// 设置磁力计干扰检测阈值
void ekf_set_mag_disturbance_detection(float threshold)
{
    mag_disturbance_threshold = threshold;
}

// 启用/禁用自适应增益
void ekf_enable_adaptive_gain(int enable)
{
    adaptive_gain_enabled = enable;
}

// 内部函数实现

// 四元数归一化
static void quaternion_normalize(float q[4])
{
    float norm = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (norm > 0.0f) {
        float inv_norm = 1.0f / norm;
        q[0] *= inv_norm;
        q[1] *= inv_norm;
        q[2] *= inv_norm;
        q[3] *= inv_norm;
    }
}

// 四元数转欧拉角
static void quaternion_to_euler(const float q[4], float* roll, float* pitch, float* yaw)
{
    // Roll (x-axis rotation)
    float sinr_cosp = 2.0f * (q[0] * q[1] + q[2] * q[3]);
    float cosr_cosp = 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]);
    *roll = atan2f(sinr_cosp, cosr_cosp) * RAD_TO_DEG;

    // Pitch (y-axis rotation)
    float sinp = 2.0f * (q[0] * q[2] - q[3] * q[1]);
    if (fabsf(sinp) >= 1.0f)
        *pitch = copysignf(90.0f, sinp); // 使用90度或-90度
    else
        *pitch = asinf(sinp) * RAD_TO_DEG;

    // Yaw (z-axis rotation)
    float siny_cosp = 2.0f * (q[0] * q[3] + q[1] * q[2]);
    float cosy_cosp = 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]);
    *yaw = atan2f(siny_cosp, cosy_cosp) * RAD_TO_DEG;
}

// 向量模长
static float vector_norm(const float v[3])
{
    return sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

// 向量点积
static float vector_dot(const float a[3], const float b[3])
{
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

// 向量叉积
static void vector_cross(const float a[3], const float b[3], float result[3])
{
    result[0] = a[1]*b[2] - a[2]*b[1];
    result[1] = a[2]*b[0] - a[0]*b[2];
    result[2] = a[0]*b[1] - a[1]*b[0];
}

// 简化的卡尔曼滤波预测步骤
static void update_step(float gyro[3], float dt)
{
    // 1. 去除偏差
    float gyro_unbiased[3];
    for (int i = 0; i < 3; i++) {
        gyro_unbiased[i] = gyro[i] * DEG_TO_RAD - bias[i];
    }
    
    // 2. 四元数积分
    float dq[4];
    dq[0] = 0.5f * (-q[1]*gyro_unbiased[0] - q[2]*gyro_unbiased[1] - q[3]*gyro_unbiased[2]);
    dq[1] = 0.5f * (q[0]*gyro_unbiased[0] + q[2]*gyro_unbiased[2] - q[3]*gyro_unbiased[1]);
    dq[2] = 0.5f * (q[0]*gyro_unbiased[1] - q[1]*gyro_unbiased[2] + q[3]*gyro_unbiased[0]);
    dq[3] = 0.5f * (q[0]*gyro_unbiased[2] + q[1]*gyro_unbiased[1] - q[2]*gyro_unbiased[0]);
    
    // 3. 更新四元数
    for (int i = 0; i < 4; i++) {
        q[i] += dq[i] * dt;
    }
    quaternion_normalize(q);
    
    // 4. 更新协方差矩阵 (简化版)
    // 在完整的EKF中，我们会更新完整的协方差矩阵
    // 这里为了简化，只增加对角线元素
    for (int i = 0; i < 4; i++) {
        P[i][i] += process_noise_gyro * dt;
    }
    for (int i = 4; i < 7; i++) {
        P[i][i] += process_noise_bias * dt;
    }
}

// 加速度计校正
static void correct_step_acc(float acc[3])
{
    // 1. 归一化加速度
    float acc_norm = vector_norm(acc);
    if (acc_norm < 0.1f) return; // 加速度太小，可能不可靠
    
    float acc_normalized[3];
    for (int i = 0; i < 3; i++) {
        acc_normalized[i] = acc[i] / acc_norm;
    }
    
    // 2. 从四元数计算重力方向
    float grav[3];
    grav[0] = 2.0f * (q[1]*q[3] - q[0]*q[2]);
    grav[1] = 2.0f * (q[0]*q[1] + q[2]*q[3]);
    grav[2] = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
    
    // 3. 计算误差（加速度和估计重力的叉积）
    float err[3];
    vector_cross(acc_normalized, grav, err);
    
    // 4. 根据误差调整四元数和偏差
    // 调整增益（可以根据加速度的置信度动态调整）
    float K = 0.1f;
    if (adaptive_gain_enabled) {
        float acc_confidence = calculate_acceleration_confidence(acc);
        K *= acc_confidence;
    }
    
    // 应用校正
    for (int i = 0; i < 3; i++) {
        bias[i] += 0.01f * err[i];
    }
    
    // 更新四元数
    q[0] += K * (err[0]*q[1] + err[1]*q[2] + err[2]*q[3]);
    q[1] += K * (-err[0]*q[0] + err[2]*q[2] - err[1]*q[3]);
    q[2] += K * (-err[1]*q[0] - err[2]*q[1] + err[0]*q[3]);
    q[3] += K * (-err[2]*q[0] + err[1]*q[1] - err[0]*q[2]);
    
    quaternion_normalize(q);
}

// 磁力计校正（仅校正偏航角）
static void correct_step_mag(float mag[3])
{
    // 1. 归一化磁场强度
    float mag_norm = vector_norm(mag);
    if (mag_norm < 0.1f) return; // 磁场太弱
    
    float mag_normalized[3];
    for (int i = 0; i < 3; i++) {
        mag_normalized[i] = mag[i] / mag_norm;
    }
    
    // 2. 从四元数计算当前磁北方向
    float mx, my;
    float q0q0 = q[0] * q[0];
    float q0q1 = q[0] * q[1];
    float q0q2 = q[0] * q[2];
    float q0q3 = q[0] * q[3];
    float q1q1 = q[1] * q[1];
    float q1q2 = q[1] * q[2];
    float q1q3 = q[1] * q[3];
    float q2q2 = q[2] * q[2];
    float q2q3 = q[2] * q[3];
    float q3q3 = q[3] * q[3];
    
    mx = 2.0f * (mag_normalized[0] * (0.5f - q2q2 - q3q3) + mag_normalized[1] * (q1q2 - q0q3) + mag_normalized[2] * (q1q3 + q0q2));
    my = 2.0f * (mag_normalized[0] * (q1q2 + q0q3) + mag_normalized[1] * (0.5f - q1q1 - q3q3) + mag_normalized[2] * (q2q3 - q0q1));
    
    // 3. 计算偏航误差
    float heading_err = atan2f(my, mx) - atan2f(0.0f, 1.0f); // 理想情况应该是北向
    
    // 4. 构建偏航误差向量（垂直于当前姿态）
    float err[3];
    err[0] = 0.0f;
    err[1] = 0.0f;
    err[2] = heading_err * 0.1f; // 缩小增益，磁力计校正主要影响偏航
    
    // 5. 应用校正
    float K = 0.01f; // 较小的增益，避免过度依赖磁力计
    
    if (adaptive_gain_enabled) {
        float mag_confidence = calculate_magnetic_confidence(mag, NULL);
        K *= mag_confidence;
    }
    
    // 更新四元数
    q[0] += K * (-q[3] * err[2]);
    q[1] += K * (-q[2] * err[2]);
    q[2] += K * (q[1] * err[2]);
    q[3] += K * (q[0] * err[2]);
    
    quaternion_normalize(q);
}

// 气压计校正（简化版本，仅用于记录高度）
static void correct_step_baro(float baro_alt, float dt)
{
    // 简单的高度更新
    float alt_diff = baro_alt - last_baro_alt;
    last_baro_alt = baro_alt;
    
    // 这里可以添加垂直加速度补偿等
    // 简化版中仅记录高度
}

// 计算加速度计测量的可信度
static float calculate_acceleration_confidence(float acc[3])
{
    // 计算加速度大小
    float acc_magnitude = vector_norm(acc);
    
    // 与标准重力(9.81m/s²)比较
    float gravity_error = fabsf(acc_magnitude - 9.81f) / 9.81f;
    
    // 转换为0-1的置信度
    float confidence = 1.0f - fminf(gravity_error, 1.0f);
    
    // 当加速度约等于重力时，置信度高
    // 当有额外加速度时，置信度降低
    return confidence;
}

// 计算磁力计测量的可信度
static float calculate_magnetic_confidence(float mag[3], float acc[3])
{
    // 1. 磁场强度检查
    float mag_norm = vector_norm(mag);
    float mag_confidence = 1.0f;
    
    // 检查磁场强度是否在合理范围内
    // 地球磁场强度约为25-65微特斯拉
    if (mag_norm < 10.0f || mag_norm > 100.0f) {
        mag_confidence *= 0.5f;
    }
    
    // 2. 如果有加速度数据，检查磁场是否与重力垂直
    // 地球磁场与重力方向不应完全平行
    if (acc) {
        float acc_norm = vector_norm(acc);
        if (acc_norm > 0.1f) {
            float normalized_acc[3], normalized_mag[3];
            for (int i = 0; i < 3; i++) {
                normalized_acc[i] = acc[i] / acc_norm;
                normalized_mag[i] = mag[i] / mag_norm;
            }
            
            // 计算角度差异
            float dot_product = fabsf(vector_dot(normalized_acc, normalized_mag));
            // 如果磁场和重力近乎平行，减少置信度
            if (dot_product > 0.8f) {
                mag_confidence *= (1.0f - (dot_product - 0.8f) * 5.0f);
            }
        }
    }
    
    return fmaxf(mag_confidence, 0.0f);
}