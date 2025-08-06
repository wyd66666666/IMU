#include "fusion_filter.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

#define DEG_TO_RAD (PI / 180.0f)
#define RAD_TO_DEG (180.0f / PI)

// 内部状态
static float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};  // 四元数
static float gyro_bias[3] = {0};               // 陀螺仪偏差
static float accel_gravity[3] = {0, 0, 1.0f};  // 重力方向 (归一化)
static float mag_north[3] = {1.0f, 0, 0};      // 磁北方向 (归一化)
static float baro_alt_filtered = 0;            // 滤波后的气压高度

// 参数
static float gyro_weight = 0.98f;              // 陀螺仪权重
static float accel_weight = 0.02f;             // 加速度计权重
static float mag_weight = 0.01f;               // 磁力计权重
static float baro_weight = 0.05f;              // 气压计权重

// 配置
static int use_mag = 0;                        // 是否使用磁力计
static int initialized = 0;                    // 初始化标志
static char debug_info[128];                   // 调试信息

// 输出
static attitude_data_t current_attitude = {0};

// 内部函数声明
static void quaternion_normalize(float q[4]);
static void quaternion_to_euler(const float q[4], float* roll, float* pitch, float* yaw);
static float vector_norm(const float v[3]);
static void vector_normalize(float v[3]);
static float vector_dot(const float a[3], const float b[3]);
static void vector_cross(const float a[3], const float b[3], float result[3]);
static void update_from_gyro(float gyro[3], float dt);
static void correct_from_accel(float acc[3]);
static void correct_from_mag(float mag[3]);
static float calculate_acc_confidence(float acc[3]);

// 初始化融合滤波器
void fusion_init(void)
{
    // 初始化四元数为单位四元数
    q[0] = 1.0f;
    q[1] = q[2] = q[3] = 0.0f;
    
    // 初始化其他变量
    memset(gyro_bias, 0, sizeof(gyro_bias));
    accel_gravity[0] = accel_gravity[1] = 0;
    accel_gravity[2] = 1.0f;
    mag_north[0] = 1.0f;
    mag_north[1] = mag_north[2] = 0;
    
    // 初始化姿态
    memset(&current_attitude, 0, sizeof(attitude_data_t));
    current_attitude.confidence = 0.5f;
    
    // 设置状态
    initialized = 1;
    sprintf(debug_info, "Fusion initialized");
}

//// 设置参数
//void fusion_set_params(float gyro_w, float accel_w, float mag_w)
//{
//    if (gyro_w > 0 && gyro_w < 1.0f) gyro_weight = gyro_w;
//    if (accel_w > 0 && accel_w < 1.0f) accel_weight = accel_w;
//    if (mag_w > 0 && mag_w < 1.0f) mag_weight = mag_w;
//    
//    sprintf(debug_info, "Weights: G=%.3f A=%.3f M=%.3f", 
//            gyro_weight, accel_weight, mag_weight);
//}

// 1. 修改坐标系配置，交换或反转轴以匹配您的传感器方向
static void correct_from_accel(float acc[3])
{
    float acc_norm = vector_norm(acc);
    if (acc_norm < 0.1f) return;  // 加速度太小，可能不可靠
    
    // 归一化加速度
    float acc_normalized[3];
    // 坐标系校正 - 反转Y轴和Z轴方向以匹配板子方向
    acc_normalized[0] = acc[0] / acc_norm;
    acc_normalized[1] = -acc[1] / acc_norm;  // 反转Y轴
    acc_normalized[2] = acc[2] / acc_norm;
    
    // 从当前姿态计算重力方向
    float gravity_body[3];
    gravity_body[0] = 2.0f * (q[1] * q[3] - q[0] * q[2]);
    gravity_body[1] = 2.0f * (q[0] * q[1] + q[2] * q[3]);
    gravity_body[2] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
    
    // 计算误差（叉积）
    float error[3];
    vector_cross(acc_normalized, gravity_body, error);
    
    // 创建校正四元数
    float q_corr[4] = {1.0f, 0, 0, 0};
    
    // 基于误差和权重计算校正量
    // 降低校正权重，减少横滚抖动
    float correction_weight = accel_weight * 0.5f * calculate_acc_confidence(acc);
    q_corr[0] = 1.0f;
    q_corr[1] = error[0] * correction_weight;
    q_corr[2] = error[1] * correction_weight;
    q_corr[3] = error[2] * correction_weight;
    
    // 归一化校正四元数
    quaternion_normalize(q_corr);
    
    // 应用校正（四元数乘法）
    float q_temp[4];
    q_temp[0] = q[0] * q_corr[0] - q[1] * q_corr[1] - q[2] * q_corr[2] - q[3] * q_corr[3];
    q_temp[1] = q[0] * q_corr[1] + q[1] * q_corr[0] + q[2] * q_corr[3] - q[3] * q_corr[2];
    q_temp[2] = q[0] * q_corr[2] - q[1] * q_corr[3] + q[2] * q_corr[0] + q[3] * q_corr[1];
    q_temp[3] = q[0] * q_corr[3] + q[1] * q_corr[2] - q[2] * q_corr[1] + q[3] * q_corr[0];
    
    // 更新四元数
    memcpy(q, q_temp, sizeof(q));
    quaternion_normalize(q);
}

// 2. 修改磁力计处理，添加更强的干扰检测
static void correct_from_mag(float mag[3])
{
    if (!mag) return;
    
    float mag_norm = vector_norm(mag);
    if (mag_norm < 5.0f || mag_norm > 100.0f) {
        sprintf(debug_info, "Mag field strength abnormal: %.1f uT", mag_norm);
        return;  // 磁场太弱或太强，可能不可靠
    }
    
    // 检查磁场是否正常 - 计算与垂直面的夹角
    float normalized_mag[3] = {
        mag[0] / mag_norm,
        mag[1] / mag_norm, 
        mag[2] / mag_norm
    };
    
    // 计算磁场与重力的夹角，地球磁场应该基本上与重力垂直
    float g_dot_m = fabsf(normalized_mag[2]);  // 与Z轴的点积
    if (g_dot_m > 0.8f) {
        // 磁场与重力接近平行，可能有干扰
        sprintf(debug_info, "Mag interference detected: %.2f", g_dot_m);
        return;
    }
    
    // 将磁场向量转换到地理坐标系（考虑当前姿态）
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
    
    // 把磁场从机体坐标系转到地理坐标系
    float lx = normalized_mag[0] * (q0q0 + q1q1 - q2q2 - q3q3) + 
               2.0f * normalized_mag[1] * (q1q2 - q0q3) + 
               2.0f * normalized_mag[2] * (q1q3 + q0q2);
    float ly = 2.0f * normalized_mag[0] * (q1q2 + q0q3) + 
               normalized_mag[1] * (q0q0 - q1q1 + q2q2 - q3q3) + 
               2.0f * normalized_mag[2] * (q2q3 - q0q1);
    
    // 计算磁偏角
    float yaw_mag = atan2f(ly, lx);
    
    // 从当前姿态计算偏航角
    float roll, pitch, yaw;
    quaternion_to_euler(q, &roll, &pitch, &yaw);
    yaw *= DEG_TO_RAD;  // 转换为弧度
    
    // 计算偏航误差
    float yaw_error = yaw_mag - yaw;
    
    // 标准化到 -PI 到 PI
    if (yaw_error > PI) yaw_error -= 2.0f * PI;
    if (yaw_error < -PI) yaw_error += 2.0f * PI;
    
    // 减小磁力计影响，防止漂移
    float mag_weight_adjusted = mag_weight * 0.2f;
    
    // 检测突变 - 如果偏航误差太大，减少权重
    if (fabsf(yaw_error) > 0.3f) {  // ~17度
        mag_weight_adjusted *= 0.1f;  // 大大减少影响
        sprintf(debug_info, "Mag error large: %.1f deg", yaw_error * RAD_TO_DEG);
    } else {
        sprintf(debug_info, "Mag OK, Yaw err: %.1f deg", yaw_error * RAD_TO_DEG);
    }
    
    // 创建偏航校正四元数
    float q_yaw[4];
    float half_error = yaw_error * 0.5f * mag_weight_adjusted;
    q_yaw[0] = cosf(half_error);
    q_yaw[1] = 0;
    q_yaw[2] = 0;
    q_yaw[3] = sinf(half_error);
    
    // 应用校正（四元数乘法）
    float q_temp[4];
    q_temp[0] = q[0] * q_yaw[0] - q[1] * q_yaw[1] - q[2] * q_yaw[2] - q[3] * q_yaw[3];
    q_temp[1] = q[0] * q_yaw[1] + q[1] * q_yaw[0] + q[2] * q_yaw[3] - q[3] * q_yaw[2];
    q_temp[2] = q[0] * q_yaw[2] - q[1] * q_yaw[3] + q[2] * q_yaw[0] + q[3] * q_yaw[1];
    q_temp[3] = q[0] * q_yaw[3] + q[1] * q_yaw[2] - q[2] * q_yaw[1] + q[3] * q_yaw[0];
    
    // 更新四元数
    memcpy(q, q_temp, sizeof(q));
    quaternion_normalize(q);
}

// 3. 修改欧拉角计算，确保坐标系正确
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
    
    // 确保偏航角在0-360范围内
    if (*yaw < 0.0f) *yaw += 360.0f;
}

// 4. 修改融合参数设置函数，使用更安全的默认值
void fusion_set_params(float gyro_w, float accel_w, float mag_w)
{
    // 更安全的默认值：更依赖陀螺仪，减少加速度和磁力计的影响
    gyro_weight = 0.99f;  // 高度依赖陀螺仪
    accel_weight = 0.01f; // 很小的加速度影响
    mag_weight = 0.005f;  // 极小的磁力计影响
    
    // 只有当输入参数在合理范围内时才使用
    if (gyro_w >= 0.9f && gyro_w < 1.0f) gyro_weight = gyro_w;
    if (accel_w > 0 && accel_w < 0.1f) accel_weight = accel_w;
    if (mag_w > 0 && mag_w < 0.05f) mag_weight = mag_w;
    
    sprintf(debug_info, "Weights: G=%.3f A=%.3f M=%.3f", 
            gyro_weight, accel_weight, mag_weight);
}

// 设置陀螺仪偏差
void fusion_set_gyro_bias(float bias[3])
{
    if (bias) {
        memcpy(gyro_bias, bias, sizeof(float) * 3);
    }
}

// 启用/禁用磁力计
void fusion_use_magnetometer(int enable)
{
    use_mag = enable;
    sprintf(debug_info, "Magnetometer %s", enable ? "enabled" : "disabled");
}

// 获取调试信息
const char* fusion_get_debug_info(void)
{
    return debug_info;
}

// 更新姿态估计
void fusion_update(float acc[3], float gyro[3], float mag[3], float baro_alt, float dt)
{
    // 存储原始数据到姿态结构
    memcpy(current_attitude.accel, acc, sizeof(float) * 3);
    memcpy(current_attitude.gyro, gyro, sizeof(float) * 3);
    if (mag) memcpy(current_attitude.mag, mag, sizeof(float) * 3);
    
    // 0. 基于陀螺仪更新姿态
    update_from_gyro(gyro, dt);
    
    // 1. 基于加速度计校正Roll和Pitch
    float acc_confidence = calculate_acc_confidence(acc);
    if (acc_confidence > 0.1f) {
        correct_from_accel(acc);
    }
    
    // 2. 基于磁力计校正Yaw
    if (use_mag && mag && acc_confidence > 0.5f) {
        // 仅在加速度稳定时使用磁力计
        correct_from_mag(mag);
    }
    
    // 3. 更新气压高度
    if (baro_alt != 0.0f) {
        if (baro_alt_filtered == 0.0f) {
            baro_alt_filtered = baro_alt;
        } else {
            baro_alt_filtered = baro_alt_filtered * (1.0f - baro_weight) + baro_alt * baro_weight;
        }
        current_attitude.altitude = baro_alt_filtered;
    }
    
    // 4. 更新姿态结构体
    quaternion_to_euler(q, &current_attitude.roll, &current_attitude.pitch, &current_attitude.yaw);
    current_attitude.confidence = acc_confidence;
    
    // 5. 限制偏航角范围到0-360度
    if (current_attitude.yaw < 0) current_attitude.yaw += 360.0f;
    if (current_attitude.yaw >= 360.0f) current_attitude.yaw -= 360.0f;
}

// 获取当前姿态
attitude_data_t fusion_get_attitude(void)
{
    return current_attitude;
}

// 基于陀螺仪更新四元数
static void update_from_gyro(float gyro[3], float dt)
{
    // 去除偏差并转换为弧度/秒
    float wx = (gyro[0] - gyro_bias[0]) * DEG_TO_RAD;
    float wy = (gyro[1] - gyro_bias[1]) * DEG_TO_RAD;
    float wz = (gyro[2] - gyro_bias[2]) * DEG_TO_RAD;
    
    // 四元数微分方程
    float dq0 = 0.5f * (-q[1] * wx - q[2] * wy - q[3] * wz);
    float dq1 = 0.5f * (q[0] * wx + q[2] * wz - q[3] * wy);
    float dq2 = 0.5f * (q[0] * wy - q[1] * wz + q[3] * wx);
    float dq3 = 0.5f * (q[0] * wz + q[1] * wy - q[2] * wx);
    
    // 更新四元数
    q[0] += dq0 * dt;
    q[1] += dq1 * dt;
    q[2] += dq2 * dt;
    q[3] += dq3 * dt;
    
    // 归一化
    quaternion_normalize(q);
}

//// 基于加速度计校正
//static void correct_from_accel(float acc[3])
//{
//    float acc_norm = vector_norm(acc);
//    if (acc_norm < 0.1f) return;  // 加速度太小，可能不可靠
//    
//    // 归一化加速度
//    float acc_normalized[3];
//    acc_normalized[0] = acc[0] / acc_norm;
//    acc_normalized[1] = acc[1] / acc_norm;
//    acc_normalized[2] = acc[2] / acc_norm;
//    
//    // 从当前姿态计算重力方向
//    float gravity_body[3];
//    gravity_body[0] = 2.0f * (q[1] * q[3] - q[0] * q[2]);
//    gravity_body[1] = 2.0f * (q[0] * q[1] + q[2] * q[3]);
//    gravity_body[2] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
//    
//    // 计算误差（叉积）
//    float error[3];
//    vector_cross(acc_normalized, gravity_body, error);
//    
//    // 创建校正四元数
//    float q_corr[4] = {1.0f, 0, 0, 0};
//    
//    // 基于误差和权重计算校正量
//    float correction_weight = accel_weight * calculate_acc_confidence(acc);
//    q_corr[0] = 1.0f;
//    q_corr[1] = error[0] * correction_weight;
//    q_corr[2] = error[1] * correction_weight;
//    q_corr[3] = error[2] * correction_weight;
//    
//    // 归一化校正四元数
//    quaternion_normalize(q_corr);
//    
//    // 应用校正（四元数乘法）
//    float q_temp[4];
//    q_temp[0] = q[0] * q_corr[0] - q[1] * q_corr[1] - q[2] * q_corr[2] - q[3] * q_corr[3];
//    q_temp[1] = q[0] * q_corr[1] + q[1] * q_corr[0] + q[2] * q_corr[3] - q[3] * q_corr[2];
//    q_temp[2] = q[0] * q_corr[2] - q[1] * q_corr[3] + q[2] * q_corr[0] + q[3] * q_corr[1];
//    q_temp[3] = q[0] * q_corr[3] + q[1] * q_corr[2] - q[2] * q_corr[1] + q[3] * q_corr[0];
//    
//    // 更新四元数
//    memcpy(q, q_temp, sizeof(q));
//    quaternion_normalize(q);
//}

//// 基于磁力计校正
//static void correct_from_mag(float mag[3])
//{
//    float mag_norm = vector_norm(mag);
//    if (mag_norm < 0.1f) return;  // 磁场太弱
//    
//    // 归一化磁场
//    float mag_normalized[3];
//    mag_normalized[0] = mag[0] / mag_norm;
//    mag_normalized[1] = mag[1] / mag_norm;
//    mag_normalized[2] = mag[2] / mag_norm;
//    
//    // 计算当前姿态下的地理坐标系
//    float q0q0 = q[0] * q[0];
//    float q0q1 = q[0] * q[1];
//    float q0q2 = q[0] * q[2];
//    float q0q3 = q[0] * q[3];
//    float q1q1 = q[1] * q[1];
//    float q1q2 = q[1] * q[2];
//    float q1q3 = q[1] * q[3];
//    float q2q2 = q[2] * q[2];
//    float q2q3 = q[2] * q[3];
//    float q3q3 = q[3] * q[3];
//    
//    // 将磁场向量转换到地理坐标系
//    float lx = mag_normalized[0] * (q0q0 + q1q1 - q2q2 - q3q3) + 
//               2.0f * mag_normalized[1] * (q1q2 - q0q3) + 
//               2.0f * mag_normalized[2] * (q1q3 + q0q2);
//    float ly = 2.0f * mag_normalized[0] * (q1q2 + q0q3) + 
//               mag_normalized[1] * (q0q0 - q1q1 + q2q2 - q3q3) + 
//               2.0f * mag_normalized[2] * (q2q3 - q0q1);
//    
//    // 计算磁偏角
//    float yaw_mag = atan2f(ly, lx);
//    
//    // 从当前姿态计算偏航角
//    float roll, pitch, yaw;
//    quaternion_to_euler(q, &roll, &pitch, &yaw);
//    yaw *= DEG_TO_RAD;  // 转换为弧度
//    
//    // 计算偏航误差
//    float yaw_error = yaw_mag - yaw;
//    
//    // 标准化到 -PI 到 PI
//    if (yaw_error > PI) yaw_error -= 2.0f * PI;
//    if (yaw_error < -PI) yaw_error += 2.0f * PI;
//    
//    // 创建偏航校正四元数
//    float q_yaw[4];
//    float half_error = yaw_error * 0.5f * mag_weight;
//    q_yaw[0] = cosf(half_error);
//    q_yaw[1] = 0;
//    q_yaw[2] = 0;
//    q_yaw[3] = sinf(half_error);
//    
//    // 应用校正（四元数乘法）
//    float q_temp[4];
//    q_temp[0] = q[0] * q_yaw[0] - q[1] * q_yaw[1] - q[2] * q_yaw[2] - q[3] * q_yaw[3];
//    q_temp[1] = q[0] * q_yaw[1] + q[1] * q_yaw[0] + q[2] * q_yaw[3] - q[3] * q_yaw[2];
//    q_temp[2] = q[0] * q_yaw[2] - q[1] * q_yaw[3] + q[2] * q_yaw[0] + q[3] * q_yaw[1];
//    q_temp[3] = q[0] * q_yaw[3] + q[1] * q_yaw[2] - q[2] * q_yaw[1] + q[3] * q_yaw[0];
//    
//    // 更新四元数
//    memcpy(q, q_temp, sizeof(q));
//    quaternion_normalize(q);
//}

// 四元数归一化
static void quaternion_normalize(float q[4])
{
    float norm = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (norm > 0.0f) {
        float inv_norm = 1.0f / norm;
        q[0] *= inv_norm;
        q[1] *= inv_norm;
        q[2] *= inv_norm;
        q[3] *= inv_norm;
    }
}

//// 四元数转欧拉角
//static void quaternion_to_euler(const float q[4], float* roll, float* pitch, float* yaw)
//{
//    // Roll (x-axis rotation)
//    float sinr_cosp = 2.0f * (q[0] * q[1] + q[2] * q[3]);
//    float cosr_cosp = 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]);
//    *roll = atan2f(sinr_cosp, cosr_cosp) * RAD_TO_DEG;

//    // Pitch (y-axis rotation)
//    float sinp = 2.0f * (q[0] * q[2] - q[3] * q[1]);
//    if (fabsf(sinp) >= 1.0f)
//        *pitch = copysignf(90.0f, sinp); // 使用90度或-90度
//    else
//        *pitch = asinf(sinp) * RAD_TO_DEG;

//    // Yaw (z-axis rotation)
//    float siny_cosp = 2.0f * (q[0] * q[3] + q[1] * q[2]);
//    float cosy_cosp = 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]);
//    *yaw = atan2f(siny_cosp, cosy_cosp) * RAD_TO_DEG;
//}

// 计算加速度计测量的可信度
static float calculate_acc_confidence(float acc[3])
{
    // 计算加速度大小
    float acc_magnitude = vector_norm(acc);
    
    // 与标准重力(9.81m/s²)比较
    float gravity_error = fabsf(acc_magnitude - 9.81f) / 9.81f;
    
    // 转换为0-1的置信度
    float confidence = 1.0f - fminf(gravity_error, 1.0f);
    
    // 当加速度约等于重力时，置信度高
    // 当有额外加速度时，置信度降低
    return confidence * confidence;  // 平方增强低置信度的惩罚
}

// 向量模长
static float vector_norm(const float v[3])
{
    return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

// 向量归一化
static void vector_normalize(float v[3])
{
    float norm = vector_norm(v);
    if (norm > 0.0f) {
        float inv_norm = 1.0f / norm;
        v[0] *= inv_norm;
        v[1] *= inv_norm;
        v[2] *= inv_norm;
    }
}

// 向量点积
static float vector_dot(const float a[3], const float b[3])
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

// 向量叉积
static void vector_cross(const float a[3], const float b[3], float result[3])
{
    result[0] = a[1] * b[2] - a[2] * b[1];
    result[1] = a[2] * b[0] - a[0] * b[2];
    result[2] = a[0] * b[1] - a[1] * b[0];
}