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
static float gyro_bias_learned[3] = {0};       // 学习到的陀螺仪偏差

// 参数
static float beta = 0.1f;                      // Madgwick算法增益参数

// 配置
static int use_mag = 0;                        // 是否使用磁力计
static int initialized = 0;                    // 初始化标志
static char debug_info[128];                   // 调试信息

// 全局变量添加
static float prev_yaw = 0.0f;      // 上一次的偏航角
static float yaw_continuous = 0.0f; // 连续偏航角（不限范围）

// 输出
static attitude_data_t current_attitude = {0};

// 内部函数声明
static void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt);
static void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt);
static float invSqrt(float x);

// 初始化融合滤波器
void fusion_init(void)
{
    // 初始化四元数为单位四元数
    q[0] = 1.0f;
    q[1] = q[2] = q[3] = 0.0f;
    
    // 初始化其他变量
    memset(gyro_bias, 0, sizeof(gyro_bias));
    memset(gyro_bias_learned, 0, sizeof(gyro_bias_learned));
    
    // 初始化姿态
    memset(&current_attitude, 0, sizeof(attitude_data_t));
    current_attitude.confidence = 0.5f;
    
    // 设置状态
    initialized = 1;
    sprintf(debug_info, "Madgwick filter initialized");
}

// 设置参数
void fusion_set_params(float gyro_w, float accel_w, float mag_w)
{
    // 对于Madgwick算法，我们使用beta参数
    // beta值越大，加速度计和磁力计的权重越大
    // 典型值：0.033 ~ 0.1
    beta = 0.033f;  // 保守的值，减少抖动
    
    sprintf(debug_info, "Beta=%.3f", beta);
}

// 设置陀螺仪偏差
void fusion_set_gyro_bias(float bias[3])
{
    if (bias) {
        memcpy(gyro_bias, bias, sizeof(float) * 3);
        memcpy(gyro_bias_learned, bias, sizeof(float) * 3);
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
    static char full_debug[256];
    sprintf(full_debug, "%s, Bias[%.2f,%.2f,%.2f]", 
            debug_info, 
            gyro_bias_learned[0], 
            gyro_bias_learned[1], 
            gyro_bias_learned[2]);
    return full_debug;
}

// 更新姿态估计
void fusion_update(float acc[3], float gyro[3], float mag[3], float baro_alt, float dt)
{
    // 存储原始数据
    memcpy(current_attitude.accel, acc, sizeof(float) * 3);
    memcpy(current_attitude.gyro, gyro, sizeof(float) * 3);
    if (mag) memcpy(current_attitude.mag, mag, sizeof(float) * 3);
    
    // 应用陀螺仪偏差校正
    float gx = (gyro[0] - gyro_bias_learned[0]) * DEG_TO_RAD;
    float gy = (gyro[1] - gyro_bias_learned[1]) * DEG_TO_RAD;
    float gz = (gyro[2] - gyro_bias_learned[2]) * DEG_TO_RAD;
    
    // 动态学习陀螺仪偏差（仅在静止时）
    float gyro_magnitude = sqrtf(gyro[0]*gyro[0] + gyro[1]*gyro[1] + gyro[2]*gyro[2]);
    float acc_magnitude = sqrtf(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]);
    
    if (gyro_magnitude < 2.0f && fabsf(acc_magnitude - 9.81f) < 0.5f) {
        // 设备可能静止，缓慢更新偏差估计
        float alpha = 0.001f;  // 学习率
        gyro_bias_learned[0] = gyro_bias_learned[0] * (1.0f - alpha) + gyro[0] * alpha;
        gyro_bias_learned[1] = gyro_bias_learned[1] * (1.0f - alpha) + gyro[1] * alpha;
        gyro_bias_learned[2] = gyro_bias_learned[2] * (1.0f - alpha) + gyro[2] * alpha;
    }
    
    // 检测磁场干扰
    if (use_mag && mag) {
        float mag_norm = sqrtf(mag[0]*mag[0] + mag[1]*mag[1] + mag[2]*mag[2]);
        static float last_mag_norm = 0;
        static int first_mag = 1;
        
        if (first_mag) {
            last_mag_norm = mag_norm;
            first_mag = 0;
        }
        
        // 检测磁场强度变化，变化过大说明有干扰
        float mag_change = fabsf(mag_norm - last_mag_norm) / (last_mag_norm + 0.01f);
        if (mag_change > 0.1f) { // 10%变化认为有干扰
            // 有干扰时不使用磁力计
            MadgwickAHRSupdateIMU(gx, gy, gz, acc[0], acc[1], acc[2], dt);
            sprintf(debug_info, "Mag disturbance: %.1f%%", mag_change * 100.0f);
        } else {
            // 无干扰时正常使用，但使用较小的增益
            float old_beta = beta;
            beta *= 0.1f; // 减小磁力计影响
            MadgwickAHRSupdate(gx, gy, gz, acc[0], acc[1], acc[2], mag[0], mag[1], mag[2], dt);
            beta = old_beta; // 恢复原始beta值
        }
        last_mag_norm = mag_norm;
    } else {
        // 不使用磁力计
        MadgwickAHRSupdateIMU(gx, gy, gz, acc[0], acc[1], acc[2], dt);
    }
    
    // 更新欧拉角
    float sinr_cosp = 2.0f * (q[0] * q[1] + q[2] * q[3]);
    float cosr_cosp = 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]);
    current_attitude.roll = atan2f(sinr_cosp, cosr_cosp) * RAD_TO_DEG;

    float sinp = 2.0f * (q[0] * q[2] - q[3] * q[1]);
    if (fabsf(sinp) >= 1.0f)
        current_attitude.pitch = copysignf(90.0f, sinp);
    else
        current_attitude.pitch = asinf(sinp) * RAD_TO_DEG;

    float siny_cosp = 2.0f * (q[0] * q[3] + q[1] * q[2]);
    float cosy_cosp = 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]);
    
    // 计算当前原始偏航角
    float current_raw_yaw = atan2f(siny_cosp, cosy_cosp) * RAD_TO_DEG;
    
    // 处理偏航角连续性
    static int yaw_initialized = 0;
    if (!yaw_initialized) {
        prev_yaw = current_raw_yaw;
        yaw_continuous = current_raw_yaw;
        yaw_initialized = 1;
    } else {
        // 计算角度差（处理边界跨越）
        float yaw_diff = current_raw_yaw - prev_yaw;
        
        // 处理-180/+180边界跨越
        if (yaw_diff > 180.0f) yaw_diff -= 360.0f;
        if (yaw_diff < -180.0f) yaw_diff += 360.0f;
        
        // 平滑处理大的变化
        if (fabsf(yaw_diff) > 20.0f) {
            yaw_diff = yaw_diff * 0.2f; // 减缓大的变化
        }
        
        // 更新连续偏航角
        yaw_continuous += yaw_diff;
        prev_yaw = current_raw_yaw;
    }
    
    // 使用连续偏航角，不做范围限制
    current_attitude.yaw = yaw_continuous;
    
    // 更新高度
    current_attitude.altitude = baro_alt;
    
    // 计算置信度
    float gravity_error = fabsf(acc_magnitude - 9.81f) / 9.81f;
    current_attitude.confidence = 1.0f - fminf(gravity_error, 1.0f);
    
    // 更新调试信息，显示连续偏航角和映射到0-360范围的偏航角
    char yaw_info[50];
    float mapped_yaw = fmodf(yaw_continuous + 3600.0f, 360.0f); // 确保为正数后取模
    sprintf(yaw_info, "Yaw: %.1f (%.1f)", yaw_continuous, mapped_yaw);
    
    // 如果调试信息中没有特殊信息，则显示偏航角信息
    if (strstr(debug_info, "disturbance") == NULL) {
        strcpy(debug_info, yaw_info);
    }
}

// 获取当前姿态
attitude_data_t fusion_get_attitude(void)
{
    return current_attitude;
}

// Madgwick算法实现（6轴版本）
static void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // 如果加速度计数据无效，仅使用陀螺仪
    if((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)) {
        // 仅陀螺仪积分
        qDot1 = 0.5f * (-q[1] * gx - q[2] * gy - q[3] * gz);
        qDot2 = 0.5f * (q[0] * gx + q[2] * gz - q[3] * gy);
        qDot3 = 0.5f * (q[0] * gy - q[1] * gz + q[3] * gx);
        qDot4 = 0.5f * (q[0] * gz + q[1] * gy - q[2] * gx);
        
        q[0] += qDot1 * dt;
        q[1] += qDot2 * dt;
        q[2] += qDot3 * dt;
        q[3] += qDot4 * dt;
        
        // 归一化四元数
        recipNorm = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        q[0] *= recipNorm;
        q[1] *= recipNorm;
        q[2] *= recipNorm;
        q[3] *= recipNorm;
        return;
    }

    // 四元数变化率（陀螺仪测量）
    qDot1 = 0.5f * (-q[1] * gx - q[2] * gy - q[3] * gz);
    qDot2 = 0.5f * (q[0] * gx + q[2] * gz - q[3] * gy);
    qDot3 = 0.5f * (q[0] * gy - q[1] * gz + q[3] * gx);
    qDot4 = 0.5f * (q[0] * gz + q[1] * gy - q[2] * gx);

    // 归一化加速度计测量
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // 辅助变量，避免重复计算
    _2q0 = 2.0f * q[0];
    _2q1 = 2.0f * q[1];
    _2q2 = 2.0f * q[2];
    _2q3 = 2.0f * q[3];
    _4q0 = 4.0f * q[0];
    _4q1 = 4.0f * q[1];
    _4q2 = 4.0f * q[2];
    _8q1 = 8.0f * q[1];
    _8q2 = 8.0f * q[2];
    q0q0 = q[0] * q[0];
    q1q1 = q[1] * q[1];
    q2q2 = q[2] * q[2];
    q3q3 = q[3] * q[3];

    // 梯度下降算法校正步骤
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q[1] - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q[2] + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q[3] - _2q1 * ax + 4.0f * q2q2 * q[3] - _2q2 * ay;
    
    // 归一化步长
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // 应用反馈步骤
    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;

    // 积分以得到四元数
    q[0] += qDot1 * dt;
    q[1] += qDot2 * dt;
    q[2] += qDot3 * dt;
    q[3] += qDot4 * dt;

    // 归一化四元数
    recipNorm = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    q[0] *= recipNorm;
    q[1] *= recipNorm;
    q[2] *= recipNorm;
    q[3] *= recipNorm;
}

// Madgwick算法实现（9轴版本）
static void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt)
{
    // 简化版本：如果磁力计数据无效，使用6轴版本
        if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
        MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az, dt);
        return;
    }

    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

    // 如果加速度计或磁力计数据无效，仅使用陀螺仪
    if((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)) {
        MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az, dt);
        return;
    }

    // 四元数变化率（陀螺仪测量）
    qDot1 = 0.5f * (-q[1] * gx - q[2] * gy - q[3] * gz);
    qDot2 = 0.5f * (q[0] * gx + q[2] * gz - q[3] * gy);
    qDot3 = 0.5f * (q[0] * gy - q[1] * gz + q[3] * gx);
    qDot4 = 0.5f * (q[0] * gz + q[1] * gy - q[2] * gx);

    // 归一化加速度计测量
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // 归一化磁力计测量
    recipNorm = invSqrt(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    // 辅助变量，避免重复计算
    _2q0mx = 2.0f * q[0] * mx;
    _2q0my = 2.0f * q[0] * my;
    _2q0mz = 2.0f * q[0] * mz;
    _2q1mx = 2.0f * q[1] * mx;
    _2q0 = 2.0f * q[0];
    _2q1 = 2.0f * q[1];
    _2q2 = 2.0f * q[2];
    _2q3 = 2.0f * q[3];
    _2q0q2 = 2.0f * q[0] * q[2];
    _2q2q3 = 2.0f * q[2] * q[3];
    q0q0 = q[0] * q[0];
    q0q1 = q[0] * q[1];
    q0q2 = q[0] * q[2];
    q0q3 = q[0] * q[3];
    q1q1 = q[1] * q[1];
    q1q2 = q[1] * q[2];
    q1q3 = q[1] * q[3];
    q2q2 = q[2] * q[2];
    q2q3 = q[2] * q[3];
    q3q3 = q[3] * q[3];

    // 参考磁场方向
    hx = mx * q0q0 - _2q0my * q[3] + _2q0mz * q[2] + mx * q1q1 + _2q1 * my * q[2] + _2q1 * mz * q[3] - mx * q2q2 - mx * q3q3;
    hy = _2q0mx * q[3] + my * q0q0 - _2q0mz * q[1] + _2q1mx * q[2] - my * q1q1 + my * q2q2 + _2q2 * mz * q[3] - my * q3q3;
    _2bx = sqrtf(hx * hx + hy * hy);
    _2bz = -_2q0mx * q[2] + _2q0my * q[1] + mz * q0q0 + _2q1mx * q[3] - mz * q1q1 + _2q2 * my * q[3] - mz * q2q2 + mz * q3q3;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // 梯度下降算法校正步骤
    s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q[2] * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q[3] + _2bz * q[1]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q[2] * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q[1] * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q[3] * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q[2] + _2bz * q[0]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q[3] - _4bz * q[1]) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q[2] * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q[2] - _2bz * q[0]) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q[1] + _2bz * q[3]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q[0] - _4bz * q[2]) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q[3] + _2bz * q[1]) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q[0] + _2bz * q[2]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q[1] * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    
    // 归一化步长
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // 应用反馈步骤（使用更小的beta值以减少磁力计影响）
    float beta_mag = beta * 0.1f;  // 磁力计使用更小的增益
    qDot1 -= beta_mag * s0;
    qDot2 -= beta_mag * s1;
    qDot3 -= beta_mag * s2;
    qDot4 -= beta_mag * s3;

    // 积分以得到四元数
    q[0] += qDot1 * dt;
    q[1] += qDot2 * dt;
    q[2] += qDot3 * dt;
    q[3] += qDot4 * dt;

    // 归一化四元数
    recipNorm = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    q[0] *= recipNorm;
    q[1] *= recipNorm;
    q[2] *= recipNorm;
    q[3] *= recipNorm;
}

// 快速反平方根算法
static float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    y = y * (1.5f - (halfx * y * y));
    return y;
}


// 获取原始偏航角（0-360范围）
float fusion_get_raw_yaw(void)
{
    float raw_yaw = prev_yaw;
    // 确保范围在0-360
    while (raw_yaw < 0.0f) raw_yaw += 360.0f;
    while (raw_yaw >= 360.0f) raw_yaw -= 360.0f;
    return raw_yaw;
}

// 获取连续偏航角（不限范围）
float fusion_get_continuous_yaw(void)
{
    return yaw_continuous;
}
