#include "imu_attitude.h"
#include <math.h>
#include <string.h> // For memset

// --- 宏定义与全局变量 ---
#ifndef PI
#define PI 3.14159265358979323846f
#endif

#define DEG_TO_RAD (PI / 180.0f)
#define RAD_TO_DEG (180.0f / PI)

// Mahony算法参数
static float Kp = 1.0f;
static float Ki = 0.05f;

// 姿态四元数
static volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

// 陀螺仪积分误差项
static volatile float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;

// 当前姿态角
static attitude_t current_attitude = {0};

// --- 函数实现 ---

/**
 * @brief 初始化或重置姿态解算器
 */
void imu_attitude_init(void)
{
    q0 = 1.0f;
    q1 = q2 = q3 = 0.0f;
    integralFBx = integralFBy = integralFBz = 0.0f;
    memset(&current_attitude, 0, sizeof(attitude_t));
}

/**
 * @brief 设置PI控制器增益
 * @param kp 比例增益
 * @param ki 积分增益
 */
void imu_attitude_set_gains(float kp, float ki)
{
    Kp = kp;
    Ki = ki;
}

// 内部函数：将四元数转换为欧拉角并处理万向节锁
static void update_euler_from_quaternion(void)
{
    // Roll (x-axis rotation)
    float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
    float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
    current_attitude.roll = atan2f(sinr_cosp, cosr_cosp) * RAD_TO_DEG;

    // Pitch (y-axis rotation)
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    // 处理万向节锁 (Gimbal Lock)
    if (fabsf(sinp) >= 1.0f) {
        current_attitude.pitch = copysignf(90.0f, sinp); // 使用90度或-90度
    } else {
        current_attitude.pitch = asinf(sinp) * RAD_TO_DEG;
    }

    // Yaw (z-axis rotation)
    float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
    float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    current_attitude.yaw = atan2f(siny_cosp, cosy_cosp) * RAD_TO_DEG;
}

/**
 * @brief 使用Mahony算法更新姿态 (9轴)
 * @param ax, ay, az 加速度计数据 (单位: m/s^2 或 g)
 * @param gx, gy, gz 陀螺仪数据 (单位: rad/s)
 * @param mx, my, mz 磁力计数据 (经过校准的)
 * @param dt 时间间隔 (单位: s)
 */
void imu_attitude_update(float ax, float ay, float az,
                         float gx, float gy, float gz,
                         float mx, float my, float mz,
                         float dt)
{
    float norm;
    float hx, hy, bx, bz;
    float vx, vy, vz, wx, wy, wz;
    float ex, ey, ez;
    float qa, qb, qc;

    // --- 1. 单位化加速度计和磁力计矢量 ---
    // G431的FPU性能强劲，直接使用标准sqrtf()，精度更高
    norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm < 1e-6f) return; // 加速度计数据无效，直接返回，避免除零
    ax /= norm;
    ay /= norm;
    az /= norm;

    norm = sqrtf(mx * mx + my * my + mz * mz);
    if (norm < 1e-6f) return; // 磁力计数据无效，直接返回
    mx /= norm;
    my /= norm;
    mz /= norm;

    // --- 2. 将地理坐标系下的参考矢量（重力和磁场）转换到机体坐标系下 ---
    // 通过将机体坐标系(b)下的四元数逆运算得到导航坐标系(n)下的向量
    // 参考磁场矢量在导航坐标系(n)中的表示
    hx = 2.0f * mx * (0.5f - q2 * q2 - q3 * q3) + 2.0f * my * (q1 * q2 - q0 * q3) + 2.0f * mz * (q1 * q3 + q0 * q2);
    hy = 2.0f * mx * (q1 * q2 + q0 * q3) + 2.0f * my * (0.5f - q1 * q1 - q3 * q3) + 2.0f * mz * (q2 * q3 - q0 * q1);
    
    // 水平磁场分量(bx)和垂直磁场分量(bz)
    bx = sqrtf(hx * hx + hy * hy);
    bz = 2.0f * mx * (q1 * q3 - q0 * q2) + 2.0f * my * (q2 * q3 + q0 * q1) + 2.0f * mz * (0.5f - q1 * q1 - q2 * q2);

    // 根据当前姿态估计的重力矢量和磁场矢量在机体坐标系(b)下的表示
    vx = 2.0f * (q1 * q3 - q0 * q2);
    vy = 2.0f * (q0 * q1 + q2 * q3);
    vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    wx = 2.0f * bx * (0.5f - q2 * q2 - q3 * q3) + 2.0f * bz * (q1 * q3 - q0 * q2);
    wy = 2.0f * bx * (q1 * q2 - q0 * q3) + 2.0f * bz * (q0 * q1 + q2 * q3);
    wz = 2.0f * bx * (q0 * q2 + q1 * q3) + 2.0f * bz * (0.5f - q1 * q1 - q2 * q2);

    // --- 3. 计算误差 ---
    // 误差是传感器测量值（ax, ay, az, mx, my, mz）与姿态估计值（vx, vy, vz, wx, wy, wz）的叉积
    ex = (ay * vz - az * vy) + (my * wz - mz * wy);
    ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
    ez = (ax * vy - ay * vx) + (mx * wy - my * wx);

    // --- 4. PI控制器校正 ---
    // 积分项补偿陀螺仪零偏
    if (Ki > 0.0f) {
        integralFBx += Ki * ex * dt;
        integralFBy += Ki * ey * dt;
        integralFBz += Ki * ez * dt;
    } else {
        integralFBx = 0.0f;
        integralFBy = 0.0f;
        integralFBz = 0.0f;
    }
    
    // 校正陀螺仪测量值
    gx += Kp * ex + integralFBx;
    gy += Kp * ey + integralFBy;
    gz += Kp * ez + integralFBz;

    // --- 5. 积分陀螺仪速率，更新四元数 ---
    // 注意：这里的gx, gy, gz单位必须是 rad/s
    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;

    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // --- 6. 单位化四元数，防止计算误差累积 ---
    norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 /= norm;
    q1 /= norm;
    q2 /= norm;
    q3 /= norm;

    // --- 7. 更新欧拉角输出 ---
    update_euler_from_quaternion();
}


/**
 * @brief 获取当前姿态角
 * @return attitude_t 结构体，包含roll, pitch, yaw (单位: 度)
 */
attitude_t imu_attitude_get(void)
{
    return current_attitude;
}


