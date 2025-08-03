#include "imu_attitude.h"
#include <math.h>

// 定义PI
#define PI 3.14159265358979f

// Mahony算法参数
#define TWO_KP 2.0f     // 比例增益
#define TWO_KI 0.005f   // 积分增益

// 四元数
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

// 积分误差
static float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;

// 当前姿态角
static attitude_t current_attitude = {0.0f, 0.0f, 0.0f};

// 初始化
void imu_attitude_init(void)
{
    q0 = 1.0f;
    q1 = q2 = q3 = 0.0f;
    integralFBx = integralFBy = integralFBz = 0.0f;
    current_attitude.roll = current_attitude.pitch = current_attitude.yaw = 0.0f;
}

// 使用Mahony算法更新姿态
void imu_attitude_update(float ax, float ay, float az, 
                         float gx, float gy, float gz,
                         float mx, float my, float mz,
                         float dt)
{
    float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float hx, hy, bx, bz;
    float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // 单位化加速度
    recipNorm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // 单位化磁力计数据
    recipNorm = 1.0f / sqrtf(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    // 辅助变量，减少重复计算
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;

    // 地球磁场的参考方向
    hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
    hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
    
    // bx, bz包含了地球磁场的参考方向
    bx = sqrtf(hx * hx + hy * hy);
    bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

    // 根据估计方向计算重力和磁场的方向误差
    halfvx = q1q3 - q0q2;
    halfvy = q0q1 + q2q3;
    halfvz = q0q0 - 0.5f + q3q3;
    
    halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
    halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
    halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);
    
    // 计算方向误差
    halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
    halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
    halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

    // 积分误差比例于误差向量
    integralFBx += TWO_KI * halfex * dt;
    integralFBy += TWO_KI * halfey * dt;
    integralFBz += TWO_KI * halfez * dt;
    
    // 应用积分反馈
    gx += integralFBx;
    gy += integralFBy;
    gz += integralFBz;
    
    // 应用比例反馈
    gx += TWO_KP * halfex;
    gy += TWO_KP * halfey;
    gz += TWO_KP * halfez;
    
    // 四元数变化率正比于旋转的一半
    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;
    
    // 更新四元数
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);
    
    // 单位化四元数
    recipNorm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
    
    // 计算欧拉角 (x-y-z, 内旋)
    current_attitude.roll = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 180.0f / PI;
    current_attitude.pitch = asinf(2.0f * (q0 * q2 - q3 * q1)) * 180.0f / PI;
    current_attitude.yaw = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * 180.0f / PI;
}

// 获取当前姿态角
attitude_t imu_attitude_get(void)
{
    return current_attitude;
}

