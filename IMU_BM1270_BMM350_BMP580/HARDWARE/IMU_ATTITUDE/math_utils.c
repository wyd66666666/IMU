/**
 * @file math_utils.c
 * @brief 数学工具函数实现
 * @author Claude
 * @date 2025-08-03
 */

#include "enhanced_attitude.h"
#include <math.h>
#include <float.h>

/**
 * @brief 向量归一化
 * @param v 输入输出向量
 */
void Vector3_Normalize(vector3_t *v)
{
    if (!v) return;
    
    float length = sqrtf(v->x * v->x + v->y * v->y + v->z * v->z);
    if (length > FLT_EPSILON) {
        float inv_length = 1.0f / length;
        v->x *= inv_length;
        v->y *= inv_length;
        v->z *= inv_length;
    }
}

/**
 * @brief 计算向量长度
 * @param v 输入向量
 * @return 向量长度
 */
float Vector3_Length(const vector3_t *v)
{
    if (!v) return 0.0f;
    return sqrtf(v->x * v->x + v->y * v->y + v->z * v->z);
}

/**
 * @brief 向量点积
 * @param a 向量a
 * @param b 向量b
 * @return 点积结果
 */
float Vector3_Dot(const vector3_t *a, const vector3_t *b)
{
    if (!a || !b) return 0.0f;
    return a->x * b->x + a->y * b->y + a->z * b->z;
}

/**
 * @brief 向量叉积
 * @param a 向量a
 * @param b 向量b
 * @param result 结果向量
 */
void Vector3_Cross(const vector3_t *a, const vector3_t *b, vector3_t *result)
{
    if (!a || !b || !result) return;
    
    result->x = a->y * b->z - a->z * b->y;
    result->y = a->z * b->x - a->x * b->z;
    result->z = a->x * b->y - a->y * b->x;
}

/**
 * @brief 向量缩放
 * @param v 输入向量
 * @param scale 缩放因子
 * @param result 结果向量
 */
void Vector3_Scale(const vector3_t *v, float scale, vector3_t *result)
{
    if (!v || !result) return;
    
    result->x = v->x * scale;
    result->y = v->y * scale;
    result->z = v->z * scale;
}

/**
 * @brief 四元数归一化
 * @param q 输入输出四元数
 */
void Quaternion_Normalize(quaternion_t *q)
{
    if (!q) return;
    
    float norm = sqrtf(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
    if (norm > FLT_EPSILON) {
        float inv_norm = 1.0f / norm;
        q->w *= inv_norm;
        q->x *= inv_norm;
        q->y *= inv_norm;
        q->z *= inv_norm;
    } else {
        // 如果四元数接近零，重置为单位四元数
        q->w = 1.0f;
        q->x = q->y = q->z = 0.0f;
    }
}

/**
 * @brief 四元数转欧拉角
 * @param q 输入四元数
 * @param euler 输出欧拉角 (rad)
 */
void Quaternion_ToEuler(const quaternion_t *q, euler_t *euler)
{
    if (!q || !euler) return;
    
    // 四元数转欧拉角 (ZYX顺序, 内旋)
    float w = q->w, x = q->x, y = q->y, z = q->z;
    
    // Roll (x轴旋转)
    float sinr_cosp = 2.0f * (w * x + y * z);
    float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
    euler->roll = atan2f(sinr_cosp, cosr_cosp);
    
    // Pitch (y轴旋转)
    float sinp = 2.0f * (w * y - z * x);
    if (fabsf(sinp) >= 1.0f) {
        euler->pitch = copysignf(PI_F / 2.0f, sinp); // 万向锁情况
    } else {
        euler->pitch = asinf(sinp);
    }
    
    // Yaw (z轴旋转)
    float siny_cosp = 2.0f * (w * z + x * y);
    float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
    euler->yaw = atan2f(siny_cosp, cosy_cosp);
}

/**
 * @brief 欧拉角转四元数
 * @param euler 输入欧拉角 (rad)
 * @param q 输出四元数
 */
void Quaternion_FromEuler(const euler_t *euler, quaternion_t *q)
{
    if (!euler || !q) return;
    
    float roll = euler->roll * 0.5f;
    float pitch = euler->pitch * 0.5f;
    float yaw = euler->yaw * 0.5f;
    
    float cr = cosf(roll);
    float sr = sinf(roll);
    float cp = cosf(pitch);
    float sp = sinf(pitch);
    float cy = cosf(yaw);
    float sy = sinf(yaw);
    
    q->w = cr * cp * cy + sr * sp * sy;
    q->x = sr * cp * cy - cr * sp * sy;
    q->y = cr * sp * cy + sr * cp * sy;
    q->z = cr * cp * sy - sr * sp * cy;
}

/**
 * @brief 四元数乘法
 * @param a 四元数a
 * @param b 四元数b
 * @param result 结果四元数
 */
void Quaternion_Multiply(const quaternion_t *a, const quaternion_t *b, quaternion_t *result)
{
    if (!a || !b || !result) return;
    
    result->w = a->w * b->w - a->x * b->x - a->y * b->y - a->z * b->z;
    result->x = a->w * b->x + a->x * b->w + a->y * b->z - a->z * b->y;
    result->y = a->w * b->y - a->x * b->z + a->y * b->w + a->z * b->x;
    result->z = a->w * b->z + a->x * b->y - a->y * b->x + a->z * b->w;
}