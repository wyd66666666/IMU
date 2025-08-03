/**
 * @file enhanced_attitude.c
 * @brief 增强型姿态解算算法实现
 * @author Claude
 * @date 2025-08-03
 */

#include "enhanced_attitude.h"
#include <string.h>
#include <stdlib.h>

// 静态函数声明
static void update_adaptive_gains(enhanced_attitude_t *att, 
                                 const vector3_t *accel, 
                                 const vector3_t *mag);
static void detect_magnetic_interference(enhanced_attitude_t *att, const vector3_t *mag);
static void estimate_gyro_bias(enhanced_attitude_t *att, const vector3_t *error);
static void initialize_reference_vectors(enhanced_attitude_t *att, 
                                        const vector3_t *accel, 
                                        const vector3_t *mag);

/**
 * @brief 初始化增强型姿态解算器
 * @param att 姿态解算器实例
 */
void Enhanced_Attitude_Init(enhanced_attitude_t *att)
{
    memset(att, 0, sizeof(enhanced_attitude_t));
    
    // 初始化四元数为单位四元数
    att->q.w = 1.0f;
    att->q.x = att->q.y = att->q.z = 0.0f;
    
    // 初始化增益
    att->kp = 2.0f;
    att->ki = 0.01f;
    
    // 初始化软磁矩阵为单位矩阵
    for(int i = 0; i < 3; i++) {
        att->mag_cal.scale[i] = 1.0f;
        for(int j = 0; j < 3; j++) {
            att->mag_cal.soft_iron[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
    
    // 初始化置信度
    att->accel_confidence = 1.0f;
    att->mag_confidence = 1.0f;
    att->fusion_quality = 0.0f;
    
    att->is_initialized = false;
}

/**
 * @brief 更新姿态解算
 * @param att 姿态解算器实例
 * @param accel 加速度计数据 (m/s²)
 * @param gyro 陀螺仪数据 (rad/s)
 * @param mag 磁力计数据 (任意单位)
 * @param dt 时间步长 (s)
 */
void Enhanced_Attitude_Update(enhanced_attitude_t *att, 
                             const vector3_t *accel, 
                             const vector3_t *gyro, 
                             const vector3_t *mag, 
                             float dt)
{
    if (!att || !accel || !gyro || !mag || dt <= 0) return;
    
    att->dt = dt;
    att->update_count++;
    
    // 应用校准
    vector3_t accel_cal = *accel;
    vector3_t gyro_cal, mag_cal;
    
    Enhanced_Attitude_ApplyGyroCalibration(&att->gyro_cal, gyro, &gyro_cal);
    Enhanced_Attitude_ApplyMagCalibration(&att->mag_cal, mag, &mag_cal);
    
    // 初始化参考向量
    if (!att->is_initialized) {
        initialize_reference_vectors(att, &accel_cal, &mag_cal);
        att->is_initialized = true;
        return;
    }
    
    // 归一化传感器数据
    Vector3_Normalize(&accel_cal);
    Vector3_Normalize(&mag_cal);
    
    // 检测磁干扰
    detect_magnetic_interference(att, &mag_cal);
    
    // 更新自适应增益
    update_adaptive_gains(att, &accel_cal, &mag_cal);
    
    // 从四元数计算预测的重力和磁场方向
    vector3_t gravity_pred, mag_pred;
    
    // 预测重力方向 (0, 0, -1) 在机体坐标系中的表示
    gravity_pred.x = 2.0f * (att->q.x * att->q.z - att->q.w * att->q.y);
    gravity_pred.y = 2.0f * (att->q.y * att->q.z + att->q.w * att->q.x);
    gravity_pred.z = att->q.w * att->q.w - att->q.x * att->q.x - att->q.y * att->q.y + att->q.z * att->q.z;
    
    // 计算加速度误差
    vector3_t accel_error;
    Vector3_Cross(&accel_cal, &gravity_pred, &accel_error);
    
    // 计算磁场误差（仅在磁场可信时使用）
    vector3_t mag_error = {0, 0, 0};
    if (att->mag_confidence > 0.3f) {
        // 将磁场投影到水平面
        vector3_t mag_horizontal = mag_cal;
        float mag_down = Vector3_Dot(&mag_horizontal, &gravity_pred);
        mag_horizontal.x -= gravity_pred.x * mag_down;
        mag_horizontal.y -= gravity_pred.y * mag_down;
        mag_horizontal.z -= gravity_pred.z * mag_down;
        Vector3_Normalize(&mag_horizontal);
        
        // 预测水平磁场方向
        vector3_t mag_ref_horizontal = att->mag_ref;
        float ref_down = Vector3_Dot(&mag_ref_horizontal, &gravity_pred);
        mag_ref_horizontal.x -= gravity_pred.x * ref_down;
        mag_ref_horizontal.y -= gravity_pred.y * ref_down;
        mag_ref_horizontal.z -= gravity_pred.z * ref_down;
        Vector3_Normalize(&mag_ref_horizontal);
        
        // 计算磁场误差
        Vector3_Cross(&mag_horizontal, &mag_ref_horizontal, &mag_error);
        Vector3_Scale(&mag_error, att->mag_confidence, &mag_error);
    }
    
    // 合并误差
    vector3_t total_error;
    total_error.x = accel_error.x * att->accel_confidence + mag_error.x;
    total_error.y = accel_error.y * att->accel_confidence + mag_error.y;
    total_error.z = accel_error.z * att->accel_confidence + mag_error.z;
    
    // 估计陀螺仪偏置
    estimate_gyro_bias(att, &total_error);
    
    // 积分反馈
    att->integral_fb.x += total_error.x * att->ki * dt;
    att->integral_fb.y += total_error.y * att->ki * dt;
    att->integral_fb.z += total_error.z * att->ki * dt;
    
    // 应用PI控制器
    vector3_t gyro_corrected;
    gyro_corrected.x = gyro_cal.x + total_error.x * att->kp + att->integral_fb.x - att->gyro_bias.x;
    gyro_corrected.y = gyro_cal.y + total_error.y * att->kp + att->integral_fb.y - att->gyro_bias.y;
    gyro_corrected.z = gyro_cal.z + total_error.z * att->kp + att->integral_fb.z - att->gyro_bias.z;
    
    // 四元数积分
    quaternion_t dq;
    float half_dt = 0.5f * dt;
    dq.w = -half_dt * (att->q.x * gyro_corrected.x + att->q.y * gyro_corrected.y + att->q.z * gyro_corrected.z);
    dq.x = half_dt * (att->q.w * gyro_corrected.x + att->q.y * gyro_corrected.z - att->q.z * gyro_corrected.y);
    dq.y = half_dt * (att->q.w * gyro_corrected.y - att->q.x * gyro_corrected.z + att->q.z * gyro_corrected.x);
    dq.z = half_dt * (att->q.w * gyro_corrected.z + att->q.x * gyro_corrected.y - att->q.y * gyro_corrected.x);
    
    // 更新四元数
    att->q.w += dq.w;
    att->q.x += dq.x;
    att->q.y += dq.y;
    att->q.z += dq.z;
    
    // 归一化四元数
    Quaternion_Normalize(&att->q);
    
    // 更新欧拉角
    Quaternion_ToEuler(&att->q, &att->euler);
    
    // 计算融合质量
    float error_magnitude = Vector3_Length(&total_error);
    att->fusion_quality = expf(-error_magnitude * 10.0f); // 误差越小质量越高
}

/**
 * @brief 更新自适应增益
 */
static void update_adaptive_gains(enhanced_attitude_t *att, 
                                 const vector3_t *accel, 
                                 const vector3_t *mag)
{
    // 评估加速度计置信度（基于向量模长接近1g的程度）
    float accel_magnitude = Vector3_Length(accel);
    float accel_error = fabsf(accel_magnitude - 1.0f);
    att->accel_confidence = expf(-accel_error * 5.0f);
    
    // 评估磁力计置信度（基于磁场强度稳定性）
    float mag_magnitude = Vector3_Length(mag);
    static float mag_ref_magnitude = 0.0f;
    
    if (mag_ref_magnitude == 0.0f) {
        mag_ref_magnitude = mag_magnitude;
    } else {
        float mag_error = fabsf(mag_magnitude - mag_ref_magnitude) / mag_ref_magnitude;
        att->mag_confidence = expf(-mag_error * 3.0f);
        
        // 缓慢更新参考磁场强度
        mag_ref_magnitude = mag_ref_magnitude * 0.999f + mag_magnitude * 0.001f;
    }
    
    // 自适应调节增益
    float base_confidence = att->accel_confidence * 0.7f + att->mag_confidence * 0.3f;
    
    // 高置信度时使用高增益快速收敛，低置信度时使用低增益保持稳定
    att->kp = MAHONY_KP_MIN + (MAHONY_KP_MAX - MAHONY_KP_MIN) * base_confidence;
    att->ki = MAHONY_KI_MIN + (MAHONY_KI_MAX - MAHONY_KI_MIN) * base_confidence;
}

/**
 * @brief 检测磁干扰
 */
static void detect_magnetic_interference(enhanced_attitude_t *att, const vector3_t *mag)
{
    static vector3_t mag_history[10];
    static int history_index = 0;
    static bool history_full = false;
    
    // 保存历史数据
    mag_history[history_index] = *mag;
    history_index = (history_index + 1) % 10;
    if (history_index == 0) history_full = true;
    
    if (!history_full) return;
    
    // 计算磁场变化率
    float total_variation = 0.0f;
    for (int i = 0; i < 10; i++) {
        for (int j = i + 1; j < 10; j++) {
            vector3_t diff;
            diff.x = mag_history[i].x - mag_history[j].x;
            diff.y = mag_history[i].y - mag_history[j].y;
            diff.z = mag_history[i].z - mag_history[j].z;
            total_variation += Vector3_Length(&diff);
        }
    }
    
    // 如果变化率过大，降低磁力计置信度
    if (total_variation > MAG_INTERFERENCE_THRESHOLD) {
        att->mag_confidence *= 0.5f;
    }
}

/**
 * @brief 估计陀螺仪偏置
 */
static void estimate_gyro_bias(enhanced_attitude_t *att, const vector3_t *error)
{
    // 低通滤波器估计偏置
    float alpha = 0.001f; // 学习率
    att->gyro_bias.x = att->gyro_bias.x * (1.0f - alpha) + error->x * alpha;
    att->gyro_bias.y = att->gyro_bias.y * (1.0f - alpha) + error->y * alpha;
    att->gyro_bias.z = att->gyro_bias.z * (1.0f - alpha) + error->z * alpha;
}

/**
 * @brief 初始化参考向量
 */
static void initialize_reference_vectors(enhanced_attitude_t *att, 
                                        const vector3_t *accel, 
                                        const vector3_t *mag)
{
    // 重力参考向量就是归一化的重力向量
    att->accel_ref = *accel;
    Vector3_Normalize(&att->accel_ref);
    
    // 磁场参考向量需要归一化
    att->mag_ref = *mag;
    Vector3_Normalize(&att->mag_ref);
    
    // 根据加速度和磁场初始化姿态
    vector3_t north, east, down;
    down = att->accel_ref;
    
    // 计算东向量 = mag × down
    Vector3_Cross(&att->mag_ref, &down, &east);
    Vector3_Normalize(&east);
    
    // 计算北向量 = down × east
    Vector3_Cross(&down, &east, &north);
    Vector3_Normalize(&north);
    
    // 从方向余弦矩阵计算初始四元数
    float trace = north.x + east.y + down.z;
    if (trace > 0) {
        float s = sqrtf(trace + 1.0f) * 2.0f;
        att->q.w = 0.25f * s;
        att->q.x = (east.z - down.y) / s;
        att->q.y = (down.x - north.z) / s;
        att->q.z = (north.y - east.x) / s;
    } else if (north.x > east.y && north.x > down.z) {
        float s = sqrtf(1.0f + north.x - east.y - down.z) * 2.0f;
        att->q.w = (east.z - down.y) / s;
        att->q.x = 0.25f * s;
        att->q.y = (east.x + north.y) / s;
        att->q.z = (down.x + north.z) / s;
    } else if (east.y > down.z) {
        float s = sqrtf(1.0f + east.y - north.x - down.z) * 2.0f;
        att->q.w = (down.x - north.z) / s;
        att->q.x = (east.x + north.y) / s;
        att->q.y = 0.25f * s;
        att->q.z = (down.y + east.z) / s;
    } else {
        float s = sqrtf(1.0f + down.z - north.x - east.y) * 2.0f;
        att->q.w = (north.y - east.x) / s;
        att->q.x = (down.x + north.z) / s;
        att->q.y = (down.y + east.z) / s;
        att->q.z = 0.25f * s;
    }
    
    Quaternion_Normalize(&att->q);
    Quaternion_ToEuler(&att->q, &att->euler);
}

/**
 * @brief 获取欧拉角
 */
void Enhanced_Attitude_GetEuler(const enhanced_attitude_t *att, euler_t *euler)
{
    if (att && euler) {
        *euler = att->euler;
    }
}

/**
 * @brief 获取四元数
 */
void Enhanced_Attitude_GetQuaternion(const enhanced_attitude_t *att, quaternion_t *q)
{
    if (att && q) {
        *q = att->q;
    }
}

// 校准相关函数（简化实现）
void Enhanced_Attitude_StartMagCalibration(enhanced_attitude_t *att)
{
    if (!att) return;
    att->mag_cal.sample_count = 0;
    att->mag_cal.is_calibrated = false;
}

bool Enhanced_Attitude_UpdateMagCalibration(enhanced_attitude_t *att, const vector3_t *mag_raw)
{
    if (!att || !mag_raw) return false;
    
    // 简化的椭球拟合算法（实际应用中需要更复杂的算法）
    static vector3_t mag_min = {FLT_MAX, FLT_MAX, FLT_MAX};
    static vector3_t mag_max = {-FLT_MAX, -FLT_MAX, -FLT_MAX};
    
    // 更新最大最小值
    if (mag_raw->x < mag_min.x) mag_min.x = mag_raw->x;
    if (mag_raw->y < mag_min.y) mag_min.y = mag_raw->y;
    if (mag_raw->z < mag_min.z) mag_min.z = mag_raw->z;
    
    if (mag_raw->x > mag_max.x) mag_max.x = mag_raw->x;
    if (mag_raw->y > mag_max.y) mag_max.y = mag_raw->y;
    if (mag_raw->z > mag_max.z) mag_max.z = mag_raw->z;
    
    // 计算硬磁偏移（椭球中心）
    att->mag_cal.hard_iron.x = (mag_max.x + mag_min.x) * 0.5f;
    att->mag_cal.hard_iron.y = (mag_max.y + mag_min.y) * 0.5f;
    att->mag_cal.hard_iron.z = (mag_max.z + mag_min.z) * 0.5f;
    
    // 计算缩放因子
    float range_x = mag_max.x - mag_min.x;
    float range_y = mag_max.y - mag_min.y;
    float range_z = mag_max.z - mag_min.z;
    float avg_range = (range_x + range_y + range_z) / 3.0f;
    
    if (avg_range > 0) {
        att->mag_cal.scale[0] = avg_range / range_x;
        att->mag_cal.scale[1] = avg_range / range_y;
        att->mag_cal.scale[2] = avg_range / range_z;
    }
    
    att->mag_cal.sample_count++;
    
    // 需要足够的样本才认为校准完成
    return att->mag_cal.sample_count > 1000;
}

void Enhanced_Attitude_FinishMagCalibration(enhanced_attitude_t *att)
{
    if (!att) return;
    att->mag_cal.is_calibrated = true;
}

// 状态查询函数
float Enhanced_Attitude_GetAccelConfidence(const enhanced_attitude_t *att)
{
    return att ? att->accel_confidence : 0.0f;
}

float Enhanced_Attitude_GetMagConfidence(const enhanced_attitude_t *att)
{
    return att ? att->mag_confidence : 0.0f;
}

float Enhanced_Attitude_GetFusionQuality(const enhanced_attitude_t *att)
{
    return att ? att->fusion_quality : 0.0f;
}

bool Enhanced_Attitude_IsStable(const enhanced_attitude_t *att)
{
    return att ? (att->fusion_quality > 0.8f) : false;
}

// 应用校准函数
void Enhanced_Attitude_ApplyMagCalibration(const mag_calibration_t *cal, 
                                          const vector3_t *mag_raw, 
                                          vector3_t *mag_calibrated)
{
    if (!cal || !mag_raw || !mag_calibrated) return;
    
    if (cal->is_calibrated) {
        // 减去硬磁偏移
        vector3_t temp;
        temp.x = mag_raw->x - cal->hard_iron.x;
        temp.y = mag_raw->y - cal->hard_iron.y;
        temp.z = mag_raw->z - cal->hard_iron.z;
        
        // 应用软磁矩阵和缩放
        mag_calibrated->x = (cal->soft_iron[0][0] * temp.x + 
                            cal->soft_iron[0][1] * temp.y + 
                            cal->soft_iron[0][2] * temp.z) * cal->scale[0];
        mag_calibrated->y = (cal->soft_iron[1][0] * temp.x + 
                            cal->soft_iron[1][1] * temp.y + 
                            cal->soft_iron[1][2] * temp.z) * cal->scale[1];
        mag_calibrated->z = (cal->soft_iron[2][0] * temp.x + 
                            cal->soft_iron[2][1] * temp.y + 
                            cal->soft_iron[2][2] * temp.z) * cal->scale[2];
    } else {
        *mag_calibrated = *mag_raw;
    }
}

void Enhanced_Attitude_ApplyGyroCalibration(const gyro_calibration_t *cal, 
                                           const vector3_t *gyro_raw, 
                                           vector3_t *gyro_calibrated)
{
    if (!cal || !gyro_raw || !gyro_calibrated) return;
    
    if (cal->is_calibrated) {
        gyro_calibrated->x = gyro_raw->x - cal->offset.x;
        gyro_calibrated->y = gyro_raw->y - cal->offset.y;
        gyro_calibrated->z = gyro_raw->z - cal->offset.z;
    } else {
        *gyro_calibrated = *gyro_raw;
    }
}