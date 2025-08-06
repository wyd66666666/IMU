/**
 * @file enhanced_attitude.h
 * @brief 增强型姿态解算算法 - BMI270+BMM350融合
 * @author Claude
 * @date 2025-08-03
 * @version 1.0
 * 
 * 功能说明：
 * 1. 基于增强型Mahony算法的9轴姿态融合
 * 2. 磁力计校准和偏航角漂移纠正
 * 3. 自适应增益调节和磁干扰检测
 * 4. 陀螺仪偏置在线估计和补偿
 */

#ifndef ENHANCED_ATTITUDE_H
#define ENHANCED_ATTITUDE_H

#include <math.h>
#include <stdint.h>
#include <stdbool.h>

// 数学常数
#define PI_F                3.14159265358979f
#define DEG_TO_RAD          0.0174532925f
#define RAD_TO_DEG          57.2957795f
#define GRAVITY_MSS         9.80665f

// 算法参数
#define MAHONY_KP_MIN       0.5f        // 最小比例增益
#define MAHONY_KP_MAX       5.0f        // 最大比例增益
#define MAHONY_KI_MIN       0.0001f     // 最小积分增益
#define MAHONY_KI_MAX       0.1f        // 最大积分增益

#define MAG_DECLINATION     0.0f        // 磁偏角(度) - 根据地理位置设置
#define MAG_INTERFERENCE_THRESHOLD  500.0f  // 磁干扰检测阈值

// 数据结构定义
typedef struct {
    float x, y, z;
} vector3_t;

typedef struct {
    float w, x, y, z;
} quaternion_t;

typedef struct {
    float roll, pitch, yaw;
} euler_t;

typedef struct {
    vector3_t hard_iron;        // 硬磁偏移
    float soft_iron[3][3];      // 软磁矩阵
    float scale[3];             // 缩放因子
    bool is_calibrated;         // 校准状态
    uint32_t sample_count;      // 采样计数
} mag_calibration_t;

typedef struct {
    vector3_t offset;           // 陀螺仪零偏
    float noise_variance;       // 噪声方差
    bool is_calibrated;         // 校准状态
    uint32_t sample_count;      // 采样计数
} gyro_calibration_t;

typedef struct {
    // 姿态四元数
    quaternion_t q;
    
    // 欧拉角
    euler_t euler;
    
    // 传感器校准
    mag_calibration_t mag_cal;
    gyro_calibration_t gyro_cal;
    
    // 算法状态
    vector3_t gyro_bias;        // 陀螺仪偏置估计
    vector3_t accel_ref;        // 重力参考向量
    vector3_t mag_ref;          // 磁场参考向量
    
    // 自适应增益
    float kp;                   // 当前比例增益
    float ki;                   // 当前积分增益
    vector3_t integral_fb;      // 积分反馈
    
    // 质量评估
    float accel_confidence;     // 加速度计置信度
    float mag_confidence;       // 磁力计置信度
    float fusion_quality;       // 融合质量
    
    // 运行状态
    bool is_initialized;        // 初始化状态
    uint32_t update_count;      // 更新计数
    float dt;                   // 时间步长
    
} enhanced_attitude_t;

// 主要函数接口
void Enhanced_Attitude_Init(enhanced_attitude_t *att);
void Enhanced_Attitude_Update(enhanced_attitude_t *att, 
                             const vector3_t *accel, 
                             const vector3_t *gyro, 
                             const vector3_t *mag, 
                             float dt);
void Enhanced_Attitude_GetEuler(const enhanced_attitude_t *att, euler_t *euler);
void Enhanced_Attitude_GetQuaternion(const enhanced_attitude_t *att, quaternion_t *q);

// 校准功能
void Enhanced_Attitude_StartMagCalibration(enhanced_attitude_t *att);
bool Enhanced_Attitude_UpdateMagCalibration(enhanced_attitude_t *att, const vector3_t *mag_raw);
void Enhanced_Attitude_FinishMagCalibration(enhanced_attitude_t *att);

void Enhanced_Attitude_StartGyroCalibration(enhanced_attitude_t *att);
bool Enhanced_Attitude_UpdateGyroCalibration(enhanced_attitude_t *att, const vector3_t *gyro_raw);
void Enhanced_Attitude_FinishGyroCalibration(enhanced_attitude_t *att);

// 状态查询
float Enhanced_Attitude_GetAccelConfidence(const enhanced_attitude_t *att);
float Enhanced_Attitude_GetMagConfidence(const enhanced_attitude_t *att);
float Enhanced_Attitude_GetFusionQuality(const enhanced_attitude_t *att);
bool Enhanced_Attitude_IsStable(const enhanced_attitude_t *att);

// 参数设置
void Enhanced_Attitude_SetMagDeclination(enhanced_attitude_t *att, float declination_deg);
void Enhanced_Attitude_SetKpRange(enhanced_attitude_t *att, float kp_min, float kp_max);
void Enhanced_Attitude_SetKiRange(enhanced_attitude_t *att, float ki_min, float ki_max);

// 工具函数
void Enhanced_Attitude_ApplyMagCalibration(const mag_calibration_t *cal, 
                                          const vector3_t *mag_raw, 
                                          vector3_t *mag_calibrated);
void Enhanced_Attitude_ApplyGyroCalibration(const gyro_calibration_t *cal, 
                                           const vector3_t *gyro_raw, 
                                           vector3_t *gyro_calibrated);

// 数学工具函数
void Vector3_Normalize(vector3_t *v);
float Vector3_Length(const vector3_t *v);
float Vector3_Dot(const vector3_t *a, const vector3_t *b);
void Vector3_Cross(const vector3_t *a, const vector3_t *b, vector3_t *result);
void Vector3_Scale(const vector3_t *v, float scale, vector3_t *result);

void Quaternion_Normalize(quaternion_t *q);
void Quaternion_ToEuler(const quaternion_t *q, euler_t *euler);
void Quaternion_FromEuler(const euler_t *euler, quaternion_t *q);
void Quaternion_Multiply(const quaternion_t *a, const quaternion_t *b, quaternion_t *result);

#endif // ENHANCED_ATTITUDE_H