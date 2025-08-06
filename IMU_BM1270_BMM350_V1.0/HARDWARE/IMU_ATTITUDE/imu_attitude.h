#ifndef IMU_ATTITUDE_H
#define IMU_ATTITUDE_H

#include <math.h>
#include <stdint.h>

// --- 宏定义与全局变量 ---
#ifndef PI
#define PI 3.14159265358979323846f
#endif

#define DEG_TO_RAD (PI / 180.0f)
#define RAD_TO_DEG (180.0f / PI)
// 姿态角结构体
typedef struct {
    float roll;    // 横滚角(度)
    float pitch;   // 俯仰角(度)
    float yaw;     // 偏航角(度)
} attitude_t;

// 初始化姿态解算
void imu_attitude_init(void);

// 更新姿态解算
void imu_attitude_update(float ax, float ay, float az, 
                         float gx, float gy, float gz,
                         float mx, float my, float mz,
                         float dt);

// 获取当前姿态角
attitude_t imu_attitude_get(void);

#endif



												 