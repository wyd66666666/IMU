/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <math.h>
#include "usart_printf.h"
#include "delay.h"

#include "bmi270_bmm350.h"  // BMI270+BMM350库
#include "bmp5_port.h"      // BMP580库

#include "sensor_calibration.h"
#include "fusion_filter.h"  // 使用互补滤波器
#include "anony_protocol.h" // 匿名上位机协议
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifndef PI
#define PI 3.14159265358979323846f
#endif

#define DEG_TO_RAD (PI / 180.0f)
#define RAD_TO_DEG (180.0f / PI)

// 低通滤波器系数 (0-1，越小滤波越强)
#define ACCEL_FILTER 0.02f  // 加速度计滤波（很强）
#define GYRO_FILTER 0.01f   // 陀螺仪滤波（强）
#define MAG_FILTER 0.01f    // 磁力计滤波（非常强）
#define BARO_FILTER 0.01f   // 气压计滤波（很强）

// 校准采样数
#define GYRO_CALIB_SAMPLES 1500  // 增加陀螺仪校准样本数
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t rx_buf[RX_BUF_SIZE];
uint8_t tx_buf[TX_BUF_SIZE];
volatile uint8_t tx_busy = 0; // 发送忙标志

// 姿态解算相关变量
static uint32_t last_update_time = 0;  // 上次更新时间
static uint8_t send_to_anony = 1;      // 是否发送到匿名上位机

// 滤波后的数据
float accel_filtered[3] = {0};
float gyro_filtered[3] = {0};
float mag_filtered[3] = {0};
float baro_alt_filtered = 0.0f;

// 校准参数
float gyro_bias[3] = {0};
float calib_gyro_offset[3] = {0};
float calib_accel_offset[3] = {0};
float calib_accel_scale[3] = {1.0f, 1.0f, 1.0f};
float calib_mag_offset[3] = {0};
float sea_level_pressure = 101325.0f;
float altitude_offset = 0.0f;

// 传感器控制
uint8_t use_magnetometer = 0;  // 默认不使用磁力计
uint8_t calibration_done = 0;  // 是否进行过校准

// 陀螺仪饱和检测
uint32_t gyro_saturate_count = 0;
uint32_t gyro_error_count = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void perform_gyro_calibration(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* 增强的陀螺仪校准函数 */
void perform_gyro_calibration(void)
{
    printf("Performing enhanced gyro calibration...\n");
    printf("Keep device ABSOLUTELY STILL!\n");
    
    imu_data_t imu_data;
    float gyro_sum[3] = {0};
    float gyro_min[3] = {1000, 1000, 1000};
    float gyro_max[3] = {-1000, -1000, -1000};
    int valid_samples = 0;
    
    // LED指示校准开始
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
    
    // 等待3秒让设备完全静止
    for (int i = 3; i > 0; i--) {
        printf("Starting in %d seconds...\n", i);
        HAL_Delay(1000);
    }
    
    // 收集数据
    for (int i = 0; i < GYRO_CALIB_SAMPLES; i++) {
        if (IMU_ReadData(&imu_data) == BMI2_OK) {
            // 检查数据是否在合理范围内（静止时应该接近0）
            if (fabsf(imu_data.gyr_x) < 10.0f && 
                fabsf(imu_data.gyr_y) < 10.0f && 
                fabsf(imu_data.gyr_z) < 10.0f) {
                
                gyro_sum[0] += imu_data.gyr_x;
                gyro_sum[1] += imu_data.gyr_y;
                gyro_sum[2] += imu_data.gyr_z;
                
                // 记录最大最小值
                for (int j = 0; j < 3; j++) {
                    float val = (j == 0) ? imu_data.gyr_x : (j == 1) ? imu_data.gyr_y : imu_data.gyr_z;
                    if (val < gyro_min[j]) gyro_min[j] = val;
                    if (val > gyro_max[j]) gyro_max[j] = val;
                }
                
                valid_samples++;
            }
        }
        HAL_Delay(2);
        
        if (i % 300 == 0 && i > 0) {
            printf("Calibration progress: %d%%\n", i * 100 / GYRO_CALIB_SAMPLES);
        }
    }
    
    // 计算平均值
    if (valid_samples > GYRO_CALIB_SAMPLES * 0.8) {  // 至少80%的样本有效
        for (int i = 0; i < 3; i++) {
            gyro_bias[i] = gyro_sum[i] / valid_samples;
            float range = gyro_max[i] - gyro_min[i];
            
            printf("Axis %c - Bias: %.3f deg/s, Range: %.3f\n", 
                   'X' + i, gyro_bias[i], range);
            
            // 如果范围太大，说明设备在移动
            if (range > 2.0f) {
                printf("WARNING: Axis %c shows movement! Recalibration recommended.\n", 'X' + i);
            }
        }
        printf("Valid samples: %d/%d (%.1f%%)\n", 
               valid_samples, GYRO_CALIB_SAMPLES, 
               (float)valid_samples * 100.0f / GYRO_CALIB_SAMPLES);
    } else {
        printf("ERROR: Not enough valid samples! (%d/%d)\n", 
               valid_samples, GYRO_CALIB_SAMPLES);
        // 使用默认值
        gyro_bias[0] = gyro_bias[1] = gyro_bias[2] = 0.0f;
    }
    
    // LED指示校准完成
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  #pragma import(__use_no_semihosting)
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  // 启动串口DMA接收
  HAL_UART_Receive_DMA(&huart1, rx_buf, RX_BUF_SIZE);

  // 启动串口中断	
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

  DWT_Delay_Init(); // 初始化DWT延时
  usart_printf_init(&huart1);
	 
  printf("\n\n========================================\n");
  printf("Multi-Sensor Fusion Program v2.0\n");
  printf("- Madgwick filter algorithm\n");
  printf("- Enhanced gyro calibration\n");
  printf("- Drift compensation\n");
  printf("========================================\n\n");
    
  /* 初始化BMI270 */
  printf("Initializing BMI270 (I2C1)...\n");
  int8_t imu_status = IMU_Init(1);
  if (imu_status != BMI2_OK) {
      printf("BMI270 initialization failed!\n");
      while(1);  // 停止执行
  } else {
      printf("BMI270 initialization successful!\n");
  }
    
  /* 初始化BMP580 */
  printf("\nInitializing BMP580 (I2C2)...\n");
  int8_t bmp_status = BMP580_Init();
  if (bmp_status != BMP5_OK) {
      printf("BMP580 initialization failed!\n");
  } else {
      printf("BMP580 initialization successful!\n");
  }
  
  /* 校准检查和执行 */
  printf("\n=== Calibration Check ===\n");

  // 检查现有校准数据
  if (sensor_has_valid_calibration()) {
      printf("Valid calibration data found!\n");
      sensor_get_calibration_status();
      calibration_done = 1;
      printf("Press CAL_KEY within 3 seconds to recalibrate...\n");
  } else {
      printf("No calibration data found.\n");
      printf("Press CAL_KEY within 3 seconds to calibrate...\n");
  }

  // 等待按键检测
  uint32_t calib_wait_start = HAL_GetTick();
  uint8_t calib_key_pressed = 0;

  while (HAL_GetTick() - calib_wait_start < 3000) {
      if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET) {
          HAL_Delay(20); // 防抖
          if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET) {
              calib_key_pressed = 1;
              
              // LED确认
              HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
              HAL_Delay(300);
              HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
              
              // 等待按键释放
              while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET);
              break;
          }
      }
  }

  // 执行校准流程
  if (calib_key_pressed) {
      printf("\n=== Starting Calibration ===\n");
      
      // 选择校准类型
      uint8_t calib_type = 1; // 默认完整校准
      
      if (calibration_done) {
          // 已有校准数据，提供选择
          printf("1. Full calibration (all sensors)\n");
          printf("2. Quick gyro calibration only\n");
          printf("Press CAL_KEY again within 3s to select quick mode...\n");
          
          uint32_t select_start = HAL_GetTick();
          while (HAL_GetTick() - select_start < 3000) {
              if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET) {
                  HAL_Delay(20);
                  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET) {
                      calib_type = 2; // 快速校准
                      printf("Quick calibration selected!\n");
                      
                      // LED快闪确认
                      for (int i = 0; i < 3; i++) {
                          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
                          HAL_Delay(100);
                          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
                          HAL_Delay(100);
                      }
                      
                      while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET);
                      break;
                  }
              }
          }
      }
      
      // 执行校准
      int calib_result = -1;
      
      if (calib_type == 2) {
          printf("Starting quick gyro calibration...\n");
          calib_result = sensor_quick_gyro_calibration();
      } else {
          printf("Starting full sensor calibration...\n");
          calib_result = sensor_simple_calibration();
      }
      
      // 处理校准结果
      if (calib_result == 0) {
          printf("\n*** Calibration SUCCESS! ***\n");
          calibration_done = 1;
          
          // LED长亮表示成功
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
          HAL_Delay(2000);
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
          
          // 显示校准结果
          sensor_get_calibration_status();
          
            } else {
          printf("\n*** Calibration FAILED! ***\n");
          
          // LED快闪表示失败
          for (int i = 0; i < 5; i++) {
              HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
              HAL_Delay(200);
              HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
              HAL_Delay(200);
          }
      }
      
  } else {
      // 没有按键，跳过校准
      if (calibration_done) {
          printf("Using existing calibration data.\n");
      } else {
          printf("No calibration performed. Using raw sensor data.\n");
      }
  }

  /* 初始化Madgwick滤波器 */
  printf("\n=== Initializing Madgwick Filter ===\n");
  fusion_init();

  // 设置滤波器参数 - 使用保守的增益
  fusion_set_params(0.98f, 0.02f, 0.01f);  
  printf("Madgwick beta parameter set to 0.033\n");

  // 进行初始陀螺仪校准
  printf("\n=== Performing Initial Gyro Calibration ===\n");
  printf("This is critical for drift prevention.\n");
  perform_gyro_calibration();

  // 设置陀螺仪偏差
  fusion_set_gyro_bias(gyro_bias);

  // 默认不使用磁力计
  fusion_use_magnetometer(0);

  printf("\n--- Starting Sensor Fusion ---\n");
  printf("Keep device stationary for 3 seconds to initialize...\n");

  // 静止初始化阶段
  HAL_Delay(3000);
  
  // 加载校准参数
  if (calibration_done) {
      calib_get_gyro_offset(calib_gyro_offset);
      calib_get_accel_offset(calib_accel_offset);
      calib_get_accel_scale(calib_accel_scale);
      calib_get_mag_offset(calib_mag_offset);
      sea_level_pressure = calib_get_sea_level_pressure();
      altitude_offset = calib_get_altitude_offset();
      
      printf("Calibration data loaded successfully\n");
  }
  
  printf("System ready! Press KEY to toggle magnetometer.\n\n");

  /* 数据结构体 */
  imu_data_t imu_data;
  float temperature = 25.0f;
  float pressure = 101325.0f;
  
  // 初始化滤波器变量
  memset(accel_filtered, 0, sizeof(accel_filtered));
  memset(gyro_filtered, 0, sizeof(gyro_filtered));
  memset(mag_filtered, 0, sizeof(mag_filtered));

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    // 计算时间间隔
    uint32_t current_time = HAL_GetTick();
    float dt = (current_time - last_update_time) / 1000.0f;  // 转换为秒
    
    // 限制dt范围，防止大跳变
    if (last_update_time == 0 || dt > 0.1f || dt < 0.001f) {
        dt = 0.02f;  // 默认20ms
    }
    last_update_time = current_time;
    
    /* 读取传感器数据 */
    if (IMU_ReadData(&imu_data) == BMI2_OK) {
        
        // 读取BMP580气压计数据（如果可用）
        float baro_altitude = 0.0f;
        if (BMP580_ReadData(&temperature, &pressure) == BMP5_OK) {
            // 计算高度
            baro_altitude = 44330.0f * (1.0f - powf(pressure / 101325.0f, 1.0f/5.255f));
            
            // 应用滤波
            if (baro_alt_filtered == 0.0f) {
                baro_alt_filtered = baro_altitude;
            } else {
                baro_alt_filtered = baro_alt_filtered * (1.0f - BARO_FILTER) + baro_altitude * BARO_FILTER;
            }
        }
        
        /* 应用校准和滤波 */
        
        // 检查陀螺仪饱和
        if (fabsf(imu_data.gyr_x) > 1900.0f || 
            fabsf(imu_data.gyr_y) > 1900.0f || 
            fabsf(imu_data.gyr_z) > 1900.0f) {
            // 陀螺仪饱和，可能会导致积分误差
            gyro_saturate_count++;
            if (gyro_saturate_count % 100 == 0) {
                printf("WARNING: Gyroscope saturation detected! (%lu)\n", gyro_saturate_count);
            }
        }
        
        // 0. 应用陀螺仪零偏校正（从初始化阶段获得的）
        imu_data.gyr_x -= gyro_bias[0];
        imu_data.gyr_y -= gyro_bias[1];
        imu_data.gyr_z -= gyro_bias[2];
        
        // 1. 应用额外校准参数（如果有）
        if (calibration_done) {
            // 应用额外的陀螺仪校准
            imu_data.gyr_x -= calib_gyro_offset[0];
            imu_data.gyr_y -= calib_gyro_offset[1];
            imu_data.gyr_z -= calib_gyro_offset[2];
            
            // 应用加速度计校准
            imu_data.acc_x = (imu_data.acc_x - calib_accel_offset[0]) * calib_accel_scale[0];
            imu_data.acc_y = (imu_data.acc_y - calib_accel_offset[1]) * calib_accel_scale[1];
            imu_data.acc_z = (imu_data.acc_z - calib_accel_offset[2]) * calib_accel_scale[2];
            
            // 应用磁力计校准
            imu_data.mag_x -= calib_mag_offset[0];
            imu_data.mag_y -= calib_mag_offset[1];
            imu_data.mag_z -= calib_mag_offset[2];
        }
        
        // 2. 初始化滤波器（只在第一次运行时）
        static uint8_t first_run = 1;
        if (first_run) {
            accel_filtered[0] = imu_data.acc_x;
            accel_filtered[1] = imu_data.acc_y;
            accel_filtered[2] = imu_data.acc_z;
            
            gyro_filtered[0] = imu_data.gyr_x;
            gyro_filtered[1] = imu_data.gyr_y;
            gyro_filtered[2] = imu_data.gyr_z;
            
            mag_filtered[0] = imu_data.mag_x;
            mag_filtered[1] = imu_data.mag_y;
            mag_filtered[2] = imu_data.mag_z;
            
            first_run = 0;
        }
        
        // 3. 应用低通滤波
        accel_filtered[0] = accel_filtered[0] * (1.0f - ACCEL_FILTER) + imu_data.acc_x * ACCEL_FILTER;
        accel_filtered[1] = accel_filtered[1] * (1.0f - ACCEL_FILTER) + imu_data.acc_y * ACCEL_FILTER;
        accel_filtered[2] = accel_filtered[2] * (1.0f - ACCEL_FILTER) + imu_data.acc_z * ACCEL_FILTER;
        
        gyro_filtered[0] = gyro_filtered[0] * (1.0f - GYRO_FILTER) + imu_data.gyr_x * GYRO_FILTER;
        gyro_filtered[1] = gyro_filtered[1] * (1.0f - GYRO_FILTER) + imu_data.gyr_y * GYRO_FILTER;
        gyro_filtered[2] = gyro_filtered[2] * (1.0f - GYRO_FILTER) + imu_data.gyr_z * GYRO_FILTER;
        
        mag_filtered[0] = mag_filtered[0] * (1.0f - MAG_FILTER) + imu_data.mag_x * MAG_FILTER;
        mag_filtered[1] = mag_filtered[1] * (1.0f - MAG_FILTER) + imu_data.mag_y * MAG_FILTER;
        mag_filtered[2] = mag_filtered[2] * (1.0f - MAG_FILTER) + imu_data.mag_z * MAG_FILTER;
        
        /* 更新Madgwick滤波器姿态估计 */
        
        // 准备传感器数据
        float acc[3] = {accel_filtered[0], accel_filtered[1], accel_filtered[2]};
        float gyro[3] = {gyro_filtered[0], gyro_filtered[1], gyro_filtered[2]};
        float *mag_ptr = NULL;  // 默认不使用磁力计
        
        // 只有在启用且磁场强度合理时使用磁力计
        if (use_magnetometer) {
            float mag_norm = sqrtf(mag_filtered[0]*mag_filtered[0] + 
                                 mag_filtered[1]*mag_filtered[1] + 
                                 mag_filtered[2]*mag_filtered[2]);
            
            if (mag_norm > 5.0f && mag_norm < 100.0f) {
                mag_ptr = mag_filtered;
            } else {
                static uint32_t mag_error_count = 0;
                if (++mag_error_count % 100 == 0) {
                    printf("Mag field strength abnormal: %.1f uT\n", mag_norm);
                }
            }
        }
        
        // 更新融合滤波器
        fusion_update(acc, gyro, mag_ptr, baro_alt_filtered, dt);
        
        // 获取融合后的姿态
        attitude_data_t attitude = fusion_get_attitude();
        
        // 发送姿态角到匿名上位机（10Hz）
        static uint8_t send_count = 0;
        if (++send_count >= 5) {  // 每100ms发送一次（10Hz）
            send_count = 0;
            
            // 发送姿态数据
            ANO_Send_Attitude(attitude.roll, attitude.pitch, attitude.yaw);
            
            // 发送传感器原始数据
            int16_t acc_i16[3], gyro_i16[3], mag_i16[3];
            
            // 将浮点数转换为整数
            for (int i = 0; i < 3; i++) {
                acc_i16[i] = (int16_t)(accel_filtered[i] * 100.0f);  // cm/s²
                gyro_i16[i] = (int16_t)(gyro_filtered[i] * 10.0f);   // 0.1°/s
                mag_i16[i] = (int16_t)(mag_filtered[i]);             // uT
            }
            
            ANO_Send_Sensor(acc_i16[0], acc_i16[1], acc_i16[2],
                           gyro_i16[0], gyro_i16[1], gyro_i16[2],
                           mag_i16[0], mag_i16[1], mag_i16[2]);
            
            // 发送用户数据
            ANO_Send_UserData(attitude.confidence * 100.0f,  // 融合置信度
                             temperature,                    // 温度
                             baro_alt_filtered,              // 高度
                             gyro_bias[0],                   // X轴陀螺仪偏差
                             gyro_bias[1]);                  // Y轴陀螺仪偏差
        }
        
        // 串口打印（1Hz）
        static uint32_t print_count = 0;
        if (++print_count >= 50) {  // 每1秒打印一次
            print_count = 0;
            
            printf("Att[R:%5.1f P:%5.1f Y:%5.1f] ", 
                   attitude.roll, attitude.pitch, attitude.yaw);
            
            printf("Gyr[%5.2f %5.2f %5.2f] ", 
                   gyro_filtered[0], gyro_filtered[1], gyro_filtered[2]);
            
            printf("Bias[%5.2f %5.2f %5.2f] ", 
                   gyro_bias[0], gyro_bias[1], gyro_bias[2]);
            
            if (use_magnetometer) {
                printf("Mag[ON] ");
            } else {
                printf("Mag[OFF] ");
            }
            
            printf("%s\n", fusion_get_debug_info());
        }
        
        // 按键检测 - 切换磁力计使用状态
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET) {
            HAL_Delay(50); // 防抖
            if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET) {
                // 切换磁力计使用状态
                use_magnetometer = !use_magnetometer;
                fusion_use_magnetometer(use_magnetometer);
                
                printf("\n*** Magnetometer %s ***\n", 
                       use_magnetometer ? "ENABLED" : "DISABLED");
                
                // LED指示
                for (int i = 0; i < (use_magnetometer ? 2 : 4); i++) {
                    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
                    HAL_Delay(10);
                }
                
                // 等待按键释放
                while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET) {
                    HAL_Delay(10);
                }
            }
        }
        
        // LED心跳指示
        static uint32_t led_count = 0;
        if (++led_count >= 25) {  // 每500ms闪烁一次
            led_count = 0;
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
        }
    } else {
        // 传感器读取失败
        if (++gyro_error_count % 100 == 0) {
            printf("ERROR: IMU read failed! (%lu)\n", gyro_error_count);
            
            // 尝试重新初始化IMU
            if (gyro_error_count >= 1000) {
                printf("Attempting to reinitialize IMU...\n");
                IMU_Init(1);
                gyro_error_count = 0;
            }
        }
        HAL_Delay(10);
    }
    
    // 主循环延时 - 降低延时以获得更高的更新率
    HAL_Delay(10);  // 10ms，100Hz主循环
    
  } /* while循环结束 */
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void UART1_IDLE_Callback(uint16_t len)
{
    if(len > 0)
    {
        HAL_UART_Transmit(&huart1, rx_buf, len, 100);
        memset(rx_buf, 0, RX_BUF_SIZE); // 清空缓存
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */