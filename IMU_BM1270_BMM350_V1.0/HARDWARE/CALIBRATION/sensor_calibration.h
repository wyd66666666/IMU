/**
 * @file sensor_calibration.h
 * @brief BMI270 + BMM350 + BMP580 Advanced Calibration System
 * @author Your Name
 * @date 2024
 */

#ifndef SENSOR_CALIBRATION_H
#define SENSOR_CALIBRATION_H

#include <stdint.h>
#include <stdbool.h>

/* Math constants */
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* Calibration configuration macros */
#define CALIB_VERSION               0x01000000  // v1.0.0.0
#define CALIB_MAGIC_NUMBER          0x43414C42  // "CALB"

/* Sampling configuration */
#define ACCEL_CALIB_SAMPLES         200
#define GYRO_CALIB_SAMPLES          1000
#define MAG_CALIB_SAMPLES           3000
#define BARO_CALIB_SAMPLES          100

/* Calibration status codes */
typedef enum {
    CALIB_OK = 0,
    CALIB_ERROR_INVALID_PARAM = -1,
    CALIB_ERROR_SENSOR_COMM = -2,
    CALIB_ERROR_TIMEOUT = -3,
    CALIB_ERROR_USER_ABORT = -4,
    CALIB_ERROR_INVALID_DATA = -5,
    CALIB_ERROR_STORAGE = -6
} calib_status_t;

/* Calibration types */
typedef enum {
    CALIB_TYPE_ACCEL = 0x01,
    CALIB_TYPE_GYRO = 0x02,
    CALIB_TYPE_MAG = 0x04,
    CALIB_TYPE_BARO = 0x08,
    CALIB_TYPE_ALL = 0x0F
} calib_type_t;

/* BMI270 calibration data */
typedef struct {
    /* Accelerometer calibration parameters */
    float accel_offset[3];      // Offset (m/s²)
    float accel_scale[3];       // Scale factor
    float accel_misalignment[3][3]; // Non-orthogonal matrix
    
    /* Gyroscope calibration parameters */
    float gyro_offset[3];       // Offset (rad/s)
    float gyro_scale[3];        // Scale factor
    float gyro_misalignment[3][3]; // Non-orthogonal matrix
    
    /* Temperature compensation */
    float accel_temp_coeff[3];  // Accelerometer temperature coefficient
    float gyro_temp_coeff[3];   // Gyroscope temperature coefficient
    float ref_temperature;      // Reference temperature
} bmi270_calib_t;

/* BMM350 calibration data */
typedef struct {
    /* Hard iron and soft iron correction */
    float hard_iron[3];         // Hard iron offset (μT)
    float soft_iron[3][3];      // Soft iron matrix
    
    /* Ellipsoid parameters */
    float center[3];            // Ellipsoid center
    float radii[3];             // Ellipsoid radii
    float rotation[3][3];       // Rotation matrix
    
    /* Temperature compensation */
    float mag_temp_coeff[3];    // Temperature coefficient
    float ref_temperature;      // Reference temperature
} bmm350_calib_t;

/* BMP580 calibration data */
typedef struct {
    float pressure_offset;      // Pressure offset (Pa)
    float altitude_offset;      // Altitude offset (m)
    float sea_level_pressure;   // Sea level pressure (Pa)
    float temp_coeff;          // Temperature coefficient
} bmp580_calib_t;

/* Comprehensive calibration data */
typedef struct {
    uint32_t magic_number;      // Magic number for validation
    uint32_t version;          // Version number
    uint32_t timestamp;        // Calibration timestamp
    uint32_t crc32;           // CRC checksum
    
    bmi270_calib_t bmi270;
    bmm350_calib_t bmm350;
    bmp580_calib_t bmp580;
    
    /* Calibration quality metrics */
    float accel_fitness;       // Accelerometer calibration quality (0-1)
    float gyro_fitness;        // Gyroscope calibration quality (0-1)
    float mag_fitness;         // Magnetometer calibration quality (0-1)
    float baro_fitness;        // Barometer calibration quality (0-1)
} sensor_calib_data_t;

/* Calibration progress callback */
typedef void (*calib_progress_cb)(const char* message, int progress);

/* User interaction callback */
typedef bool (*calib_user_action_cb)(const char* instruction);

/* Calibration configuration */
typedef struct {
    calib_progress_cb progress_callback;
    calib_user_action_cb user_action_callback;
    bool enable_temp_compensation;
    bool enable_advanced_fitting;
    uint32_t timeout_ms;
} calib_config_t;

/* Sensor device structure */
typedef struct {
    void *bmi270;    // BMI270 device pointer
    void *bmm350;    // BMM350 device pointer
    void *bmp580;    // BMP580 device pointer
} sensor_devices_t;

/* Main API functions */

/**
 * @brief Initialize calibration system
 * @param devices Sensor device pointers
 * @param config Calibration configuration
 * @return Calibration status
 */
calib_status_t calib_init(const sensor_devices_t *devices, 
                         const calib_config_t *config);

/**
 * @brief Perform full calibration
 * @param calib_types Sensor types to calibrate
 * @param calib_data Output calibration data
 * @return Calibration status
 */
calib_status_t calib_perform_full(calib_type_t calib_types, 
                                 sensor_calib_data_t *calib_data);

/**
 * @brief Perform quick calibration
 * @param calib_data Output calibration data
 * @return Calibration status
 */
calib_status_t calib_perform_quick(sensor_calib_data_t *calib_data);

/**
 * @brief Load calibration data
 * @param calib_data Calibration data pointer
 * @return Calibration status
 */
calib_status_t calib_load_data(sensor_calib_data_t *calib_data);

/**
 * @brief Save calibration data
 * @param calib_data Calibration data pointer
 * @return Calibration status
 */
calib_status_t calib_save_data(const sensor_calib_data_t *calib_data);

/**
 * @brief Validate calibration data
 * @param calib_data Calibration data pointer
 * @return true if valid, false otherwise
 */
bool calib_validate_data(const sensor_calib_data_t *calib_data);

/**
 * @brief Apply calibration to BMI270 data
 * @param raw_accel Raw accelerometer data (LSB)
 * @param raw_gyro Raw gyroscope data (LSB)
 * @param temperature Temperature (°C)
 * @param calib Calibration data
 * @param cal_accel Calibrated acceleration (m/s²)
 * @param cal_gyro Calibrated gyroscope (rad/s)
 */
void calib_apply_bmi270(const int16_t raw_accel[3], const int16_t raw_gyro[3],
                       float temperature, const bmi270_calib_t *calib,
                       float cal_accel[3], float cal_gyro[3]);

/**
 * @brief Apply calibration to BMM350 data
 * @param raw_mag Raw magnetometer data (LSB)
 * @param temperature Temperature (°C)
 * @param calib Calibration data
 * @param cal_mag Calibrated magnetic field (μT)
 */
void calib_apply_bmm350(const float raw_mag[3], float temperature,
                       const bmm350_calib_t *calib, float cal_mag[3]);

/**
 * @brief Apply calibration to BMP580 data
 * @param pressure Raw pressure (Pa)
 * @param temperature Temperature (°C)
 * @param calib Calibration data
 * @return Calibrated altitude (m)
 */
float calib_apply_bmp580(float pressure, float temperature,
                        const bmp580_calib_t *calib);

/* Advanced calibration algorithms */

/**
 * @brief Ellipsoid fitting algorithm
 * @param samples Sample data
 * @param count Sample count
 * @param center Output ellipsoid center
 * @param radii Output ellipsoid radii
 * @param rotation Output rotation matrix
 * @return Fitting error
 */
float calib_ellipsoid_fit(const float samples[][3], int count,
                         float center[3], float radii[3], 
                         float rotation[3][3]);

/**
 * @brief Get calibration quality report
 * @param calib_data Calibration data
 * @param report Output report string
 * @param max_len Maximum length
 */
void calib_get_quality_report(const sensor_calib_data_t *calib_data,
                             char *report, uint32_t max_len);

/* System functions to be implemented by user */
uint32_t get_system_timestamp(void);


/**
 * @brief 简单校准接口 - 一键校准所有传感器
 * @param None
 * @return 0: 成功, -1: 失败
 */
int sensor_simple_calibration(void);

/**
 * @brief 快速陀螺仪校准 - 仅校准陀螺仪零偏
 * @param None  
 * @return 0: 成功, -1: 失败
 */
int sensor_quick_gyro_calibration(void);

/**
 * @brief 应用校准到传感器数据
 * @param imu_data IMU数据指针
 * @param mag_data 磁力计数据指针 
 * @param pressure 气压值指针
 * @param temperature 温度值
 * @return 0: 成功, -1: 失败
 */
int sensor_apply_calibration(void *imu_data, float *mag_data, float *pressure, float temperature);

/**
 * @brief 检查是否有有效的校准数据
 * @return 1: 有效, 0: 无效
 */
int sensor_has_valid_calibration(void);

/**
 * @brief 获取校准状态报告
 */
void sensor_get_calibration_status(void);

/* System functions to be implemented by user */
uint32_t get_system_timestamp(void);

/* 获取校准偏移值的接口函数 */
void calib_get_gyro_offset(float offset[3]);
void calib_get_accel_offset(float offset[3]);
void calib_get_accel_scale(float scale[3]);
void calib_get_mag_offset(float offset[3]);
float calib_get_sea_level_pressure(void);
float calib_get_altitude_offset(void);

#endif /* SENSOR_CALIBRATION_H */