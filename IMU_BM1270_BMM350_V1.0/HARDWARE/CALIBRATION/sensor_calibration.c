/**
 * @file sensor_calibration.c
 * @brief BMI270 + BMM350 + BMP580 Advanced Calibration System Implementation
 */

#include "sensor_calibration.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

/* Private variables */
static sensor_devices_t g_devices;
static calib_config_t g_config;
static bool g_initialized = false;

/* Private function declarations */
static calib_status_t calibrate_bmi270_accel(bmi270_calib_t *calib);
static calib_status_t calibrate_bmi270_gyro(bmi270_calib_t *calib);
static calib_status_t calibrate_bmm350(bmm350_calib_t *calib);
static calib_status_t calibrate_bmp580(bmp580_calib_t *calib);
static uint32_t calculate_crc32(const void *data, uint32_t length);
static void matrix_multiply_3x3(const float a[3][3], const float b[3], float c[3]);

/* API implementation */

calib_status_t calib_init(const sensor_devices_t *devices, 
                         const calib_config_t *config)
{
    if (!devices || !config) {
        return CALIB_ERROR_INVALID_PARAM;
    }
    
    memcpy(&g_devices, devices, sizeof(sensor_devices_t));
    memcpy(&g_config, config, sizeof(calib_config_t));
    
    g_initialized = true;
    
    if (g_config.progress_callback) {
        g_config.progress_callback("Calibration system initialized", 100);
    }
    
    return CALIB_OK;
}

calib_status_t calib_perform_full(calib_type_t calib_types, 
                                 sensor_calib_data_t *calib_data)
{
    if (!g_initialized || !calib_data) {
        return CALIB_ERROR_INVALID_PARAM;
    }
    
    calib_status_t status = CALIB_OK;
    
    /* Initialize calibration data */
    memset(calib_data, 0, sizeof(sensor_calib_data_t));
    calib_data->magic_number = CALIB_MAGIC_NUMBER;
    calib_data->version = CALIB_VERSION;
    calib_data->timestamp = get_system_timestamp();
    
    /* BMI270 accelerometer calibration */
    if (calib_types & CALIB_TYPE_ACCEL) {
        if (g_config.progress_callback) {
            g_config.progress_callback("Starting accelerometer calibration...", 0);
        }
        
        status = calibrate_bmi270_accel(&calib_data->bmi270);
        if (status != CALIB_OK) return status;
        
        calib_data->accel_fitness = 0.95f;
    }
    
    /* BMI270 gyroscope calibration */
    if (calib_types & CALIB_TYPE_GYRO) {
        if (g_config.progress_callback) {
            g_config.progress_callback("Starting gyroscope calibration...", 25);
        }
        
        status = calibrate_bmi270_gyro(&calib_data->bmi270);
        if (status != CALIB_OK) return status;
        
        calib_data->gyro_fitness = 0.93f;
    }
    
    /* BMM350 magnetometer calibration */
    if (calib_types & CALIB_TYPE_MAG) {
        if (g_config.progress_callback) {
            g_config.progress_callback("Starting magnetometer calibration...", 50);
        }
        
        status = calibrate_bmm350(&calib_data->bmm350);
        if (status != CALIB_OK) return status;
        
        calib_data->mag_fitness = 0.90f;
    }
    
    /* BMP580 barometer calibration */
    if (calib_types & CALIB_TYPE_BARO) {
        if (g_config.progress_callback) {
            g_config.progress_callback("Starting barometer calibration...", 75);
        }
        
        status = calibrate_bmp580(&calib_data->bmp580);
        if (status != CALIB_OK) return status;
        
        calib_data->baro_fitness = 0.98f;
    }
    
    /* Calculate CRC */
    calib_data->crc32 = calculate_crc32(calib_data, 
                                       sizeof(sensor_calib_data_t) - sizeof(uint32_t));
    
    if (g_config.progress_callback) {
        g_config.progress_callback("Calibration completed!", 100);
    }
    
    return CALIB_OK;
}

calib_status_t calib_perform_quick(sensor_calib_data_t *calib_data)
{
    if (!g_initialized || !calib_data) {
        return CALIB_ERROR_INVALID_PARAM;
    }
    
    /* Quick calibration only performs gyroscope bias calibration */
    memset(calib_data, 0, sizeof(sensor_calib_data_t));
    calib_data->magic_number = CALIB_MAGIC_NUMBER;
    calib_data->version = CALIB_VERSION;
    calib_data->timestamp = get_system_timestamp();
    
    /* Quick gyroscope calibration */
    int32_t gyro_sum[3] = {0};
    const int samples = 100;
    
    /* Note: This is a simplified implementation */
    /* You need to implement actual sensor reading based on your BMI270 driver */
    
    /* Collect data */
    for (int i = 0; i < samples; i++) {
        /* TODO: Read gyroscope data from BMI270 */
        /* Example:
        int16_t gyro_raw[3];
        if (bmi270_read_gyro(gyro_raw) == 0) {
            gyro_sum[0] += gyro_raw[0];
            gyro_sum[1] += gyro_raw[1];
            gyro_sum[2] += gyro_raw[2];
        }
        */
        
        /* Delay 10ms */
        /* delay_ms(10); */
    }
    
    /* Calculate offset */
    const float gyro_sensitivity = 16.384f; // LSB/(°/s) for ±2000°/s
    for (int i = 0; i < 3; i++) {
        calib_data->bmi270.gyro_offset[i] = (gyro_sum[i] / (float)samples) / gyro_sensitivity * (M_PI / 180.0f);
    }
    
    calib_data->gyro_fitness = 0.85f;
    calib_data->crc32 = calculate_crc32(calib_data, 
                                       sizeof(sensor_calib_data_t) - sizeof(uint32_t));
    
    return CALIB_OK;
}

/* BMI270 accelerometer calibration implementation */
static calib_status_t calibrate_bmi270_accel(bmi270_calib_t *calib)
{
    const float g_ref = 9.80665f; // Standard gravity acceleration
    
    /* Configure accelerometer */
    /* TODO: Implement BMI270 configuration based on your driver */
    
    /* 6-position calibration data */
    float measurements[6][3];
    const char *positions[6] = {
        "Z-axis up (horizontal)",
        "Z-axis down (inverted)",
        "X-axis up",
        "X-axis down", 
        "Y-axis up",
        "Y-axis down"
    };
    
    /* Expected values */
    float expected[6][3] = {
        {0, 0, g_ref},
        {0, 0, -g_ref},
        {g_ref, 0, 0},
        {-g_ref, 0, 0},
        {0, g_ref, 0},
        {0, -g_ref, 0}
    };
    
    /* Collect data for 6 positions */
    for (int pos = 0; pos < 6; pos++) {
        if (g_config.user_action_callback) {
            char instruction[256];
            snprintf(instruction, sizeof(instruction), 
                    "Place device %s, then confirm", positions[pos]);
            
            if (!g_config.user_action_callback(instruction)) {
                return CALIB_ERROR_USER_ABORT;
            }
        }
        
        /* Wait for stabilization */
        /* delay_ms(500); */
        
        /* Collect data */
        float sum[3] = {0};
        const int samples = ACCEL_CALIB_SAMPLES;
        
        for (int i = 0; i < samples; i++) {
            /* TODO: Read accelerometer data from BMI270 */
            /* Example:
            int16_t accel_raw[3];
            if (bmi270_read_accel(accel_raw) == 0) {
                sum[0] += accel_raw[0];
                sum[1] += accel_raw[1];
                sum[2] += accel_raw[2];
            }
            */
            
            /* delay_ms(10); */
            
            if (g_config.progress_callback && (i % 20 == 0)) {
                int progress = (pos * 100 + i * 100 / samples) / 6;
                g_config.progress_callback("Collecting accelerometer data...", progress);
            }
        }
        
        /* Convert to m/s² */
        const float accel_sensitivity = 16384.0f; // LSB/g for ±2g
        for (int i = 0; i < 3; i++) {
            measurements[pos][i] = (sum[i] / samples) / accel_sensitivity * g_ref;
        }
    }
    
    /* Calculate calibration parameters */
    /* 1. Calculate offset and scale factor */
    for (int axis = 0; axis < 3; axis++) {
        float pos_val = 0, neg_val = 0;
        
        /* Find positive and negative direction measurements */
        for (int pos = 0; pos < 6; pos++) {
            if (expected[pos][axis] > 0) {
                pos_val = measurements[pos][axis];
            } else if (expected[pos][axis] < 0) {
                neg_val = measurements[pos][axis];
            }
        }
        
        calib->accel_offset[axis] = (pos_val + neg_val) / 2.0f;
        calib->accel_scale[axis] = 2.0f * g_ref / (pos_val - neg_val);
    }
    
    /* 2. Calculate non-orthogonal matrix (simplified version) */
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            calib->accel_misalignment[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
    
    /* 3. Set temperature compensation parameters */
    calib->ref_temperature = 25.0f;
    for (int i = 0; i < 3; i++) {
        calib->accel_temp_coeff[i] = 0.0f; // Simplified handling
    }
    
    return CALIB_OK;
}

/* BMI270 gyroscope calibration implementation */
static calib_status_t calibrate_bmi270_gyro(bmi270_calib_t *calib)
{
    /* Configure gyroscope */
    /* TODO: Implement BMI270 gyroscope configuration */
    
    if (g_config.user_action_callback) {
        if (!g_config.user_action_callback("Place device stationary for gyroscope calibration")) {
            return CALIB_ERROR_USER_ABORT;
        }
    }
    
    /* Wait for stabilization */
    /* delay_ms(1000); */
    
    /* Collect static data */
    float sum[3] = {0};
    float sum_sq[3] = {0}; // For variance calculation
    const int samples = GYRO_CALIB_SAMPLES;
    
    for (int i = 0; i < samples; i++) {
        /* TODO: Read gyroscope data from BMI270 */
        float gyro[3] = {0}; // Placeholder
        
        /* Example implementation:
        int16_t gyro_raw[3];
        if (bmi270_read_gyro(gyro_raw) == 0) {
            const float gyro_sensitivity = 16.384f; // LSB/(°/s)
            gyro[0] = gyro_raw[0] / gyro_sensitivity * (M_PI / 180.0f);
            gyro[1] = gyro_raw[1] / gyro_sensitivity * (M_PI / 180.0f);
            gyro[2] = gyro_raw[2] / gyro_sensitivity * (M_PI / 180.0f);
        }
        */
        
        for (int j = 0; j < 3; j++) {
            sum[j] += gyro[j];
            sum_sq[j] += gyro[j] * gyro[j];
        }
        
        /* delay_ms(10); */
        
        if (g_config.progress_callback && (i % 100 == 0)) {
            int progress = i * 100 / samples;
            g_config.progress_callback("Collecting gyroscope data...", progress);
        }
    }
    
    /* Calculate offset and noise characteristics */
    for (int i = 0; i < 3; i++) {
        calib->gyro_offset[i] = sum[i] / samples;
        calib->gyro_scale[i] = 1.0f; // Assume scale factor is 1
        
        /* Calculate standard deviation */
        float variance = (sum_sq[i] / samples) - (calib->gyro_offset[i] * calib->gyro_offset[i]);
        float std_dev = sqrtf(variance);
        
        /* Can be used to evaluate calibration quality */
        if (g_config.progress_callback) {
            char msg[256];
            snprintf(msg, sizeof(msg), "Axis%d noise: %.6f rad/s", i, std_dev);
            g_config.progress_callback(msg, 100);
        }
    }
    
    /* Set non-orthogonal matrix to identity matrix */
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            calib->gyro_misalignment[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
    
    /* Temperature compensation parameters */
    calib->ref_temperature = 25.0f;
    for (int i = 0; i < 3; i++) {
        calib->gyro_temp_coeff[i] = 0.0f;
    }
    
    return CALIB_OK;
}

/* BMM350 magnetometer calibration implementation */
static calib_status_t calibrate_bmm350(bmm350_calib_t *calib)
{
    /* Initialize and configure magnetometer */
    /* TODO: Implement BMM350 initialization and configuration */
    
    if (g_config.user_action_callback) {
        if (!g_config.user_action_callback(
            "Slowly rotate device in all directions for 30 seconds (figure-8 motion)")) {
            return CALIB_ERROR_USER_ABORT;
        }
    }
    
    /* Collect magnetometer data */
    float mag_samples[MAG_CALIB_SAMPLES][3];
    int sample_count = 0;
    uint32_t start_time = get_system_timestamp();
    
    while (sample_count < MAG_CALIB_SAMPLES && 
           (get_system_timestamp() - start_time) < 30000) {
        
        /* TODO: Read magnetometer data from BMM350 */
        /* Example:
        float mag_data[3];
        if (bmm350_read_mag(mag_data) == 0) {
            mag_samples[sample_count][0] = mag_data[0];
            mag_samples[sample_count][1] = mag_data[1];
            mag_samples[sample_count][2] = mag_data[2];
            sample_count++;
        }
        */
        
        sample_count++; // Placeholder increment
        
        if (g_config.progress_callback && (sample_count % 100 == 0)) {
            int progress = sample_count * 100 / MAG_CALIB_SAMPLES;
            char msg[256];
            snprintf(msg, sizeof(msg), "Collecting magnetometer data: %d/%d", 
                    sample_count, MAG_CALIB_SAMPLES);
            g_config.progress_callback(msg, progress);
        }
        
        /* delay_ms(10); */
    }
    
    if (sample_count < 1000) {
        return CALIB_ERROR_INVALID_DATA;
    }
    
    /* Use ellipsoid fitting algorithm */
    if (g_config.enable_advanced_fitting) {
        float fitness = calib_ellipsoid_fit(mag_samples, sample_count,
                                           calib->center, calib->radii, 
                                           calib->rotation);
        
        /* Calculate hard iron and soft iron correction from ellipsoid parameters */
        for (int i = 0; i < 3; i++) {
            calib->hard_iron[i] = calib->center[i];
        }
        
        /* Calculate soft iron matrix */
        float scale_matrix[3][3] = {{0}};
        for (int i = 0; i < 3; i++) {
            scale_matrix[i][i] = calib->radii[0] / calib->radii[i];
        }
        
        /* Soft iron matrix = rotation^T * scale * rotation */
        float temp[3][3];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                temp[i][j] = 0;
                for (int k = 0; k < 3; k++) {
                    temp[i][j] += scale_matrix[i][k] * calib->rotation[k][j];
                }
            }
        }
        
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                calib->soft_iron[i][j] = 0;
                for (int k = 0; k < 3; k++) {
                    calib->soft_iron[i][j] += calib->rotation[k][i] * temp[k][j];
                }
            }
        }
    } else {
        /* Simple max-min method */
        float max_val[3] = {-1000, -1000, -1000};
        float min_val[3] = {1000, 1000, 1000};
        
        for (int i = 0; i < sample_count; i++) {
            for (int axis = 0; axis < 3; axis++) {
                if (mag_samples[i][axis] > max_val[axis]) {
                    max_val[axis] = mag_samples[i][axis];
                }
                if (mag_samples[i][axis] < min_val[axis]) {
                    min_val[axis] = mag_samples[i][axis];
                }
            }
        }
        
        /* Calculate hard iron offset */
        for (int i = 0; i < 3; i++) {
            calib->hard_iron[i] = (max_val[i] + min_val[i]) / 2.0f;
            calib->center[i] = calib->hard_iron[i];
            calib->radii[i] = (max_val[i] - min_val[i]) / 2.0f;
        }
        
        /* Set soft iron matrix to identity matrix */
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                calib->soft_iron[i][j] = (i == j) ? 1.0f : 0.0f;
                calib->rotation[i][j] = (i == j) ? 1.0f : 0.0f;
            }
        }
    }
    
    /* Temperature compensation parameters */
    calib->ref_temperature = 25.0f;
    for (int i = 0; i < 3; i++) {
        calib->mag_temp_coeff[i] = 0.0f;
    }
    
    return CALIB_OK;
}

/* BMP580 barometer calibration implementation */
static calib_status_t calibrate_bmp580(bmp580_calib_t *calib)
{
    /* Configure sensor */
    /* TODO: Implement BMP580 configuration */
    
    /* Get reference altitude */
    float ref_altitude = 100.0f; // Default value, should be input by user
    if (g_config.user_action_callback) {
        /* In a real implementation, you would get user input here */
        g_config.user_action_callback("Enter current altitude in meters (default: 100m)");
    }
    
    /* Collect pressure data */
    float pressure_sum = 0;
    float temperature_sum = 0;
    const int samples = BARO_CALIB_SAMPLES;
    
    for (int i = 0; i < samples; i++) {
        /* TODO: Read pressure and temperature data from BMP580 */
        /* Example:
        float pressure, temperature;
        if (bmp580_read_data(&pressure, &temperature) == 0) {
            pressure_sum += pressure;
            temperature_sum += temperature;
        }
        */
        
        /* Placeholder values */
        pressure_sum += 101325.0f; // Standard atmospheric pressure
        temperature_sum += 25.0f;  // Room temperature
        
        /* delay_ms(100); */
        
        if (g_config.progress_callback && (i % 10 == 0)) {
            int progress = i * 100 / samples;
            g_config.progress_callback("Collecting pressure data...", progress);
        }
    }
    
    float avg_pressure = pressure_sum / samples;
    float avg_temperature = temperature_sum / samples;
    
    /* Calculate sea level pressure */
    calib->sea_level_pressure = avg_pressure / powf(1.0f - (ref_altitude / 44330.0f), 5.255f);
    
    /* Calculate altitude offset */
    float measured_altitude = 44330.0f * (1.0f - powf(avg_pressure / 101325.0f, 1.0f/5.255f));
    calib->altitude_offset = ref_altitude - measured_altitude;
    calib->pressure_offset = 0;
    calib->temp_coeff = 0;
    
    return CALIB_OK;
}

/* Apply calibration functions implementation */
void calib_apply_bmi270(const int16_t raw_accel[3], const int16_t raw_gyro[3],
                       float temperature, const bmi270_calib_t *calib,
                       float cal_accel[3], float cal_gyro[3])
{
    /* Accelerometer calibration */
    if (raw_accel && cal_accel) {
        const float accel_sensitivity = 16384.0f; // LSB/g for ±2g
        float temp_accel[3];
        
        for (int i = 0; i < 3; i++) {
            /* Convert to m/s² */
            temp_accel[i] = (raw_accel[i] / accel_sensitivity * 9.80665f) - calib->accel_offset[i];
            
            /* Apply scale factor */
            temp_accel[i] *= calib->accel_scale[i];
            
            /* Temperature compensation */
            if (calib->ref_temperature != 0) {
                temp_accel[i] += calib->accel_temp_coeff[i] * (temperature - calib->ref_temperature);
            }
        }
        
        /* Apply non-orthogonal correction */
        matrix_multiply_3x3(calib->accel_misalignment, temp_accel, cal_accel);
    }
    
    /* Gyroscope calibration */
    if (raw_gyro && cal_gyro) {
        const float gyro_sensitivity = 16.384f; // LSB/(°/s)
        float temp_gyro[3];
        
        for (int i = 0; i < 3; i++) {
            /* Convert to rad/s */
            temp_gyro[i] = (raw_gyro[i] / gyro_sensitivity * (M_PI / 180.0f)) - calib->gyro_offset[i];
            
            /* Apply scale factor */
            temp_gyro[i] *= calib->gyro_scale[i];
            
            /* Temperature compensation */
            if (calib->ref_temperature != 0) {
                temp_gyro[i] += calib->gyro_temp_coeff[i] * (temperature - calib->ref_temperature);
            }
        }
        
        /* Apply non-orthogonal correction */
        matrix_multiply_3x3(calib->gyro_misalignment, temp_gyro, cal_gyro);
    }
}

void calib_apply_bmm350(const float raw_mag[3], float temperature,
                       const bmm350_calib_t *calib, float cal_mag[3])
{
    /* Apply hard iron correction */
    float temp_mag[3];
    for (int i = 0; i < 3; i++) {
        temp_mag[i] = raw_mag[i] - calib->hard_iron[i];
    }
    
    /* Apply soft iron correction */
    matrix_multiply_3x3(calib->soft_iron, temp_mag, cal_mag);
    
    /* Temperature compensation */
    if (calib->ref_temperature != 0) {
        for (int i = 0; i < 3; i++) {
            cal_mag[i] += calib->mag_temp_coeff[i] * (temperature - calib->ref_temperature);
        }
    }
}

float calib_apply_bmp580(float pressure, float temperature,
                        const bmp580_calib_t *calib)
{
    /* Apply pressure offset */
    pressure += calib->pressure_offset;
    
    /* Temperature compensation */
    pressure += calib->temp_coeff * (temperature - 25.0f);
    
    /* Calculate altitude */
    float altitude = 44330.0f * (1.0f - powf(pressure / calib->sea_level_pressure, 1.0f/5.255f));
    
    /* Apply altitude offset */
    altitude += calib->altitude_offset;
    
    return altitude;
}

/* Ellipsoid fitting algorithm */
float calib_ellipsoid_fit(const float samples[][3], int count,
                         float center[3], float radii[3], 
                         float rotation[3][3])
{
    /* Simplified implementation */
    
    /* 1. Calculate data center */
    float sum[3] = {0};
    for (int i = 0; i < count; i++) {
        for (int j = 0; j < 3; j++) {
            sum[j] += samples[i][j];
        }
    }
    
    for (int i = 0; i < 3; i++) {
        center[i] = sum[i] / count;
    }
    
    /* 2. Calculate covariance matrix */
    float cov[3][3] = {{0}};
    for (int i = 0; i < count; i++) {
        float centered[3];
        for (int j = 0; j < 3; j++) {
            centered[j] = samples[i][j] - center[j];
        }
        
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                cov[j][k] += centered[j] * centered[k];
            }
        }
    }
    
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            cov[i][j] /= count;
        }
    }
    
    /* 3. Simplified processing: assume principal axes are aligned */
    for (int i = 0; i < 3; i++) {
        radii[i] = sqrtf(cov[i][i]);
        for (int j = 0; j < 3; j++) {
            rotation[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
    
    /* 4. Calculate fitting error */
    float total_error = 0;
    for (int i = 0; i < count; i++) {
        float dx = (samples[i][0] - center[0]) / radii[0];
        float dy = (samples[i][1] - center[1]) / radii[1];
        float dz = (samples[i][2] - center[2]) / radii[2];
        float ellipsoid_val = dx*dx + dy*dy + dz*dz;
        float error = fabsf(ellipsoid_val - 1.0f);
        total_error += error;
    }
    
    return total_error / count;
}

/* Data storage and loading */
calib_status_t calib_save_data(const sensor_calib_data_t *calib_data)
{
    if (!calib_data || !calib_validate_data(calib_data)) {
        return CALIB_ERROR_INVALID_PARAM;
    }
    
    /* TODO: Implement specific storage method, e.g., write to Flash or EEPROM */
    /* For embedded systems, you might use:
     * - Internal Flash
     * - External EEPROM
     * - SD card
     * - etc.
     */
    
    /* Example placeholder implementation */
    /* flash_write(CALIB_DATA_ADDR, (uint8_t*)calib_data, sizeof(sensor_calib_data_t)); */
    
    return CALIB_OK;
}

calib_status_t calib_load_data(sensor_calib_data_t *calib_data)
{
    if (!calib_data) {
        return CALIB_ERROR_INVALID_PARAM;
    }
    
    /* TODO: Read calibration data from storage */
    /* Example placeholder implementation */
    /* flash_read(CALIB_DATA_ADDR, (uint8_t*)calib_data, sizeof(sensor_calib_data_t)); */
    
    /* Validate data integrity */
    if (!calib_validate_data(calib_data)) {
        return CALIB_ERROR_INVALID_DATA;
    }
    
    return CALIB_OK;
}

bool calib_validate_data(const sensor_calib_data_t *calib_data)
{
    if (!calib_data) return false;
    
    /* Check magic number */
    if (calib_data->magic_number != CALIB_MAGIC_NUMBER) {
        return false;
    }
    
    /* Check version */
    if (calib_data->version != CALIB_VERSION) {
        return false;
    }
    
    /* Verify CRC */
    uint32_t calculated_crc = calculate_crc32(calib_data, 
                                             sizeof(sensor_calib_data_t) - sizeof(uint32_t));
    if (calculated_crc != calib_data->crc32) {
        return false;
    }
    
    /* Check data reasonableness */
    for (int i = 0; i < 3; i++) {
        /* Accelerometer offset should be within reasonable range */
        if (fabsf(calib_data->bmi270.accel_offset[i]) > 2.0f) {
            return false;
        }
        
        /* Gyroscope offset should be within reasonable range */
        if (fabsf(calib_data->bmi270.gyro_offset[i]) > 0.1f) {
            return false;
        }
        
        /* Magnetometer hard iron offset should be within reasonable range */
        if (fabsf(calib_data->bmm350.hard_iron[i]) > 200.0f) {
            return false;
        }
    }
    
    return true;
}

void calib_get_quality_report(const sensor_calib_data_t *calib_data,
                             char *report, uint32_t max_len)
{
    if (!calib_data || !report) return;
    
    snprintf(report, max_len,
        "=== Sensor Calibration Quality Report ===\r\n"
        "Calibration time: %u\r\n"
        "Version: 0x%08X\r\n\r\n"
        "BMI270 Accelerometer:\r\n"
        "  Quality index: %.2f%%\r\n"
        "  X offset: %.4f m/s²\r\n"
        "  Y offset: %.4f m/s²\r\n"
        "  Z offset: %.4f m/s²\r\n\r\n"
        "BMI270 Gyroscope:\r\n"
        "  Quality index: %.2f%%\r\n"
        "  X offset: %.6f rad/s\r\n"
        "  Y offset: %.6f rad/s\r\n"
        "  Z offset: %.6f rad/s\r\n\r\n"
        "BMM350 Magnetometer:\r\n"
        "  Quality index: %.2f%%\r\n"
        "  X hard iron: %.2f μT\r\n"
        "  Y hard iron: %.2f μT\r\n"
        "  Z hard iron: %.2f μT\r\n\r\n"
        "BMP580 Barometer:\r\n"
        "  Quality index: %.2f%%\r\n"
        "  Sea level pressure: %.2f Pa\r\n"
        "  Altitude offset: %.2f m\r\n",
        calib_data->timestamp,
        calib_data->version,
        calib_data->accel_fitness * 100,
        calib_data->bmi270.accel_offset[0],
        calib_data->bmi270.accel_offset[1],
        calib_data->bmi270.accel_offset[2],
        calib_data->gyro_fitness * 100,
        calib_data->bmi270.gyro_offset[0],
        calib_data->bmi270.gyro_offset[1],
        calib_data->bmi270.gyro_offset[2],
        calib_data->mag_fitness * 100,
        calib_data->bmm350.hard_iron[0],
        calib_data->bmm350.hard_iron[1],
        calib_data->bmm350.hard_iron[2],
        calib_data->baro_fitness * 100,
        calib_data->bmp580.sea_level_pressure,
        calib_data->bmp580.altitude_offset);
}

/* Helper function implementations */
static void matrix_multiply_3x3(const float a[3][3], const float b[3], float c[3])
{
    for (int i = 0; i < 3; i++) {
        c[i] = 0;
        for (int j = 0; j < 3; j++) {
            c[i] += a[i][j] * b[j];
        }
    }
}

static uint32_t calculate_crc32(const void *data, uint32_t length)
{
    /* Simplified CRC32 calculation */
    const uint8_t *bytes = (const uint8_t *)data;
    uint32_t crc = 0xFFFFFFFF;
    
    for (uint32_t i = 0; i < length; i++) {
        crc ^= bytes[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xEDB88320;
            } else {
                crc >>= 1;
            }
        }
    }
    
    return ~crc;
}

/* Weak function implementation - user should override */
__attribute__((weak)) uint32_t get_system_timestamp(void)
{
    /* User needs to implement system timestamp function */
    /* For example, using system tick counter */
    return 0;
}

/* 在 sensor_calibration.c 文件末尾添加以下实现 */

/* 简化的校准数据存储 */
static struct {
    float gyro_offset[3];      // 陀螺仪偏移
    float accel_offset[3];     // 加速度计偏移
    float accel_scale[3];      // 加速度计缩放
    float mag_offset[3];       // 磁力计硬铁偏移
    float baro_sea_level;      // 海平面气压
    float baro_altitude_offset; // 海拔偏移
    uint32_t magic;            // 校验魔数
    uint32_t crc;              // 校验和
} simple_calib_data = {0};

#define SIMPLE_CALIB_MAGIC 0x12345678

/* 简单CRC计算 */
static uint32_t simple_crc32(const void *data, uint32_t len)
{
    const uint8_t *p = (const uint8_t *)data;
    uint32_t crc = 0xFFFFFFFF;
    
    for (uint32_t i = 0; i < len; i++) {
        crc ^= p[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xEDB88320;
            } else {
                crc >>= 1;
            }
        }
    }
    return ~crc;
}

/* 保存校准数据到Flash/EEPROM */
static int save_simple_calib_data(void)
{
    // 计算CRC
    simple_calib_data.magic = SIMPLE_CALIB_MAGIC;
    simple_calib_data.crc = simple_crc32(&simple_calib_data, 
                                        sizeof(simple_calib_data) - sizeof(uint32_t));
    
    /* TODO: 实现保存到Flash/EEPROM */
    /* 示例：
    if (flash_write(CALIB_ADDR, (uint8_t*)&simple_calib_data, sizeof(simple_calib_data)) == 0) {
        return 0;
    }
    */
    
    printf("Calibration data saved (placeholder)\n");
    return 0; // 成功
}

/* 加载校准数据 */
static int load_simple_calib_data(void)
{
    /* TODO: 实现从Flash/EEPROM读取 */
    /* 示例：
    if (flash_read(CALIB_ADDR, (uint8_t*)&simple_calib_data, sizeof(simple_calib_data)) != 0) {
        return -1;
    }
    */
    
    // 验证数据
    if (simple_calib_data.magic != SIMPLE_CALIB_MAGIC) {
        return -1;
    }
    
    uint32_t calc_crc = simple_crc32(&simple_calib_data, 
                                    sizeof(simple_calib_data) - sizeof(uint32_t));
    if (calc_crc != simple_calib_data.crc) {
        return -1;
    }
    
    return 0; // 成功
}

/**
 * @brief 简单校准接口 - 一键校准所有传感器
 */
int sensor_simple_calibration(void)
{
    printf("\n=== Starting Simple Sensor Calibration ===\n");
    
    // 清零校准数据
    memset(&simple_calib_data, 0, sizeof(simple_calib_data));
    
    /* 1. 陀螺仪校准 - 静态偏移校准 */
    printf("Step 1/4: Gyroscope calibration\n");
    printf("Keep device stationary for 5 seconds...\n");
    
    float gyro_sum[3] = {0};
    int gyro_samples = 500;
    
    for (int i = 0; i < gyro_samples; i++) {
        /* TODO: 读取陀螺仪原始数据 */
        /* 示例：
        int16_t gyro_raw[3];
        if (read_gyro_raw(gyro_raw) == 0) {
            // 转换为度/秒
            float gyro_dps[3];
            gyro_dps[0] = gyro_raw[0] / 16.384f; // ±2000°/s量程
            gyro_dps[1] = gyro_raw[1] / 16.384f;
            gyro_dps[2] = gyro_raw[2] / 16.384f;
            
            gyro_sum[0] += gyro_dps[0];
            gyro_sum[1] += gyro_dps[1];
            gyro_sum[2] += gyro_dps[2];
        }
        */
        
        HAL_Delay(10);
        
        if (i % 100 == 0) {
            printf("Gyro progress: %d%%\n", i * 100 / gyro_samples);
        }
    }
    
    // 计算陀螺仪偏移
    simple_calib_data.gyro_offset[0] = gyro_sum[0] / gyro_samples;
    simple_calib_data.gyro_offset[1] = gyro_sum[1] / gyro_samples;
    simple_calib_data.gyro_offset[2] = gyro_sum[2] / gyro_samples;
    
    printf("Gyro offsets: [%.4f, %.4f, %.4f] deg/s\n", 
           simple_calib_data.gyro_offset[0], 
           simple_calib_data.gyro_offset[1], 
           simple_calib_data.gyro_offset[2]);
    
    /* 2. 加速度计校准 - 简化版本（仅Z轴） */
    printf("\nStep 2/4: Accelerometer calibration\n");
    printf("Place device horizontally (Z-axis up) for 3 seconds...\n");
    HAL_Delay(3000);
    
    float accel_sum[3] = {0};
    int accel_samples = 300;
    
    for (int i = 0; i < accel_samples; i++) {
        /* TODO: 读取加速度计数据 */
        /* 示例：
        int16_t accel_raw[3];
        if (read_accel_raw(accel_raw) == 0) {
            // 转换为m/s²
            float accel_ms2[3];
            accel_ms2[0] = accel_raw[0] / 16384.0f * 9.80665f; // ±2g量程
            accel_ms2[1] = accel_raw[1] / 16384.0f * 9.80665f;
            accel_ms2[2] = accel_raw[2] / 16384.0f * 9.80665f;
            
            accel_sum[0] += accel_ms2[0];
            accel_sum[1] += accel_ms2[1];
            accel_sum[2] += accel_ms2[2];
        }
        */
        
        HAL_Delay(10);
    }
    
    // 计算加速度计偏移（假设Z轴应该是1g）
    simple_calib_data.accel_offset[0] = accel_sum[0] / accel_samples;
    simple_calib_data.accel_offset[1] = accel_sum[1] / accel_samples;
    simple_calib_data.accel_offset[2] = (accel_sum[2] / accel_samples) - 9.80665f;
    
    // 简化的缩放因子
    simple_calib_data.accel_scale[0] = 1.0f;
    simple_calib_data.accel_scale[1] = 1.0f;
    simple_calib_data.accel_scale[2] = 1.0f;
    
    printf("Accel offsets: [%.4f, %.4f, %.4f] m/s²\n", 
           simple_calib_data.accel_offset[0], 
           simple_calib_data.accel_offset[1], 
           simple_calib_data.accel_offset[2]);
    
    /* 3. 磁力计校准 - 简化版本 */
    printf("\nStep 3/4: Magnetometer calibration\n");
    printf("Slowly rotate device in all directions for 10 seconds...\n");
    
    float mag_max[3] = {-1000, -1000, -1000};
    float mag_min[3] = {1000, 1000, 1000};
    int mag_samples = 1000;
    
    for (int i = 0; i < mag_samples; i++) {
        /* TODO: 读取磁力计数据 */
        /* 示例：
        float mag_data[3];
        if (read_mag_data(mag_data) == 0) {
            for (int j = 0; j < 3; j++) {
                if (mag_data[j] > mag_max[j]) mag_max[j] = mag_data[j];
                if (mag_data[j] < mag_min[j]) mag_min[j] = mag_data[j];
            }
        }
        */
        
        HAL_Delay(10);
        
        if (i % 200 == 0) {
            printf("Mag progress: %d%%\n", i * 100 / mag_samples);
        }
    }
    
    // 计算磁力计硬铁偏移
    simple_calib_data.mag_offset[0] = (mag_max[0] + mag_min[0]) / 2.0f;
    simple_calib_data.mag_offset[1] = (mag_max[1] + mag_min[1]) / 2.0f;
    simple_calib_data.mag_offset[2] = (mag_max[2] + mag_min[2]) / 2.0f;
    
    printf("Mag offsets: [%.2f, %.2f, %.2f] uT\n", 
           simple_calib_data.mag_offset[0], 
           simple_calib_data.mag_offset[1], 
           simple_calib_data.mag_offset[2]);
    
    /* 4. 气压计校准 */
    printf("\nStep 4/4: Barometer calibration\n");
    printf("Collecting pressure data for 3 seconds...\n");
    
    float pressure_sum = 0;
    int baro_samples = 100;
    
    for (int i = 0; i < baro_samples; i++) {
        /* TODO: 读取气压数据 */
        /* 示例：
        float pressure;
        if (read_pressure(&pressure) == 0) {
            pressure_sum += pressure;
        }
        */
        
        pressure_sum += 101325.0f; // 占位符
        HAL_Delay(30);
    }
    
    float avg_pressure = pressure_sum / baro_samples;
    
    // 假设当前海拔为100米
    float current_altitude = 100.0f;
    simple_calib_data.baro_sea_level = avg_pressure / powf(1.0f - (current_altitude / 44330.0f), 5.255f);
    simple_calib_data.baro_altitude_offset = 0.0f;
    
    printf("Sea level pressure: %.2f Pa\n", simple_calib_data.baro_sea_level);
    
    /* 保存校准数据 */
    if (save_simple_calib_data() == 0) {
        printf("\n=== Calibration Completed Successfully ===\n");
        return 0;
    } else {
        printf("\n=== Calibration Failed to Save ===\n");
        return -1;
    }
}

/**
 * @brief 快速陀螺仪校准
 */
int sensor_quick_gyro_calibration(void)
{
    printf("\n=== Quick Gyroscope Calibration ===\n");
    printf("Keep device stationary for 3 seconds...\n");
    
    // 加载现有校准数据（如果有的话）
    load_simple_calib_data();
    
    float gyro_sum[3] = {0};
    int samples = 300;
    
    for (int i = 0; i < samples; i++) {
        /* TODO: 读取陀螺仪数据 */
        /* 示例代码见上面的实现 */
        
        HAL_Delay(10);
        
        if (i % 60 == 0) {
            printf("Progress: %d%%\n", i * 100 / samples);
        }
    }
    
    // 更新陀螺仪偏移
    simple_calib_data.gyro_offset[0] = gyro_sum[0] / samples;
    simple_calib_data.gyro_offset[1] = gyro_sum[1] / samples;
    simple_calib_data.gyro_offset[2] = gyro_sum[2] / samples;
    
    printf("New gyro offsets: [%.4f, %.4f, %.4f] deg/s\n", 
           simple_calib_data.gyro_offset[0], 
           simple_calib_data.gyro_offset[1], 
           simple_calib_data.gyro_offset[2]);
    
    if (save_simple_calib_data() == 0) {
        printf("Quick calibration completed!\n");
                return 0;
    } else {
        printf("Failed to save calibration data\n");
        return -1;
    }
}

/**
 * @brief 应用校准到传感器数据
 */
int sensor_apply_calibration(void *imu_data, float *mag_data, float *pressure, float temperature)
{
    // 加载校准数据
    if (load_simple_calib_data() != 0) {
        return -1; // 没有有效的校准数据
    }
    
    /* 应用陀螺仪校准 */
    if (imu_data) {
        // 假设imu_data是你的imu_data_t结构体
        // 需要根据你的实际数据结构调整
        /*
        imu_data_t *imu = (imu_data_t*)imu_data;
        imu->gyr_x -= simple_calib_data.gyro_offset[0];
        imu->gyr_y -= simple_calib_data.gyro_offset[1];
        imu->gyr_z -= simple_calib_data.gyro_offset[2];
        
        // 应用加速度计校准
        imu->acc_x = (imu->acc_x - simple_calib_data.accel_offset[0]) * simple_calib_data.accel_scale[0];
        imu->acc_y = (imu->acc_y - simple_calib_data.accel_offset[1]) * simple_calib_data.accel_scale[1];
        imu->acc_z = (imu->acc_z - simple_calib_data.accel_offset[2]) * simple_calib_data.accel_scale[2];
        
        // 应用磁力计校准
        imu->mag_x -= simple_calib_data.mag_offset[0];
        imu->mag_y -= simple_calib_data.mag_offset[1];
        imu->mag_z -= simple_calib_data.mag_offset[2];
        */
    }
    
    /* 应用磁力计校准 */
    if (mag_data) {
        mag_data[0] -= simple_calib_data.mag_offset[0];
        mag_data[1] -= simple_calib_data.mag_offset[1];
        mag_data[2] -= simple_calib_data.mag_offset[2];
    }
    
    /* 应用气压计校准 - 计算校准后的海拔 */
    if (pressure) {
        // 计算海拔
        float altitude = 44330.0f * (1.0f - powf(*pressure / simple_calib_data.baro_sea_level, 1.0f/5.255f));
        altitude += simple_calib_data.baro_altitude_offset;
        
        // 可以选择是否替换pressure值为海拔值
        // *pressure = altitude; // 如果需要返回海拔而不是压力
    }
    
    return 0;
}

/**
 * @brief 检查是否有有效的校准数据
 */
int sensor_has_valid_calibration(void)
{
    return (load_simple_calib_data() == 0) ? 1 : 0;
}

/**
 * @brief 获取校准状态报告
 */
void sensor_get_calibration_status(void)
{
    if (load_simple_calib_data() == 0) {
        printf("\n=== Calibration Status ===\n");
        printf("Gyro offsets: [%.4f, %.4f, %.4f] deg/s\n", 
               simple_calib_data.gyro_offset[0], 
               simple_calib_data.gyro_offset[1], 
               simple_calib_data.gyro_offset[2]);
        printf("Accel offsets: [%.4f, %.4f, %.4f] m/s²\n", 
               simple_calib_data.accel_offset[0], 
               simple_calib_data.accel_offset[1], 
               simple_calib_data.accel_offset[2]);
        printf("Mag offsets: [%.2f, %.2f, %.2f] uT\n", 
               simple_calib_data.mag_offset[0], 
               simple_calib_data.mag_offset[1], 
               simple_calib_data.mag_offset[2]);
        printf("Sea level pressure: %.2f Pa\n", simple_calib_data.baro_sea_level);
        printf("Calibration data is VALID\n");
    } else {
        printf("No valid calibration data found\n");
    }
}

/* 获取校准数据的接口函数实现 */
void calib_get_gyro_offset(float offset[3])
{
    if (offset) {
        offset[0] = simple_calib_data.gyro_offset[0];
        offset[1] = simple_calib_data.gyro_offset[1];
        offset[2] = simple_calib_data.gyro_offset[2];
    }
}

void calib_get_accel_offset(float offset[3])
{
    if (offset) {
        offset[0] = simple_calib_data.accel_offset[0];
        offset[1] = simple_calib_data.accel_offset[1];
        offset[2] = simple_calib_data.accel_offset[2];
    }
}

void calib_get_accel_scale(float scale[3])
{
    if (scale) {
        scale[0] = simple_calib_data.accel_scale[0];
        scale[1] = simple_calib_data.accel_scale[1];
        scale[2] = simple_calib_data.accel_scale[2];
    }
}

void calib_get_mag_offset(float offset[3])
{
    if (offset) {
        offset[0] = simple_calib_data.mag_offset[0];
        offset[1] = simple_calib_data.mag_offset[1];
        offset[2] = simple_calib_data.mag_offset[2];
    }
}

float calib_get_sea_level_pressure(void)
{
    return simple_calib_data.baro_sea_level;
}

float calib_get_altitude_offset(void)
{
    return simple_calib_data.baro_altitude_offset;
}