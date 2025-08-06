/**
 * Copyright (c) 2023 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef BMI270_BMM350_H
#define BMI270_BMM350_H

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************/
/*!                 Header Files                                              */
#include <stdint.h>
#include "bmi270.h"
#include "bmm350.h"

/******************************************************************************/
/*!                Macro definition                                           */

/*! Earth's gravity in m/s^2 */
#define GRAVITY_EARTH (9.80665f)

/******************************************************************************/
/*!                 Type Definitions                                          */

/*! Structure to hold IMU data */
typedef struct {
    /* Accelerometer data in m/s^2 */
    float acc_x;
    float acc_y;
    float acc_z;
    
    /* Gyroscope data in degrees/s */
    float gyr_x;
    float gyr_y;
    float gyr_z;
    
    /* Magnetometer data in uT */
    float mag_x;
    float mag_y;
    float mag_z;
    
    /* Raw sensor data */
    int16_t acc_raw_x;
    int16_t acc_raw_y;
    int16_t acc_raw_z;
    
    int16_t gyr_raw_x;
    int16_t gyr_raw_y;
    int16_t gyr_raw_z;
    
    /* Status register */
    uint8_t status;
} imu_data_t;

/******************************************************************************/
/*!                 Function Declarations                                     */

/*!
 *  @brief Initialize BMI270 and BMM350 sensors
 *
 *  @param[in] use_spi : 1 for SPI interface, 0 for I2C interface
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval < 0 -> Failure Info
 */
int8_t IMU_Init(uint8_t use_spi);

/*!
 *  @brief Read data from BMI270 and BMM350 sensors
 *
 *  @param[out] data : Pointer to structure to store IMU data
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval < 0 -> Failure Info
 */
int8_t IMU_ReadData(imu_data_t *data);

/*!
 *  @brief Deinitialize IMU sensors
 *
 *  @return void
 */
void IMU_Deinit(void);

/*!
 *  @brief This API is used to print the BMM350 execution status.
 *
 *  @param[in] api_name : Name of the API whose execution status has to be printed.
 *  @param[in] rslt     : Error code returned by the API whose execution status has to be printed.
 *
 *  @return void.
 */
void bmm350_error_codes_print_result(const char api_name[], int8_t rslt);

#ifdef __cplusplus
}
#endif

#endif /* BMI270_BMM350_H */