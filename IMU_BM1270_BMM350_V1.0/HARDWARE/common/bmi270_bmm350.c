/**
 * Copyright (c) 2023 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/******************************************************************************/
/*!                 Header Files                                              */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "bmi270_bmm350.h"
#include "delay.h"
#include "common.h"

/******************************************************************************/
/*!                 Static Variables                                          */

/* BMI270 device structure */
static struct bmi2_dev bmi_dev;

/* BMM350 device structure */
static struct bmm350_dev bmm_dev = { 0 };

/* Initialization flag */
static uint8_t is_initialized = 0;

/*******************************************************************************/
/*!                 Static Functions Declarations                              */

/*!
 *  @brief This function configures accelerometer and gyroscope
 *
 *  @param[in] bmi       : Structure instance of bmi2_dev
 *
 *  @return Status of execution
 */
static int8_t bmi2_accel_gyro_conf(struct bmi2_dev *bmi);

/*!
 *  @brief This function configures auxiliary sensor (bmm350)
 *
 *  @param[in] mag_odr     : ODR of magnetometer
 *  @param[in] bmm         : Structure instance of bmm350_dev
 *  @param[in] bmi         : Structure instance of bmi2_dev
 *
 *  @return Status of execution
 */
static int8_t bmi2_aux_conf(uint8_t mag_odr, struct bmm350_dev *bmm, struct bmi2_dev *bmi);

/*!
 *  @brief This function converts lsb to meter per second squared for 16 bit accelerometer at
 *  range 2G, 4G, 8G or 16G.
 *
 *  @param[in] val       : LSB from each axis.
 *  @param[in] g_range   : Gravity range.
 *  @param[in] bit_width : Resolution for accel.
 *
 *  @return Value in meter per second squared.
 */
static float lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width);

/*!
 *  @brief This function converts lsb to degree per second for 16 bit gyro at
 *  range 125, 250, 500, 1000 or 2000dps.
 *
 *  @param[in] val       : LSB from each axis.
 *  @param[in] dps       : Degree per second.
 *  @param[in] bit_width : Resolution for gyro.
 *
 *  @return Value in degree per second.
 */
static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width);

/**
 *  @brief - This function reads data from auxiliary sensor in data mode.
 */
static int8_t bmi2_aux_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);

/**
 *  @brief - This function writes data to the auxiliary sensor in data mode.
 */
static int8_t bmi2_aux_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);

/*!
 *  @brief This function provides the delay for required time (Microsecond)
 */
static void bmi2_aux_delay_us(uint32_t period, void *intf_ptr);

/*!
 *  @brief Function to setup Auxiliary interface (BMM350)
 */
static int8_t bmi2_aux_interface_init(struct bmm350_dev *bmm, struct bmi2_dev *bmi);

/*******************************************************************************/
/*!                             Function Definitions                           */

/*!
 *  @brief Initialize BMI270 and BMM350 sensors
 */
int8_t IMU_Init(uint8_t use_spi)
{
    /* Status of api are returned to this variable. */
    int8_t rslt;

    /* Variable to select the pull-up resistor which is set to trim register */
    uint8_t regdata;

    /* Accel, Gyro and Aux sensors are listed in array. */
    uint8_t sensor_list[3] = {BMI2_ACCEL, BMI2_GYRO, BMI2_AUX};

    /* 默认使用I2C接口 */
    printf("Initializing IMU with I2C Interface\n");
    
    /* Interface reference is given as a parameter
     * For I2C : BMI2_I2C_INTF
     * For SPI : BMI2_SPI_INTF
     */
    rslt = bmi2_interface_init(&bmi_dev, BMI2_I2C_INTF);
    
    if (rslt != BMI2_OK)
    {
        printf("Failed to initialize I2C interface\n");
        bmi2_error_codes_print_result(rslt);
        return rslt;
    }

    /* Interface initialization for auxiliary sensor (bmm350) */
    rslt = bmi2_aux_interface_init(&bmm_dev, &bmi_dev);
    if (rslt != BMI2_OK)
    {
        printf("Failed to initialize auxiliary interface\n");
        bmi2_error_codes_print_result(rslt);
        return rslt;
    }

    /* Initialize sensor */
    rslt = bmi270_init(&bmi_dev);
    if (rslt != BMI2_OK)
    {
        printf("Failed to initialize BMI270\n");
        bmi2_error_codes_print_result(rslt);
        return rslt;
    }

    /* Pull-up resistor 10k is set to the trim register */
    regdata = BMI2_ASDA_PUPSEL_10K;
    rslt = bmi2_set_regs(BMI2_AUX_IF_TRIM, &regdata, 1, &bmi_dev);
    if (rslt != BMI2_OK)
    {
        printf("Failed to set pull-up resistor\n");
        bmi2_error_codes_print_result(rslt);
        return rslt;
    }

    /* Configure accel and gyro */
    rslt = bmi2_accel_gyro_conf(&bmi_dev);
    if (rslt != BMI2_OK)
    {
        printf("Failed to configure accel and gyro\n");
        return rslt;
    }

    /* Configure Auxiliary sensor (bmm350)
     * Mag ODR to be given as first parameter
     */
    rslt = bmi2_aux_conf(BMI2_AUX_ODR_100HZ, &bmm_dev, &bmi_dev);
    if (rslt != BMI2_OK)
    {
        printf("Failed to configure auxiliary sensor\n");
        return rslt;
    }

    /* NOTE:
     * Accel, Gyro and Aux enable must be done after setting configurations
     */
    rslt = bmi2_sensor_enable(sensor_list, 3, &bmi_dev);
    if (rslt != BMI2_OK)
    {
        printf("Failed to enable sensors\n");
        bmi2_error_codes_print_result(rslt);
        return rslt;
    }

    /* Map data ready interrupt to interrupt pin. */
    rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT1, &bmi_dev);
    if (rslt != BMI2_OK)
    {
        printf("Failed to map data ready interrupt\n");
        bmi2_error_codes_print_result(rslt);
        return rslt;
    }

    /* Check if BMM350 is properly initialized */
    if (bmm_dev.chip_id != BMM350_CHIP_ID)
    {
        printf("\nInvalid BMM350 (Aux) sensor - Chip ID : 0x%x\n", bmm_dev.chip_id);
        return BMI2_E_DEV_NOT_FOUND;
    }

    is_initialized = 1;
    printf("\nIMU initialized successfully\n");
    printf("BMI270 and BMM350 (Chip ID: 0x%x) are ready\n", bmm_dev.chip_id);

    return BMI2_OK;
}

/*!
 *  @brief Read data from BMI270 and BMM350 sensors
 */
int8_t IMU_ReadData(imu_data_t *data)
{
    int8_t rslt;
    struct bmi2_sens_data sens_data = {{0}};
    struct bmm350_mag_temp_data mag_temp_data;

    //printf("IMU_ReadData: Starting...\n");

    if (!is_initialized)
    {
        //printf("IMU_ReadData: IMU not initialized\n");
        return BMI2_E_INVALID_STATUS;
    }

    if (data == NULL)
    {
        //printf("IMU_ReadData: Data pointer is NULL\n");
        return BMI2_E_NULL_PTR;
    }

    //printf("IMU_ReadData: Clearing data structure...\n");
    /* 清零数据结构 */
    memset(data, 0, sizeof(imu_data_t));

    //printf("IMU_ReadData: Calling bmi2_get_sensor_data...\n");
    /* Read sensor data */
    rslt = bmi2_get_sensor_data(&sens_data, &bmi_dev);
    //printf("IMU_ReadData: bmi2_get_sensor_data returned: %d\n", rslt);
    
    if (rslt != BMI2_OK)
    {
        //printf("IMU_ReadData: bmi2_get_sensor_data failed: %d\n", rslt);
        return rslt;
    }

    //printf("IMU_ReadData: Sensor status: 0x%02X\n", sens_data.status);
    /* 存储状态寄存器 */
    data->status = sens_data.status;

    /* 检查加速度计数据是否就绪 */
    if (sens_data.status & BMI2_DRDY_ACC)
    {
        //printf("IMU_ReadData: Processing accelerometer data...\n");
        /* Store raw accelerometer data */
        data->acc_raw_x = sens_data.acc.x;
        data->acc_raw_y = sens_data.acc.y;
        data->acc_raw_z = sens_data.acc.z;

        /* Converting lsb to meter per second squared for 16 bit accelerometer at 2G range. */
        data->acc_x = lsb_to_mps2(sens_data.acc.x, 2.0f, bmi_dev.resolution);
        data->acc_y = lsb_to_mps2(sens_data.acc.y, 2.0f, bmi_dev.resolution);
        data->acc_z = lsb_to_mps2(sens_data.acc.z, 2.0f, bmi_dev.resolution);
        //printf("IMU_ReadData: Accelerometer data processed\n");
    }
    else
    {
        //printf("IMU_ReadData: Accelerometer data not ready\n");
    }

    /* 检查陀螺仪数据是否就绪 */
    if (sens_data.status & BMI2_DRDY_GYR)
    {
        //printf("IMU_ReadData: Processing gyroscope data...\n");
        /* Store raw gyroscope data */
        data->gyr_raw_x = sens_data.gyr.x;
        data->gyr_raw_y = sens_data.gyr.y;
        data->gyr_raw_z = sens_data.gyr.z;

        /* Converting lsb to degree per second for 16 bit gyro at 2000dps range. */
        data->gyr_x = lsb_to_dps(sens_data.gyr.x, 2000.0f, bmi_dev.resolution);
        data->gyr_y = lsb_to_dps(sens_data.gyr.y, 2000.0f, bmi_dev.resolution);
        data->gyr_z = lsb_to_dps(sens_data.gyr.z, 2000.0f, bmi_dev.resolution);
        //printf("IMU_ReadData: Gyroscope data processed\n");
    }
    else
    {
        //printf("IMU_ReadData: Gyroscope data not ready\n");
    }

    //printf("IMU_ReadData: Checking magnetometer...\n");
    /* 尝试读取磁力计数据，但不让它影响整体返回结果 */
    if (bmm_dev.chip_id == BMM350_CHIP_ID)
    {
        //printf("IMU_ReadData: Reading magnetometer data...\n");
        int8_t mag_rslt = bmm350_get_compensated_mag_xyz_temp_data(&mag_temp_data, &bmm_dev);
        //printf("IMU_ReadData: Magnetometer read returned: %d\n", mag_rslt);
        
        if (mag_rslt == BMM350_OK)
        {
            data->mag_x = mag_temp_data.x;
            data->mag_y = mag_temp_data.y;
            data->mag_z = mag_temp_data.z;
            //printf("IMU_ReadData: Magnetometer data processed\n");
        }
        else
        {
            /* 磁力计读取失败，但不影响整体结果 */
            //printf("IMU_ReadData: Magnetometer read failed: %d\n", mag_rslt);
            data->mag_x = 0.0f;
            data->mag_y = 0.0f;
            data->mag_z = 0.0f;
        }
    }
    else
    {
        //printf("IMU_ReadData: BMM350 not available (chip_id: 0x%02X)\n", bmm_dev.chip_id);
    }

    //printf("IMU_ReadData: Checking return conditions...\n");
    /* 只要有加速度计或陀螺仪数据就认为成功 */
    if ((sens_data.status & BMI2_DRDY_ACC) || (sens_data.status & BMI2_DRDY_GYR))
    {
        //printf("IMU_ReadData: Success - returning BMI2_OK\n");
        return BMI2_OK;
    }
    else
    {
        //printf("IMU_ReadData: No sensor data ready. Status: 0x%02X\n", sens_data.status);
        return BMI2_W_FIFO_EMPTY;
    }
}

/*!
 *  @brief Deinitialize IMU sensors
 */
void IMU_Deinit(void)
{
    is_initialized = 0;
    //bmi2_coines_deinit();
}
/*******************************************************************************/
/*!               Static Functions Definitions                                */

/*!
 * @brief This function configures accelerometer and gyroscope
 */
static int8_t bmi2_accel_gyro_conf(struct bmi2_dev *bmi)
{
    int8_t rslt;
    struct bmi2_sens_config config[2];

    config[0].type = BMI2_ACCEL;
    config[1].type = BMI2_GYRO;

    /* Get default configurations for the type of feature selected. */
    rslt = bmi2_get_sensor_config(config, 2, bmi);
    if (rslt != BMI2_OK)
    {
        bmi2_error_codes_print_result(rslt);
        return rslt;
    }

    /* Configurations for accel. */
    config[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
    config[0].cfg.acc.bwp = BMI2_ACC_OSR2_AVG2;
    config[0].cfg.acc.odr = BMI2_ACC_ODR_100HZ;
    config[0].cfg.acc.range = BMI2_ACC_RANGE_2G;

    /* Configurations for gyro. */
    config[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
    config[1].cfg.gyr.noise_perf = BMI2_GYR_RANGE_2000;
    config[1].cfg.gyr.bwp = BMI2_GYR_OSR2_MODE;
    config[1].cfg.gyr.odr = BMI2_GYR_ODR_100HZ;
    config[1].cfg.gyr.range = BMI2_GYR_RANGE_2000;
    config[1].cfg.gyr.ois_range = BMI2_GYR_OIS_2000;

    /* Set new configurations for accel and gyro */
    rslt = bmi2_set_sensor_config(config, 2, bmi);
    if (rslt != BMI2_OK)
    {
        bmi2_error_codes_print_result(rslt);
    }

    return rslt;
}

/*!
 * @brief This function configures auxiliary sensor (bmm350)
 */
static int8_t bmi2_aux_conf(uint8_t mag_odr, struct bmm350_dev *bmm, struct bmi2_dev *bmi)
{
    int8_t rslt;
    struct bmi2_sens_config bmi2_conf;
    enum bmm350_data_rates bmm350_odr;
    uint8_t bmi2_sens = BMI2_AUX;

    switch (mag_odr)
    {
    case BMI2_AUX_ODR_0_78HZ:
    case BMI2_AUX_ODR_1_56HZ:
    case BMI2_AUX_ODR_3_12HZ:
    case BMI2_AUX_ODR_6_25HZ:
    case BMI2_AUX_ODR_12_5HZ:
        bmm350_odr = BMM350_ODR_12_5HZ;
        break;
    case BMI2_AUX_ODR_25HZ:
    case BMI2_AUX_ODR_50HZ:
        bmm350_odr = BMM350_DATA_RATE_50HZ;
        break;
    case BMI2_AUX_ODR_100HZ:
        bmm350_odr = BMM350_DATA_RATE_100HZ;
        break;
    case BMI2_AUX_ODR_200HZ:
        bmm350_odr = BMM350_DATA_RATE_200HZ;
        break;
    case BMI2_AUX_ODR_400HZ:
    case BMI2_AUX_ODR_800HZ:
        bmm350_odr = BMM350_DATA_RATE_400HZ;
        break;
    default:
        mag_odr = 0;
        printf("Error : Unsupported Auxiliary Magnetometer ODR : %u\n", mag_odr);
        return BMI2_E_INVALID_INPUT;
    }

    bmi2_conf.type = BMI2_AUX;
    bmi2_conf.cfg.aux.odr = mag_odr;
    bmi2_conf.cfg.aux.aux_en = BMI2_ENABLE;
    bmi2_conf.cfg.aux.i2c_device_addr = BMM350_I2C_ADSEL_SET_LOW;
    bmi2_conf.cfg.aux.fcu_write_en = BMI2_ENABLE;
    bmi2_conf.cfg.aux.manual_en = BMI2_ENABLE;
    bmi2_conf.cfg.aux.aux_rd_burst = BMI2_AUX_READ_LEN_3;
    bmi2_conf.cfg.aux.man_rd_burst = BMI2_AUX_READ_LEN_3;
    bmi2_conf.cfg.aux.read_addr = BMM350_REG_MAG_X_XLSB;

    rslt = bmi2_set_sensor_config(&bmi2_conf, 1, bmi);
    if (rslt != BMI2_OK)
    {
        bmi2_error_codes_print_result(rslt);
        return rslt;
    }

    if (mag_odr != 0) /* Proceed only if ODR is valid */
    {
        rslt = bmm350_init(bmm);
        if (rslt != BMM350_OK)
        {
            bmm350_error_codes_print_result("bmm350_init", rslt);
            return rslt;
        }

        /* Set ODR and performance */
        rslt = bmm350_set_odr_performance(bmm350_odr, BMM350_AVERAGING_4, bmm);
        if (rslt != BMM350_OK)
        {
            bmm350_error_codes_print_result("bmm350_set_odr_performance", rslt);
            return rslt;
        }

        /* Enable all axis */
        rslt = bmm350_enable_axes(BMM350_X_EN, BMM350_Y_EN, BMM350_Z_EN, bmm);
        if (rslt != BMM350_OK)
        {
            bmm350_error_codes_print_result("bmm350_enable_axes", rslt);
            return rslt;
        }

        rslt = bmm350_set_powermode(BMM350_NORMAL_MODE, bmm);
        if (rslt != BMM350_OK)
        {
            bmm350_error_codes_print_result("bmm350_set_powermode", rslt);
            return rslt;
        }
        
        bmi2_conf.cfg.aux.manual_en = BMI2_ENABLE;
    }
    else
    {
        bmi2_conf.cfg.aux.aux_en = BMI2_DISABLE;
        bmi2_conf.cfg.aux.fcu_write_en = BMI2_DISABLE;
        bmi2_conf.cfg.aux.manual_en = BMI2_ENABLE;

        rslt = bmi2_set_sensor_config(&bmi2_conf, 1, bmi);
        if (rslt != BMI2_OK)
        {
            bmi2_error_codes_print_result(rslt);
            return rslt;
        }

        rslt = bmi2_sensor_disable(&bmi2_sens, 1, bmi);
        if (rslt != BMI2_OK)
        {
            bmi2_error_codes_print_result(rslt);
        }
    }

    return rslt;
}

/*!
 * @brief This function converts lsb to meter per second squared for 16 bit accelerometer at
 * range 2G, 4G, 8G or 16G.
 */
static float lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width)
{
    double power = 2;
    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));
    return (GRAVITY_EARTH * val * g_range) / half_scale;
}

/*!
 * @brief This function converts lsb to degree per second for 16 bit gyro at
 * range 125, 250, 500, 1000 or 2000dps.
 */
static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width)
{
    double power = 2;
    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));
    return (dps / (half_scale)) * (val);
}

/*!
 * @brief This function reads the data from auxiliary sensor in data mode.
 */
static int8_t bmi2_aux_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    int8_t rslt;
    struct bmi2_dev *dev = (struct bmi2_dev *)intf_ptr;
    rslt = bmi2_read_aux_man_mode(reg_addr, reg_data, (uint16_t)length, dev);
    return rslt;
}

/*!
 * @brief This function writes the data to auxiliary sensor in data mode.
 */
static int8_t bmi2_aux_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    int8_t rslt;
    struct bmi2_dev *dev = (struct bmi2_dev *)intf_ptr;
    rslt = bmi2_write_aux_man_mode(reg_addr, reg_data, (uint16_t)length, dev);
    return rslt;
}

/*!
 * Delay function map to COINES platform
 */
static void bmi2_aux_delay_us(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;
    delay_us(period);
}

/*!
 *  @brief Function to setup Auxiliary interface (BMM350)
 */
static int8_t bmi2_aux_interface_init(struct bmm350_dev *bmm, struct bmi2_dev *bmi)
{
    int8_t rslt = BMI2_OK;

    if (bmm != NULL)
    {
        /* To enable the i2c interface settings for bmm350. */
        bmm->intf_ptr = bmi;
        bmm->read = bmi2_aux_i2c_read;
        bmm->write = bmi2_aux_i2c_write;
        bmm->delay_us = bmi2_aux_delay_us;

        /* As per datasheet, aux interface will support only for I2C */
        //bmm->intf = bmm350_I2C_INTF;
    }
    else
    {
        rslt = BMI2_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief Prints the execution status of the APIs.
 */
void bmm350_error_codes_print_result(const char api_name[], int8_t rslt)
{
    if (rslt != BMM350_OK)
    {
        printf("%s\t", api_name);

        switch (rslt)
        {
        case BMM350_E_NULL_PTR:
            printf("Error [%d] : Null pointer error.", rslt);
            printf(
                "It occurs when the user tries to assign value (not address) to a pointer, which has been initialized to NULL.\r\n");
            break;

        case BMM350_E_COM_FAIL:
            printf("Error [%d] : Communication failure error.", rslt);
            printf(
                "It occurs due to read/write operation failure and also due to power failure during communication\r\n");
            break;

        case BMM350_E_DEV_NOT_FOUND:
            printf("Error [%d] : Device not found error. It occurs when the device chip id is incorrectly read\r\n",
                   rslt);
            break;

        case BMM350_E_INVALID_CONFIG:
            printf("Error [%d] : Invalid sensor configuration.", rslt);
            printf(" It occurs when there is a mismatch in the requested feature with the available one\r\n");
            break;

        default:
            printf("Error [%d] : Unknown error code\r\n", rslt);
            break;
        }
    }
}

