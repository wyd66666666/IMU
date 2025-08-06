/**\
 * Copyright (c) 2023 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

/******************************************************************************/
/*!                 Header Files                                              */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "bmi270.h"
#include "bmm350.h"
#include "delay.h"
#include "common.h"
#include "usart_printf.h"

#include "imu_attitude.h"    // ������̬������ض���?
#include "anony_protocol.h"  // ��������Э����ض���?
/******************************************************************************/
/*!                Macro definition                                           */

/*! Macro define limit to print data */
#define SAMPLE_COUNT UINT8_C(100)

/*! Earth's gravity in m/s^2 */
#define GRAVITY_EARTH (9.80665f)

/*******************************************************************************/
/*!                 Static Functions Definitions                               */

/*!
 *  @brief This function configures accelerometer and gyroscope
 *
 *  @param[in] bmi       : Structure instance of bmi2_dev
 *
 *  @return None
 */
static void bmi2_accel_gyro_conf(struct bmi2_dev *bmi);

/*!
 *  @brief This function configures auxiliary sensor (bmm350)
 *
 *  @param[in] mag_odr     : ODR of magnetometer
 *  @param[in] bmm         : Structure instance of bmm350_dev
 *  @param[in] bmi         : Structure instance of bmi2_dev
 *
 *  @return None
 */
static void bmi2_aux_conf(uint8_t mag_odr, struct bmm350_dev *bmm, struct bmi2_dev *bmi);

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
 *
 *  @param[in] reg_addr     : Register address.
 *  @param[out] reg_data    : Aux data pointer to store the read data.
 *  @param[in] length       : No of bytes to read.
 *  @param[in] intf_ptr     : Interface pointer
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval < 0 -> Failure Info
 */
static int8_t bmi2_aux_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);

/**
 *  @brief - This function writes data to the auxiliary sensor in data mode.
 *
 *  @param[in] reg_addr     : Register address.
 *  @param[out] reg_data    : Aux data pointer to store the data being written.
 *  @param[in] length       : No of bytes to read.
 *  @param[in] intf_ptr     : Interface pointer
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval < 0 -> Failure Info
 */
static int8_t bmi2_aux_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);

/*!
 *  @brief This function provides the delay for required time (Microsecond) as per the input provided in some of the
 *  APIs.
 *
 *  @param[in] period       : The required wait time in microsecond.
 *  @param[in] intf_ptr     : Interface pointer
 *
 *  @return void.
 */
static void bmi2_aux_delay_us(uint32_t period, void *intf_ptr);

/*!
 *  @brief Function to setup Auxiliary interface (BMM350)
 *
 *  @param[in] bmm  : Structure instance of bmm350_dev
 *  @param[in] bmi  : Structure instance of bmi2_dev
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval < 0 -> Failure Info
 */
int8_t bmi2_aux_interface_init(struct bmm350_dev *bmm, struct bmi2_dev *bmi);

/*!
 *  @brief This API is used to print the execution status.
 *
 *  @param[in] api_name : Name of the API whose execution status has to be printed.
 *  @param[in] rslt     : Error code returned by the API whose execution status has to be printed.
 *
 *  @return void.
 */
void bmm350_error_codes_print_result(const char api_name[], int8_t rslt);

/*******************************************************************************/
/*!                             Function                                       */

/* This function starts the execution of program. */

//int IMU_INIT_GETDATA(void)
//{
//    /* Status of api are returned to this variable. */
//    int8_t rslt;
//    uint8_t retry_count = 0;
//    uint8_t init_retry = 0;
//    
//    /* Variable to select the pull-up resistor which is set to trim register */
//    uint8_t regdata;

//    /* Accel, Gyro and Aux sensors are listed in array. */
//    uint8_t sensor_list[3] = {BMI2_ACCEL, BMI2_GYRO, BMI2_AUX};

//    /* Sensor initialization configuration. */
//    struct bmm350_dev bmm = { 0 };
//    struct bmi2_dev bmi;

//    /* bmm350 magnetometer data */
//    struct bmm350_mag_temp_data mag_temp_data;
//    struct bmm350_raw_mag_data raw_data;

//    /* Structure to define type of sensor and their respective data. */
//    struct bmi2_sens_data sens_data = {{0}};

//    /* Variables to define read the accel and gyro data in float */
//    float acc_x = 0, acc_y = 0, acc_z = 0;
//    float gyr_x = 0, gyr_y = 0, gyr_z = 0;

//init_retry_label:
//    /* ��ʼ���������ӿڣ��������� */
//    rslt = bmi2_interface_init(&bmi, BMI2_I2C_INTF);
//    if (rslt != BMI2_OK) {
//        printf("�ӿڳ�ʼ��ʧ�ܣ�������: %d��������...\r\n", rslt);
//        HAL_Delay(100);
//        if (++init_retry < 3) goto init_retry_label;
//        return rslt;
//    }
//    
//    /* ��ʼ�������������ӿ� */
//    rslt = bmi2_aux_interface_init(&bmm, &bmi);
//    if (rslt != BMI2_OK) {
//        printf("�����ӿڳ�ʼ��ʧ�ܣ�������: %d\r\n", rslt);
//        HAL_Delay(100);
//        if (++init_retry < 3) goto init_retry_label;
//        return rslt;
//    }

//    /* ��ʼ��BMI270������ */
//    rslt = bmi270_init(&bmi);
//    if (rslt != BMI2_OK) {
//        printf("BMI270��ʼ��ʧ�ܣ�������: %d\r\n", rslt);
//        HAL_Delay(100);
//        if (++init_retry < 3) goto init_retry_label;
//        return rslt;
//    }

//    /* ������������ */
//    regdata = BMI2_ASDA_PUPSEL_10K;
//    rslt = bmi2_set_regs(BMI2_AUX_IF_TRIM, &regdata, 1, &bmi);
//    bmi2_error_codes_print_result(rslt);

//    /* ���ü��ٶȼƺ������� */
//    bmi2_accel_gyro_conf(&bmi);

//    /* ���ø��������� */
//    bmi2_aux_conf(BMI2_AUX_ODR_100HZ, &bmm, &bmi);

//    /* ʹ�ܴ����� */
//    rslt = bmi2_sensor_enable(sensor_list, 3, &bmi);
//    if (rslt != BMI2_OK) {
//        printf("������ʹ��ʧ�ܣ�������: %d\r\n", rslt);
//        HAL_Delay(100);
//        if (++init_retry < 3) goto init_retry_label;
//        return rslt;
//    }

//    /* ӳ�����ݾ����ж� */
//    rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT1, &bmi);
//    bmi2_error_codes_print_result(rslt);

//    printf("MAGNETOMETER, ACCEL, AND GYRO DATA IN DATA MODE\n");

//    if (bmm.chip_id == BMM350_CHIP_ID)
//    {
//        printf("\nValid BMM350 (Aux) sensor - Chip ID : 0x%x\n", bmm.chip_id);
//        
//        /* �����ݶ�ȡѭ�� - ���Ӵ���ָ�����?*/
//        for (;;)
//        {
//            retry_count = 0;
//            
//            /* ������ʱ�������߸��� */
//            HAL_Delay(10);
//            
//            /* ��ȡ���������� */
//            rslt = bmi2_get_sensor_data(&sens_data, &bmi);
//            
//            /* ���Ӵ���ָ�?*/
//            if (rslt != BMI2_OK) {
//                printf("���ݶ�ȡ����: %d, ������...\r\n", rslt);
//                HAL_Delay(50);
//                if (++retry_count > 5) {
//                    printf("����ʧ�ܣ��������³�ʼ��������\r\n");
//                    goto init_retry_label;
//                }
//                continue;
//            }

//            /* ������ݾ���״�?*/
//            if ((sens_data.status & BMI2_DRDY_ACC) && (sens_data.status & BMI2_DRDY_GYR))
//            {
//                /* ת�����ٶȼ����� */
//                acc_x = lsb_to_mps2(sens_data.acc.x, (float)2, bmi.resolution);
//                acc_y = lsb_to_mps2(sens_data.acc.y, (float)2, bmi.resolution);
//                acc_z = lsb_to_mps2(sens_data.acc.z, (float)2, bmi.resolution);

//                /* ת������������ */
//                gyr_x = lsb_to_dps(sens_data.gyr.x, (float)2000, bmi.resolution);
//                gyr_y = lsb_to_dps(sens_data.gyr.y, (float)2000, bmi.resolution);
//                gyr_z = lsb_to_dps(sens_data.gyr.z, (float)2000, bmi.resolution);

//                /* ��ȡ���������� */
//                if (bmm.chip_id == BMM350_CHIP_ID)
//                {
//                    rslt = bmm350_get_compensated_mag_xyz_temp_data(&mag_temp_data, &bmm);
//                    if (rslt != BMM350_OK) {
//                        printf("���������ݶ�ȡ����: %d\r\n", rslt);
//                    }
//                }
//                
//                /* ��ӡ���������� */
//                printf("Status:%d, Acc: %d %d %d, Acc(g): %.2f %.2f %.2f, Gyr: %d %d %d, Gyr(dps): %.2f %.2f %.2f, Mag: %.2f %.2f %.2f\r\n",
//                       sens_data.status,
//                       sens_data.acc.x, sens_data.acc.y, sens_data.acc.z,
//                       acc_x, acc_y, acc_z,
//                       sens_data.gyr.x, sens_data.gyr.y, sens_data.gyr.z,
//                       gyr_x, gyr_y, gyr_z,
//                       mag_temp_data.x, mag_temp_data.y, mag_temp_data.z);
//                
//                /* ˢ�´��ڻ�������ȷ�������������?*/
//                uart_flush();
//            }
//            else
//            {
//                printf("�ȴ����ݾ���...\r\n");
//                HAL_Delay(50);
//            }
//        }
//    }
//    else
//    {
//        printf("\nInvalid BMM350 (Aux) sensor - Chip ID : 0x%x\n", bmm.chip_id);
//    }

//    return rslt;
//}

int IMU_INIT_GETDATA(void)
{
    /* Status of api are returned to this variable. */
    int8_t rslt;
    uint8_t retry_count = 0;
    uint8_t init_retry = 0;
    uint32_t last_time = 0;
    uint32_t last_send_time = 0;
    float dt = 0.01f; // ��ʼ����ʱ����
    
    /* Variable to select the pull-up resistor which is set to trim register */
    uint8_t regdata;

    /* Accel, Gyro and Aux sensors are listed in array. */
    uint8_t sensor_list[3] = {BMI2_ACCEL, BMI2_GYRO, BMI2_AUX};

    /* Sensor initialization configuration. */
    struct bmm350_dev bmm = { 0 };
    struct bmi2_dev bmi;

    /* bmm350 magnetometer data */
    struct bmm350_mag_temp_data mag_temp_data;

    /* Structure to define type of sensor and their respective data. */
    struct bmi2_sens_data sens_data = {{0}};

    /* Variables to define read the accel and gyro data in float */
    float acc_x = 0, acc_y = 0, acc_z = 0;
    float gyr_x = 0, gyr_y = 0, gyr_z = 0;
    float mag_x = 0, mag_y = 0, mag_z = 0;
    
    // ��̬��
    attitude_t attitude;

    // ��ʼ����̬����
    imu_attitude_init();

    printf("IMU��ʼ����ʼ...\r\n");
    
    // ������������
    ANO_Send_LinkData(1);

    // ʹ��whileѭ��ʵ�ֳ�ʼ������
    while (init_retry < 3) {
        /* ��ʼ���������ӿ� */
        rslt = bmi2_interface_init(&bmi, BMI2_I2C_INTF);
        if (rslt != BMI2_OK) {
            printf("�ӿڳ�ʼ��ʧ�ܣ�������: %d��������...\r\n", rslt);
            HAL_Delay(100);
            init_retry++;
            continue;
        }
        
        /* ��ʼ�������������ӿ� */
        rslt = bmi2_aux_interface_init(&bmm, &bmi);
        if (rslt != BMI2_OK) {
            printf("�����ӿڳ�ʼ��ʧ�ܣ�������: %d\r\n", rslt);
            HAL_Delay(100);
            init_retry++;
            continue;
        }

        /* ��ʼ��BMI270������ */
        rslt = bmi270_init(&bmi);
        if (rslt != BMI2_OK) {
            printf("BMI270��ʼ��ʧ�ܣ�������: %d\r\n", rslt);
            HAL_Delay(100);
            init_retry++;
            continue;
        }

        /* ������������ */
        regdata = BMI2_ASDA_PUPSEL_10K;
        rslt = bmi2_set_regs(BMI2_AUX_IF_TRIM, &regdata, 1, &bmi);
        bmi2_error_codes_print_result(rslt);

        /* ���ü��ٶȼƺ������� */
        bmi2_accel_gyro_conf(&bmi);

        /* ���ø��������� */
        bmi2_aux_conf(BMI2_AUX_ODR_100HZ, &bmm, &bmi);

        /* ʹ�ܴ����� */
        rslt = bmi2_sensor_enable(sensor_list, 3, &bmi);
        if (rslt != BMI2_OK) {
            printf("������ʹ��ʧ�ܣ�������: %d\r\n", rslt);
            HAL_Delay(100);
            init_retry++;
            continue;
        }

        /* ��ʼ���ɹ�������ѭ�� */
        break;
    }

    if (init_retry >= 3) {
        printf("��ʼ��ʧ�ܣ�����������Դ���\r\n");
        return -1;
    }

    /* ӳ�����ݾ����ж� */
    rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT1, &bmi);
    bmi2_error_codes_print_result(rslt);

    printf("IMU��ʼ����ɣ���ʼ��ȡ����\r\n");

    if (bmm.chip_id == BMM350_CHIP_ID)
    {
        printf("��Ч��BMM350�����ƴ����� - Chip ID: 0x%x\r\n", bmm.chip_id);
        
        // ��ȡ��ʼʱ���?
        last_time = HAL_GetTick();
        last_send_time = last_time;
        
        /* �����ݶ�ȡѭ�� */
        while (1)
        {
            uint32_t current_time = HAL_GetTick();
            
                        // ����ʱ����(��)
            dt = (current_time - last_time) / 1000.0f;
            if (dt <= 0.001f) dt = 0.01f; // ��ֹʱ������С
            last_time = current_time;
            
            /* ��ȡ���������� */
            rslt = bmi2_get_sensor_data(&sens_data, &bmi);
            
            /* ����ָ�?*/
            if (rslt != BMI2_OK) {
                printf("���ݶ�ȡ����: %d, ������...\r\n", rslt);
                HAL_Delay(50);
                retry_count++;
                if (retry_count > 5) {
                    printf("��ȡʧ�ܴ������࣬���³�ʼ��\r\n");
                    init_retry = 0;
                    break; // ������ѭ�������³��Գ�ʼ��
                }
                continue;
            }
            
            retry_count = 0; // �ɹ���ȡ�����ô������?

            /* ������ݾ���״�?*/
            if ((sens_data.status & BMI2_DRDY_ACC) && (sens_data.status & BMI2_DRDY_GYR))
            {
                /* ת�����ٶȼ����� (m/s2) */
                acc_x = lsb_to_mps2(sens_data.acc.x, (float)2, bmi.resolution);
                acc_y = lsb_to_mps2(sens_data.acc.y, (float)2, bmi.resolution);
                acc_z = lsb_to_mps2(sens_data.acc.z, (float)2, bmi.resolution);

                /* ת������������ (�Ӷ�/��ת��Ϊ����/��) */
                gyr_x = lsb_to_dps(sens_data.gyr.x, (float)2000, bmi.resolution) * 0.0174533f;
                gyr_y = lsb_to_dps(sens_data.gyr.y, (float)2000, bmi.resolution) * 0.0174533f;
                gyr_z = lsb_to_dps(sens_data.gyr.z, (float)2000, bmi.resolution) * 0.0174533f;

                /* ��ȡ���������� (΢��˹��) */
                if (bmm.chip_id == BMM350_CHIP_ID)
                {
                    rslt = bmm350_get_compensated_mag_xyz_temp_data(&mag_temp_data, &bmm);
                    if (rslt != BMM350_OK) {
                        printf("���������ݶ�ȡ����: %d\r\n", rslt);
                        mag_x = mag_y = mag_z = 0.0f;
                    } else {
                        mag_x = mag_temp_data.x;
                        mag_y = mag_temp_data.y;
                        mag_z = mag_temp_data.z;
                    }
                }
                
                // ������̬����
                imu_attitude_update(acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z, mag_x, mag_y, mag_z, dt);
                
                // ��ȡ��̬��
                attitude = imu_attitude_get();
                
                // �������ݷ���Ƶ��Ϊ50Hz
                if (current_time - last_send_time >= 20) { // 20ms���?
                    last_send_time = current_time;
                    
                    // ������̬������ - ʹ���������Э��?
                    // 发送姿态角数据 - 使用匿名协议V8格式
                    ANO_Send_Attitude(attitude.roll, attitude.pitch, attitude.yaw);
                    
                    // ���ʹ�����ԭʼ����
                    // 发送惯性传感器数据(加速度+陀螺仪) - 转换为协议要求的单位
                    int16_t acc_cm_x = (int16_t)(acc_x * 100);  // m/s² 转为 cm/s²
                    int16_t acc_cm_y = (int16_t)(acc_y * 100);
                    int16_t acc_cm_z = (int16_t)(acc_z * 100);
                    int16_t gyro_raw_x = (int16_t)(gyr_x * 57.2958f * 32768.0f / 2000.0f);  // rad/s转为原始数据
                    int16_t gyro_raw_y = (int16_t)(gyr_y * 57.2958f * 32768.0f / 2000.0f);
                    int16_t gyro_raw_z = (int16_t)(gyr_z * 57.2958f * 32768.0f / 2000.0f);
                    
                    ANO_Send_InertialSensor(acc_cm_x, acc_cm_y, acc_cm_z, 
                                            gyro_raw_x, gyro_raw_y, gyro_raw_z, 0);
                    
                    // 发送磁力计温度数据
                    ANO_Send_MagTempSensor((int16_t)mag_x, (int16_t)mag_y, (int16_t)mag_z, 
                                           250, 1); // 假设25°C，状态正�?
                    
                    // �����û��Զ�������
                    // 发送用户自定义数据(灵活格式�?
                    ANO_Send_UserData(attitude.roll, attitude.pitch, attitude.yaw, 
                                      sqrtf(acc_x*acc_x + acc_y*acc_y + acc_z*acc_z), 0);
                    
                    // ÿ�����һ����̬�����ݵ������ն�?
                    static uint32_t print_time = 0;
                    if (current_time - print_time >= 1000) {
                        print_time = current_time;
                        printf("Roll: %.2f, Pitch: %.2f, Yaw: %.2f\r\n", 
                               attitude.roll, attitude.pitch, attitude.yaw);
                        
                        // ˢ�´��ڻ�����
                        uart_flush();
                    }
                }
            }
            
            // ������ʱ������CPU����
            HAL_Delay(1);
        }
    }
    else
    {
        printf("\nInvalid BMM350 (Aux) sensor - Chip ID : 0x%x\n", bmm.chip_id);
    }

    return rslt;
}


/*******************************************************************************/
/*!               Static  Functions Definitions                                */

/*!
 * @brief This function configures accelerometer and gyroscope
 */
static void bmi2_accel_gyro_conf(struct bmi2_dev *bmi)
{
    int8_t rslt;

    struct bmi2_sens_config config[2];

    config[0].type = BMI2_ACCEL;
    config[1].type = BMI2_GYRO;

    /* Get default configurations for the type of feature selected. */
    rslt = bmi2_get_sensor_config(config, 2, bmi);
    bmi2_error_codes_print_result(rslt);

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
    bmi2_error_codes_print_result(rslt);
}

/*!
 * @brief This function configures auxiliary sensor (bmm350)
 */
static void bmi2_aux_conf(uint8_t mag_odr, struct bmm350_dev *bmm, struct bmi2_dev *bmi)
{
    int8_t rslt;

    struct bmi2_sens_config bmi2_conf;
    //struct bmm350_settings bmm1_sett;
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
        break;
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
    bmi2_error_codes_print_result(rslt);

//    bmm1_sett.pwr_mode = bmm350_POWERMODE_FORCED;

    if (mag_odr != 0) /* Proceed only if ODR is valid */
    {
        rslt = bmm350_init(bmm);
        bmm350_error_codes_print_result("bmm350_init", rslt);

        /* Set ODR and performance */
        rslt = bmm350_set_odr_performance(bmm350_odr, BMM350_AVERAGING_4, bmm);
        bmm350_error_codes_print_result("bmm350_set_odr_performance", rslt);

        /* Enable all axis */
        rslt = bmm350_enable_axes(BMM350_X_EN, BMM350_Y_EN, BMM350_Z_EN, bmm);
        bmm350_error_codes_print_result("bmm350_enable_axes", rslt);

        rslt = bmm350_set_powermode(BMM350_NORMAL_MODE, bmm);
        bmm350_error_codes_print_result("bmm350_set_powermode", rslt);
        
        bmi2_conf.cfg.aux.manual_en = BMI2_ENABLE;
//        rslt = bmi2_set_sensor_config(&bmi2_conf, 1, bmi);
//        bmi2_error_codes_print_result(rslt);
    }
    else
    {
        bmi2_conf.cfg.aux.aux_en = BMI2_DISABLE;
        bmi2_conf.cfg.aux.fcu_write_en = BMI2_DISABLE;
        bmi2_conf.cfg.aux.manual_en = BMI2_ENABLE;

        rslt = bmi2_set_sensor_config(&bmi2_conf, 1, bmi);
        bmi2_error_codes_print_result(rslt);

        rslt = bmi2_sensor_disable(&bmi2_sens, 1, bmi);
        bmi2_error_codes_print_result(rslt);
    }
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
int8_t bmi2_aux_interface_init(struct bmm350_dev *bmm, struct bmi2_dev *bmi)
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
//        bmm->intf = bmm350_I2C_INTF;
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
