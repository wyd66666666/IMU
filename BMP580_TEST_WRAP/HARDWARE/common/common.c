/**
 * Copyright (C) 2023 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#include "common.h"
#include "bmi2_defs.h"
#include "delay.h"
#include "i2c.h" // 使用HAL的I2C库

/******************************************************************************/
/*!                 Macro definitions                                         */
#define BMI2XY_SHUTTLE_ID  UINT16_C(0x1B8)

/*! Macro that defines read write length */
#define READ_WRITE_LEN     UINT8_C(46)

/******************************************************************************/
/*!                Static variable definition                                 */

/*! Variable that holds the I2C device address */
static uint8_t dev_addr;

/*! Variable that holds the I2C bus instance */
static uint8_t bus_inst;

/*! Structure to hold interface configurations */
static struct coines_intf_config intf_conf;

/*! I2C handle pointer (需要根据你的HAL配置修改) */
extern I2C_HandleTypeDef hi2c1; // 假设使用hi2c1，请根据实际配置修改

/******************************************************************************/
/*!                User interface functions                                   */

/*!
 * I2C read function map to HAL platform
 */
BMI2_INTF_RETURN_TYPE bmi2_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    struct coines_intf_config intf_info = *(struct coines_intf_config *)intf_ptr;
    uint16_t dev_addr_shifted = intf_info.dev_addr << 1; // HAL库需要左移一位的设备地址
    
    if (HAL_I2C_Mem_Read(&hi2c1, dev_addr_shifted, reg_addr, I2C_MEMADD_SIZE_8BIT, reg_data, len, 1000) == HAL_OK)
    {
        return 0; // 成功返回0
    }
    return 1; // 失败返回非0
}

/*!
 * I2C write function map to HAL platform
 */
BMI2_INTF_RETURN_TYPE bmi2_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    struct coines_intf_config intf_info = *(struct coines_intf_config *)intf_ptr;
    uint16_t dev_addr_shifted = intf_info.dev_addr << 1; // HAL库需要左移一位的设备地址
    
    if (HAL_I2C_Mem_Write(&hi2c1, dev_addr_shifted, reg_addr, I2C_MEMADD_SIZE_8BIT, (uint8_t*)reg_data, len, 1000) == HAL_OK)
    {
        return 0; // 成功返回0
    }
    return 1; // 失败返回非0
}

/*!
 * Delay function map to HAL platform
 */
void bmi2_delay_us(uint32_t period, void *intf_ptr)
{
    delay_us(period);
}

/*!
 *  @brief Function to select the interface (only I2C now).
 */
int8_t bmi2_interface_init(struct bmi2_dev *bmi, uint8_t intf)
{
    int8_t rslt = BMI2_OK;
    int result = 0;

    if (bmi != NULL)
    {
        /* 只支持I2C接口 */
        if (intf == BMI2_I2C_INTF)
        {
            printf("I2C Interface \n");

            /* 初始化I2C接口函数 */
            dev_addr = BMI2_I2C_SEC_ADDR;
            bmi->intf = BMI2_I2C_INTF;
            bmi->read = bmi2_i2c_read;
            bmi->write = bmi2_i2c_write;

            delay_ms(100); // 等待I2C稳定

            // I2C已在MX_I2C1_Init()中初始化，无需在此处初始化
            // 如果需要额外配置，可以在此处添加

            bus_inst = 0;
        }
        else
        {
            printf("Only I2C interface is supported\n");
            return BMI2_E_INVALID_INPUT;
        }

        if (0 == result)
        {
            /* 分配设备地址和总线实例到接口指针 */
            intf_conf.bus = bus_inst;
            intf_conf.dev_addr = dev_addr;
            bmi->intf_ptr = ((void *)&intf_conf);

            /* 配置微秒延时 */
            bmi->delay_us = bmi2_delay_us;

            /* 配置最大读写长度(字节) */
            bmi->read_write_len = READ_WRITE_LEN;

            /* 分配为NULL以加载默认配置文件 */
            bmi->config_file_ptr = NULL;
        }
        else
        {
            rslt = BMI2_E_COM_FAIL;
        }
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
void bmi2_error_codes_print_result(int8_t rslt)
{
    switch (rslt)
    {
        case BMI2_OK:
            /* Do nothing */
            break;

        case BMI2_W_FIFO_EMPTY:
            printf("Warning [%d] : FIFO empty\r\n", rslt);
            break;
        
        // 省略其他错误代码处理...（保持原代码不变）
        
        default:
            printf("Error [%d] : Unknown error code\r\n", rslt);
            break;
    }
}

/*!
 *  @brief Deinitializes platform
 *
 *  @return void.
 */
void bmi2_coines_deinit(void)
{
    // 这里可以添加硬件I2C的反初始化代码，如果需要的话
    // HAL_I2C_DeInit(&hi2c1);
}


