#include "bmp5_port.h"
#include "bmp580_i2c.h"
#include "delay.h"
#include <stdio.h>

/* BMP580 I2C地址 */
#define BMP580_I2C_ADDR_PRIM    0x46
#define BMP580_I2C_ADDR_SEC     0x47

static struct bmp5_dev bmp580_dev;
static struct bmp5_osr_odr_press_config osr_odr_cfg;
static uint8_t bmp580_initialized = 0;
static uint8_t dev_addr = BMP580_I2C_ADDR_PRIM;

/* I2C读函数回调 */
static BMP5_INTF_RET_TYPE bmp580_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr)
{
    uint8_t dev_id = *(uint8_t*)intf_ptr;
    if (I2C2_Read_nByte(dev_id, reg_addr, len, data) == 0)
        return BMP5_INTF_RET_SUCCESS;
    return BMP5_E_COM_FAIL;
}

/* I2C写函数回调 */
static BMP5_INTF_RET_TYPE bmp580_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr)
{
    uint8_t dev_id = *(uint8_t*)intf_ptr;
    if (I2C2_Write_nByte(dev_id, reg_addr, len, data) == 0)
        return BMP5_INTF_RET_SUCCESS;
    return BMP5_E_COM_FAIL;
}

/* 延时函数回调 */
static void bmp580_delay_us(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;
    delay_us(period);
}

int8_t BMP580_Init(void)
{
    int8_t rslt;
    
    printf("\n--- BMP580 Driver ---\n");
    
    /* 1. 初始化I2C2硬件 */
    I2C2_Init();
    
    /* 2. 配置BMP5设备结构体 */
    bmp580_dev.intf = BMP5_I2C_INTF;
    bmp580_dev.read = bmp580_i2c_read;
    bmp580_dev.write = bmp580_i2c_write;
    bmp580_dev.delay_us = bmp580_delay_us;
    bmp580_dev.intf_ptr = &dev_addr;
    
    /* 3. 软复位并初始化传感器 */
    rslt = bmp5_soft_reset(&bmp580_dev);
    if (rslt != BMP5_OK) {
        // 尝试备用地址
        dev_addr = BMP580_I2C_ADDR_SEC;
        rslt = bmp5_soft_reset(&bmp580_dev);
        if (rslt != BMP5_OK) {
            printf("BMP580 soft reset failed. Error: %d\n", rslt);
            return rslt;
        }
    }
    delay_ms(10);
    
    rslt = bmp5_init(&bmp580_dev);
    if (rslt != BMP5_OK) {
        printf("BMP580 init failed. Error: %d\n", rslt);
        return rslt;
    }
    printf("BMP580 init successful. Chip ID: 0x%02X\n", bmp580_dev.chip_id);

    /* 4. 配置传感器参数 */
    osr_odr_cfg.osr_t = BMP5_OVERSAMPLING_8X;
    osr_odr_cfg.osr_p = BMP5_OVERSAMPLING_16X;
    osr_odr_cfg.press_en = BMP5_ENABLE;
    osr_odr_cfg.odr = BMP5_ODR_25_HZ;
    rslt = bmp5_set_osr_odr_press_config(&osr_odr_cfg, &bmp580_dev);
    if (rslt != BMP5_OK) {
        printf("Set OSR failed. Error: %d\n", rslt);
        return rslt;
    }
    
    /* 5. 设置电源模式 */
    rslt = bmp5_set_power_mode(BMP5_POWERMODE_NORMAL, &bmp580_dev);
    if (rslt != BMP5_OK) {
        printf("Set power mode failed. Error: %d\n", rslt);
        return rslt;
    }
    
    bmp580_initialized = 1;
    printf("BMP580 initialization complete.\n");
    return BMP5_OK;
}

int8_t BMP580_ReadData(float *temperature, float *pressure)
{
    if (!bmp580_initialized) return BMP5_E_DEV_NOT_FOUND;
    
    struct bmp5_sensor_data sensor_data;
    int8_t rslt = bmp5_get_sensor_data(&sensor_data, &osr_odr_cfg, &bmp580_dev);
    
    if (rslt == BMP5_OK) {
        *temperature = sensor_data.temperature;
        *pressure = sensor_data.pressure;
    }
    
    return rslt;
}

