//#include "bmp5_port.h"
//#include "stm32g4xx_hal.h"
//#include "i2c.h"    // CubeMX生成的
//#include "usart.h"  // 假设您用USART发送数据
//#include <stdio.h>

////-----------------------------------------------------------------------------
//// 外部引用
////-----------------------------------------------------------------------------
//extern I2C_HandleTypeDef hi2c2;
//extern void delay_us(uint32_t nus);

////-----------------------------------------------------------------------------
//// 宏定义与全局变量
////-----------------------------------------------------------------------------
//#define BMP580_I2C_ADDR  (BMP5_I2C_ADDR_SEC << 1) // 地址0x47
//static struct bmp5_dev bmp5_dev;



////-----------------------------------------------------------------------------
//// 平台适配函数实现 (static)
////-----------------------------------------------------------------------------
//static BMP5_INTF_RET_TYPE bmp5_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
//{
//    if (HAL_I2C_Mem_Read(&hi2c2, BMP580_I2C_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, reg_data, len, HAL_MAX_DELAY) == HAL_OK)
//        return BMP5_OK;
//    else
//        return BMP5_E_COM_FAIL;
//}

//static BMP5_INTF_RET_TYPE bmp5_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
//{
//    if (HAL_I2C_Mem_Write(&hi2c2, BMP580_I2C_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, (uint8_t*)reg_data, len, HAL_MAX_DELAY) == HAL_OK)
//        return BMP5_OK;
//    else
//        return BMP5_E_COM_FAIL;
//}

//static void bmp5_delay_us(uint32_t period_us, void *intf_ptr)
//{
//    delay_us(period_us);
//}

////-----------------------------------------------------------------------------
//// 模块化测试程序 (使用FORCED模式)
////-----------------------------------------------------------------------------
//void BMP580_Test_Run(void)
//{
//    printf("\r\n--- BMP580 Sensor Test Program (Forced Mode Debug) ---\r\n");

//    /* 变量定义 */
//    int8_t rslt;
//    struct bmp5_osr_odr_press_config osr_cfg; // 在Forced模式下，ODR无效，只关心OSR
//    struct bmp5_sensor_data sensor_data;

//    /* 1. 绑定平台相关的接口函数 */
//    bmp5_dev.read = bmp5_i2c_read;
//    bmp5_dev.write = bmp5_i2c_write;
//    bmp5_dev.delay_us = bmp5_delay_us;
//    bmp5_dev.intf = BMP5_I2C_INTF;

//    /* 2. 初始化传感器 */
//    rslt = bmp5_init(&bmp5_dev);
//    if (rslt != BMP5_OK)
//    {
//        printf("Error: BMP580 initialization failed with code %d\r\n", rslt);
//        while(1);
//    }
//    printf("BMP580 initialization successful. Chip ID: 0x%02X\r\n", bmp5_dev.chip_id);

//    /* 3. 配置传感器 (只配置OSR，并确保压力使能) */
//    rslt = bmp5_get_osr_odr_press_config(&osr_cfg, &bmp5_dev);
//    if (rslt == BMP5_OK)
//    {
//        /* 设置一个保守的过采样率 */
//        osr_cfg.osr_t = BMP5_OVERSAMPLING_2X;
//        osr_cfg.osr_p = BMP5_OVERSAMPLING_8X;
//        
//        /* 关键：确保压力传感器被使能 */
//        osr_cfg.press_en = BMP5_ENABLE;
//        
//        rslt = bmp5_set_osr_odr_press_config(&osr_cfg, &bmp5_dev);
//        if (rslt != BMP5_OK)
//        {
//            printf("Error: Failed to set OSR config with code %d\r\n", rslt);
//        } else {
//            printf("Set OSR config successful (press_en=ENABLE).\r\n");
//        }
//    } else {
//        printf("Error: Failed to get OSR config with code %d\r\n", rslt);
//    }

//    /* 4. 先设置为休眠模式，准备进入Forced模式循环 */
//    rslt = bmp5_set_power_mode(BMP5_POWERMODE_STANDBY, &bmp5_dev);
//    if (rslt != BMP5_OK)
//    {
//        printf("Error: Failed to set STANDBY mode with code %d\r\n", rslt);
//        while(1);
//    }
//    printf("Set to STANDBY mode successful. Starting forced mode loop...\r\n\r\n");
//    HAL_Delay(100);

//    /* 5. 无限循环，每次都手动触发一次测量 */
//    while (1)
//    {
//        /* 5.1 手动触发一次测量 */
//        rslt = bmp5_set_power_mode(BMP5_POWERMODE_FORCED, &bmp5_dev);
//        if (rslt != BMP5_OK)
//        {
//            printf("Error: Failed to trigger a forced measurement.\r\n");
//            HAL_Delay(1000); // 出错后等待1秒再重试
//            continue;
//        }

//        /* 5.2 等待测量完成。这个延时很重要 */
//        /* 根据数据手册，8x OSR压力 + 2x OSR温度 的最大转换时间约为 25ms */
//        /* 我们给一个宽裕的延时，例如 50ms */
//        HAL_Delay(50);

//        /* 5.3 读取数据 */
//        rslt = bmp5_get_sensor_data(&sensor_data, &osr_cfg, &bmp5_dev);
//        
//        if (rslt == BMP5_OK)
//        {
//            /* 检查压力值是否有效 */
//            if (sensor_data.pressure < 1000.0f) // 正常大气压远大于1000 Pa
//            {
//                printf("Warning: Pressure reading is %.2f Pa, which is invalid. Checking config...\r\n", sensor_data.pressure);
//            }
//            else
//            {
//                printf("Temperature: %.2f degC, Pressure: %.2f hPa\r\n", 
//                        sensor_data.temperature, 
//                        sensor_data.pressure / 100.0f);
//            }
//        }
//        else
//        {
//            printf("Error: Failed to get sensor data in forced mode with code %d\r\n", rslt);
//        }

//        /* 每次测量的间隔 */
//        HAL_Delay(1000);
//    }
//}

#include "bmp5_port.h"
#include "stm32g4xx_hal.h"
#include "i2c.h"    // CubeMX生成的
#include "usart.h"  // 用于打印日志
#include <stdio.h>
#include <stdbool.h>
//-----------------------------------------------------------------------------
// 外部引用
//-----------------------------------------------------------------------------
extern I2C_HandleTypeDef hi2c2;
extern void delay_us(uint32_t nus);
extern void delay_ms(uint32_t nms); // 假设你也有毫秒延时

//-----------------------------------------------------------------------------
// 模块内部全局变量 (static)
//-----------------------------------------------------------------------------
#define BMP580_I2C_ADDR  (BMP5_I2C_ADDR_SEC << 1) // 地址0x47

static struct bmp5_dev bmp5_dev_g;
static struct bmp5_osr_odr_press_config osr_cfg_g;
static bool is_bmp580_initialized = false;


//-----------------------------------------------------------------------------
// 平台适配函数实现 (static)
//-----------------------------------------------------------------------------
static BMP5_INTF_RET_TYPE bmp5_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    return (HAL_I2C_Mem_Read(&hi2c2, BMP580_I2C_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, reg_data, len, HAL_MAX_DELAY) == HAL_OK) ? BMP5_OK : BMP5_E_COM_FAIL;
}
static BMP5_INTF_RET_TYPE bmp5_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    return (HAL_I2C_Mem_Write(&hi2c2, BMP580_I2C_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, (uint8_t*)reg_data, len, HAL_MAX_DELAY) == HAL_OK) ? BMP5_OK : BMP5_E_COM_FAIL;
}
static void bmp5_delay_us(uint32_t period_us, void *intf_ptr) {
    delay_us(period_us);
}

//-----------------------------------------------------------------------------
// 高级API函数实现
//-----------------------------------------------------------------------------

/**
 * @brief 初始化BMP580传感器 (包含健壮的复位逻辑)
 * 
 * @note  此函数会先对传感器执行软复位，以确保其处于已知的初始状态，
 *        然后再进行初始化和配置。这可以解决因STM32软件复位
 *        而传感器未被硬件复位导致的状态不一致问题。
 * 
 * @return int8_t 0表示成功 (BMP5_OK)，其他为错误码。
 */
int8_t BMP580_Init(void)
{
    int8_t rslt;
    
    // 全局静态变量，在函数第一次被调用时打印一次信息
    static bool first_run = true;
    if (first_run) {
        printf("\r\n--- BMP580 Driver ---\r\n");
        first_run = false;
    }

    printf("BMP580_Init: Starting initialization process...\r\n");

    /* 1. 绑定平台相关的接口函数 */
    //    这个操作只需要做一次，但每次调用都执行也无妨，开销很小。
    bmp5_dev_g.read = bmp5_i2c_read;
    bmp5_dev_g.write = bmp5_i2c_write;
    bmp5_dev_g.delay_us = bmp5_delay_us;
    bmp5_dev_g.intf = BMP5_I2C_INTF;

    /* 
     * 2. 强制执行软复位 (关键步骤)
     *    这会把传感器恢复到上电时的默认状态。
     */
    printf("BMP580_Init: Performing a soft reset to ensure clean state...\r\n");
    rslt = bmp5_soft_reset(&bmp5_dev_g);
    if (rslt != BMP5_OK)
    {
        // 在某些总线状态下，复位命令本身可能返回通信失败，但复位可能已经生效。
        // 所以我们只打印一个警告，然后继续尝试初始化。
        printf("BMP580_Init: Warning - Soft reset command returned error %d. Continuing...\r\n", rslt);
    }

    /* 
     * 3. 等待传感器完成内部重启流程 (必须)
     *    根据数据手册，软复位后需要约2ms的时间。我们给一个宽裕的值。
     */
    delay_ms(5); 

    /* 4. 执行官方的初始化函数 */
    //    这个函数会检查Chip ID，并从NVM中加载校准参数。
    rslt = bmp5_init(&bmp5_dev_g);
    if (rslt != BMP5_OK)
    {
        printf("BMP580_Init: FATAL - bmp5_init failed with code %d. Check hardware and connections.\r\n", rslt);
        is_bmp580_initialized = false;
        return rslt;
    }
    printf("BMP580_Init: bmp5_init successful. Chip ID: 0x%02X\r\n", bmp5_dev_g.chip_id);

    /* 5. 配置传感器 OSR (过采样率) 和压力使能 */
    rslt = bmp5_get_osr_odr_press_config(&osr_cfg_g, &bmp5_dev_g);
    if (rslt == BMP5_OK)
    {
        osr_cfg_g.osr_t = BMP5_OVERSAMPLING_2X;
        osr_cfg_g.osr_p = BMP5_OVERSAMPLING_8X;
        osr_cfg_g.press_en = BMP5_ENABLE; // 确保压力测量是使能的
        
        rslt = bmp5_set_osr_odr_press_config(&osr_cfg_g, &bmp5_dev_g);
        if (rslt != BMP5_OK)
        {
            printf("BMP580_Init: Warning - bmp5_set_osr_odr_press_config failed with code %d\r\n", rslt);
        } else {
            printf("BMP580_Init: Set OSR config successful.\r\n");
        }
    } else {
        printf("BMP580_Init: Warning - bmp5_get_osr_odr_press_config failed with code %d\r\n", rslt);
    }
    
    /* 6. 设置为休眠模式 (Standby)，准备好被GetData函数随时唤醒 */
    rslt = bmp5_set_power_mode(BMP5_POWERMODE_STANDBY, &bmp5_dev_g);
    if (rslt != BMP5_OK)
    {
        printf("BMP580_Init: FATAL - Failed to set STANDBY mode with code %d\r\n", rslt);
        is_bmp580_initialized = false;
        return rslt;
    }
    
    printf("BMP580_Init: Initialization complete, sensor is in STANDBY mode.\r\n\r\n");
    is_bmp580_initialized = true;

    return BMP5_OK;
}

/**
 * @brief 获取一次BMP580的传感器数据
 */
int8_t BMP580_GetData(float* temperature, float* pressure)
{
    if (!is_bmp580_initialized)
    {
        return BMP5_E_DEV_NOT_FOUND; // 或者自定义一个“未初始化”错误
    }
    if (temperature == NULL || pressure == NULL)
    {
        return BMP5_E_NULL_PTR;
    }

    int8_t rslt;
    struct bmp5_sensor_data sensor_data;

    /* 1. 手动触发一次测量 */
    rslt = bmp5_set_power_mode(BMP5_POWERMODE_FORCED, &bmp5_dev_g);
    if (rslt != BMP5_OK)
    {
        return rslt; // 触发失败
    }

    /* 2. 等待测量完成 (约25ms，给50ms余量) */
    delay_ms(50); // 使用毫秒延时

    /* 3. 读取数据 */
    rslt = bmp5_get_sensor_data(&sensor_data, &osr_cfg_g, &bmp5_dev_g);
    
    if (rslt == BMP5_OK)
    {
        *temperature = sensor_data.temperature;
        *pressure = sensor_data.pressure;
    }

    return rslt;
}

