#ifndef BMP580_I2C_H
#define BMP580_I2C_H

#include <stdint.h>
#include "stm32g4xx_hal.h"

/* 软件I2C引脚定义 (I2C2 for BMP580) */
#define I2C2_SCL_PIN         GPIO_PIN_4    // PC4
#define I2C2_SCL_GPIO_PORT   GPIOC
#define I2C2_SDA_PIN         GPIO_PIN_8    // PA8
#define I2C2_SDA_GPIO_PORT   GPIOA

/* 函数声明 */
void I2C2_Init(void);
void I2C2_Start(void);
void I2C2_Stop(void);
uint8_t I2C2_Wait_Ack(void);
void I2C2_Ack(void);
void I2C2_NAck(void);
void I2C2_Send_Byte(uint8_t txd);
uint8_t I2C2_Read_Byte(uint8_t ack);

/* 封装的读写函数 */
uint8_t I2C2_Write_nByte(uint8_t SlaveAddress, uint8_t REG_Address, uint16_t len, const uint8_t *buf);
uint8_t I2C2_Read_nByte(uint8_t SlaveAddress, uint8_t REG_Address, uint16_t len, uint8_t *buf);

#endif /* BMP580_I2C_H */


