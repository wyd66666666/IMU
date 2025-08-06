#ifndef _MYIIC_H
#define _MYIIC_H

#include "stdint.h"
#include "stm32g4xx.h"

// STM32G4系列不支持位带操作！
// G4系列基于Cortex-M4核心，但没有位带区域
// 我们需要使用标准的寄存器操作

// 重新定义IO操作宏，使用HAL库函数
#define PAout(n)   (HAL_GPIO_WritePin(GPIOA, 1<<(n), GPIO_PIN_SET))
#define PAin(n)    (HAL_GPIO_ReadPin(GPIOA, 1<<(n)))

#define PBout(n)   (HAL_GPIO_WritePin(GPIOB, 1<<(n), GPIO_PIN_SET))
#define PBin(n)    (HAL_GPIO_ReadPin(GPIOB, 1<<(n)))

// IO方向设置
#define SDA_IN_1()  {GPIOB->MODER&=~(3<<(7*2));GPIOB->MODER|=0<<7*2;}  // PB7输入模式
#define SDA_OUT_1() {GPIOB->MODER&=~(3<<(7*2));GPIOB->MODER|=1<<7*2;}  // PB7输出模式

// IO操作 - 使用直接寄存器操作
#define IIC_SCL_1_HIGH()  (GPIOA->BSRR = GPIO_PIN_15)     // PA15置高
#define IIC_SCL_1_LOW()   (GPIOA->BRR = GPIO_PIN_15)      // PA15置低
#define IIC_SDA_1_HIGH()  (GPIOB->BSRR = GPIO_PIN_7)      // PB7置高
#define IIC_SDA_1_LOW()   (GPIOB->BRR = GPIO_PIN_7)       // PB7置低

// 读取引脚状态
#define READ_SCL_1   ((GPIOA->IDR & GPIO_PIN_15) ? 1 : 0)  // 读PA15
#define READ_SDA_1   ((GPIOB->IDR & GPIO_PIN_7) ? 1 : 0)   // 读PB7

// 为了兼容性，定义IIC_SCL_1和IIC_SDA_1为函数
void Set_SCL(uint8_t val);
void Set_SDA(uint8_t val);

#define IIC_SCL_1  Set_SCL
#define IIC_SDA_1  Set_SDA

// IIC所有操作函数
void IIC_Init(void);
void IIC_Start(void);
void IIC_Stop(void);
void IIC_Send_Byte(uint8_t txd);
uint8_t IIC_Read_Byte(unsigned char ack);
uint8_t IIC_Wait_Ack(void);
void IIC_Ack(void);
void IIC_NAck(void);

uint8_t IIC_Write_1Byte(uint8_t SlaveAddress,uint8_t REG_Address,uint8_t REG_data);
uint8_t IIC_Read_1Byte(uint8_t SlaveAddress,uint8_t REG_Address);
uint8_t IIC_Write_nByte(uint8_t SlaveAddress, uint8_t REG_Address, uint16_t len, const uint8_t *buf);
uint8_t IIC_Read_nByte(uint8_t SlaveAddress, uint8_t REG_Address, uint16_t len, uint8_t *buf);

#endif



