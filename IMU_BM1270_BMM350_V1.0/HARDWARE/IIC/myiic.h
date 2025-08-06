#ifndef _MYIIC_H
#define _MYIIC_H

#include "stdint.h"
#include "stm32g4xx.h"

// STM32G4ϵ�в�֧��λ��������
// G4ϵ�л���Cortex-M4���ģ���û��λ������
// ������Ҫʹ�ñ�׼�ļĴ�������

// ���¶���IO�����꣬ʹ��HAL�⺯��
#define PAout(n)   (HAL_GPIO_WritePin(GPIOA, 1<<(n), GPIO_PIN_SET))
#define PAin(n)    (HAL_GPIO_ReadPin(GPIOA, 1<<(n)))

#define PBout(n)   (HAL_GPIO_WritePin(GPIOB, 1<<(n), GPIO_PIN_SET))
#define PBin(n)    (HAL_GPIO_ReadPin(GPIOB, 1<<(n)))

// IO��������
#define SDA_IN_1()  {GPIOB->MODER&=~(3<<(7*2));GPIOB->MODER|=0<<7*2;}  // PB7����ģʽ
#define SDA_OUT_1() {GPIOB->MODER&=~(3<<(7*2));GPIOB->MODER|=1<<7*2;}  // PB7���ģʽ

// IO���� - ʹ��ֱ�ӼĴ�������
#define IIC_SCL_1_HIGH()  (GPIOA->BSRR = GPIO_PIN_15)     // PA15�ø�
#define IIC_SCL_1_LOW()   (GPIOA->BRR = GPIO_PIN_15)      // PA15�õ�
#define IIC_SDA_1_HIGH()  (GPIOB->BSRR = GPIO_PIN_7)      // PB7�ø�
#define IIC_SDA_1_LOW()   (GPIOB->BRR = GPIO_PIN_7)       // PB7�õ�

// ��ȡ����״̬
#define READ_SCL_1   ((GPIOA->IDR & GPIO_PIN_15) ? 1 : 0)  // ��PA15
#define READ_SDA_1   ((GPIOB->IDR & GPIO_PIN_7) ? 1 : 0)   // ��PB7

// Ϊ�˼����ԣ�����IIC_SCL_1��IIC_SDA_1Ϊ����
void Set_SCL(uint8_t val);
void Set_SDA(uint8_t val);

#define IIC_SCL_1  Set_SCL
#define IIC_SDA_1  Set_SDA

// IIC���в�������
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



