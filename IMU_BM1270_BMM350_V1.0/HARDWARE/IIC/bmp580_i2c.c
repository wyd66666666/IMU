#include "bmp580_i2c.h"
#include "delay.h"

/* 引脚操作宏 */
#define I2C2_SCL_HIGH()  HAL_GPIO_WritePin(I2C2_SCL_GPIO_PORT, I2C2_SCL_PIN, GPIO_PIN_SET)
#define I2C2_SCL_LOW()   HAL_GPIO_WritePin(I2C2_SCL_GPIO_PORT, I2C2_SCL_PIN, GPIO_PIN_RESET)
#define I2C2_SDA_HIGH()  HAL_GPIO_WritePin(I2C2_SDA_GPIO_PORT, I2C2_SDA_PIN, GPIO_PIN_SET)
#define I2C2_SDA_LOW()   HAL_GPIO_WritePin(I2C2_SDA_GPIO_PORT, I2C2_SDA_PIN, GPIO_PIN_RESET)
#define I2C2_SDA_READ()  HAL_GPIO_ReadPin(I2C2_SDA_GPIO_PORT, I2C2_SDA_PIN)

/* 方向设置宏 (直接操作寄存器，效率高) */
#define I2C2_SDA_IN()  do { GPIOA->MODER &= ~(GPIO_MODER_MODE8); } while(0) // PA8 输入
#define I2C2_SDA_OUT() do { GPIOA->MODER &= ~(GPIO_MODER_MODE8); GPIOA->MODER |= GPIO_MODER_MODE8_0; } while(0) // PA8 输出

/* I2C初始化 */
void I2C2_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* 使能GPIO时钟 */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* 配置SCL引脚(PC4) */
    GPIO_InitStruct.Pin = I2C2_SCL_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;  // 开漏输出
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(I2C2_SCL_GPIO_PORT, &GPIO_InitStruct);

    /* 配置SDA引脚(PA8) */
    GPIO_InitStruct.Pin = I2C2_SDA_PIN;
    HAL_GPIO_Init(I2C2_SDA_GPIO_PORT, &GPIO_InitStruct);

    /* 初始状态为高电平 */
    I2C2_SCL_HIGH();
    I2C2_SDA_HIGH();
}

/* 产生I2C起始信号 */
void I2C2_Start(void)
{
    I2C2_SDA_OUT();
    I2C2_SDA_HIGH();
    I2C2_SCL_HIGH();
    delay_us(4);
    I2C2_SDA_LOW();
    delay_us(4);
    I2C2_SCL_LOW();
}

/* 产生I2C停止信号 */
void I2C2_Stop(void)
{
    I2C2_SDA_OUT();
    I2C2_SCL_LOW();
    I2C2_SDA_LOW();
    delay_us(4);
    I2C2_SCL_HIGH();
    I2C2_SDA_HIGH();
    delay_us(4);
}

/* 等待应答信号到来 */
uint8_t I2C2_Wait_Ack(void)
{
    uint8_t ucErrTime = 0;
    I2C2_SDA_IN();
    I2C2_SDA_HIGH();
    delay_us(1);
    I2C2_SCL_HIGH();
    delay_us(1);
    while(I2C2_SDA_READ())
    {
        ucErrTime++;
        if(ucErrTime > 250)
        {
            I2C2_Stop();
            return 1;
        }
    }
    I2C2_SCL_LOW();
    return 0;
}

/* 产生ACK应答 */
void I2C2_Ack(void)
{
    I2C2_SCL_LOW();
    I2C2_SDA_OUT();
    I2C2_SDA_LOW();
    delay_us(2);
    I2C2_SCL_HIGH();
    delay_us(2);
    I2C2_SCL_LOW();
}

/* 不产生ACK应答 */
void I2C2_NAck(void)
{
    I2C2_SCL_LOW();
    I2C2_SDA_OUT();
    I2C2_SDA_HIGH();
    delay_us(2);
    I2C2_SCL_HIGH();
    delay_us(2);
    I2C2_SCL_LOW();
}

/* 发送一个字节 */
void I2C2_Send_Byte(uint8_t txd)
{
    uint8_t t;
    I2C2_SDA_OUT();
    I2C2_SCL_LOW();
    for(t = 0; t < 8; t++)
    {
        if((txd & 0x80) >> 7)
            I2C2_SDA_HIGH();
        else
            I2C2_SDA_LOW();
        txd <<= 1;
        delay_us(2);
        I2C2_SCL_HIGH();
        delay_us(2);
        I2C2_SCL_LOW();
        delay_us(2);
    }
}

/* 读取一个字节 */
uint8_t I2C2_Read_Byte(uint8_t ack)
{
    unsigned char i, receive = 0;
    I2C2_SDA_IN();
    for(i = 0; i < 8; i++)
    {
        I2C2_SCL_LOW();
        delay_us(2);
        I2C2_SCL_HIGH();
        receive <<= 1;
        if(I2C2_SDA_READ())
            receive++;
        delay_us(1);
    }
    if(!ack)
        I2C2_NAck();
    else
        I2C2_Ack();
    return receive;
}

/* 写入多个字节 */
uint8_t I2C2_Write_nByte(uint8_t SlaveAddress, uint8_t REG_Address, uint16_t len, const uint8_t *buf)
{
    I2C2_Start();
    I2C2_Send_Byte((SlaveAddress << 1) | 0);
    if(I2C2_Wait_Ack())
    {
        I2C2_Stop();
        return 1;
    }
    I2C2_Send_Byte(REG_Address);
    if(I2C2_Wait_Ack())
    {
        I2C2_Stop();
        return 1;
    }
    while(len--)
    {
        I2C2_Send_Byte(*buf++);
        if(I2C2_Wait_Ack())
        {
            I2C2_Stop();
            return 1;
        }
    }
    I2C2_Stop();
    return 0;
}

/* 读取多个字节 */
uint8_t I2C2_Read_nByte(uint8_t SlaveAddress, uint8_t REG_Address, uint16_t len, uint8_t *buf)
{
    I2C2_Start();
    I2C2_Send_Byte((SlaveAddress << 1) | 0);
    if(I2C2_Wait_Ack())
    {
        I2C2_Stop();
        return 1;
    }
    I2C2_Send_Byte(REG_Address);
    if(I2C2_Wait_Ack())
    {
        I2C2_Stop();
        return 1;
    }
    I2C2_Start();
    I2C2_Send_Byte((SlaveAddress << 1) | 1);
    if(I2C2_Wait_Ack())
    {
        I2C2_Stop();
        return 1;
    }
    while(len)
    {
        if(len == 1)
            *buf = I2C2_Read_Byte(0);
        else
            *buf = I2C2_Read_Byte(1);
        buf++;
        len--;
    }
    I2C2_Stop();
    return 0;
}


