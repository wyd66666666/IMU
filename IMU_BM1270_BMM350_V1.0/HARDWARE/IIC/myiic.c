#include "myiic.h"
#include "delay.h"

// SCL控制函数
void Set_SCL(uint8_t val)
{
    if (val)
        GPIOA->BSRR = GPIO_PIN_15;  // 置高
    else
        GPIOA->BRR = GPIO_PIN_15;   // 置低
}

// SDA控制函数
void Set_SDA(uint8_t val)
{
    if (val)
        GPIOB->BSRR = GPIO_PIN_7;   // 置高
    else
        GPIOB->BRR = GPIO_PIN_7;    // 置低
}

//IIC初始化
void IIC_Init(void)
{
    GPIO_InitTypeDef GPIO_Initure;
    
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    //PA15初始化设置 (SCL)
    GPIO_Initure.Pin = GPIO_PIN_15;
    GPIO_Initure.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_Initure.Pull = GPIO_PULLUP;
    GPIO_Initure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_Initure);
    
    //PB7初始化设置 (SDA)
    GPIO_Initure.Pin = GPIO_PIN_7;
    GPIO_Initure.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_Initure.Pull = GPIO_PULLUP;
    GPIO_Initure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_Initure);
    
    Set_SDA(1);
    Set_SCL(1);
}

//产生IIC起始信号
void IIC_Start(void)
{
    SDA_OUT_1();     //sda线输出
    Set_SDA(1);
    Set_SCL(1);
    delay_us(4);
    Set_SDA(0);      //START:when CLK is high,DATA change form high to low 
    delay_us(4);
    Set_SCL(0);      //钳住I2C总线，准备发送或接收数据 
}

//产生IIC停止信号
void IIC_Stop(void)
{
    SDA_OUT_1();     //sda线输出
    Set_SCL(0);
    Set_SDA(0);      //STOP:when CLK is high DATA change form low to high
    delay_us(4);
    Set_SCL(1);
    Set_SDA(1);      //发送I2C总线结束信号
    delay_us(4);
}

//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
uint8_t IIC_Wait_Ack(void)
{
    uint8_t ucErrTime=0;
    
    SDA_IN_1();      //SDA设置为输入
    Set_SDA(1);      // 释放SDA线
    delay_us(1);
    Set_SCL(1);      // SCL高电平
    delay_us(1);
    
    while(READ_SDA_1)  // 读取SDA状态
    {
        ucErrTime++;
        if(ucErrTime>250)
        {
            IIC_Stop();
            return 1;
        }
    }
    Set_SCL(0);      //时钟输出0
    return 0;
}

//产生ACK应答
void IIC_Ack(void)
{
    Set_SCL(0);
    SDA_OUT_1();
    Set_SDA(0);
    delay_us(2);
    Set_SCL(1);
    delay_us(2);
    Set_SCL(0);
}

//不产生ACK应答
void IIC_NAck(void)
{
    Set_SCL(0);
    SDA_OUT_1();
    Set_SDA(1);
    delay_us(2);
    Set_SCL(1);
    delay_us(2);
    Set_SCL(0);
}

//IIC发送一个字节
void IIC_Send_Byte(uint8_t txd)
{
    uint8_t t;
    SDA_OUT_1();
    Set_SCL(0);      //拉低时钟开始数据传输
    
    for(t=0;t<8;t++)
    {
        if((txd&0x80)>>7)
            Set_SDA(1);
        else
            Set_SDA(0);
        
        txd<<=1;
        delay_us(2);
        Set_SCL(1);
        delay_us(2);
        Set_SCL(0);
        delay_us(2);
    }
}

//读1个字节，ack=1时，发送ACK，ack=0，发送nACK
uint8_t IIC_Read_Byte(unsigned char ack)
{
    unsigned char i,receive=0;
    SDA_IN_1();      //SDA设置为输入
    
    for(i=0;i<8;i++)
    {
        Set_SCL(0);
        delay_us(2);
        Set_SCL(1);
        receive<<=1;
        if(READ_SDA_1)
            receive++;
        delay_us(1);
    }
    
    if (!ack)
        IIC_NAck();  //发送nACK
    else
        IIC_Ack();   //发送ACK
        
    return receive;
}



uint8_t IIC_Write_1Byte(uint8_t SlaveAddress,uint8_t REG_Address,uint8_t REG_data)
//uint8_t MPU_Write_Byte(uint8_t reg,uint8_t data) 				 
{ 
    IIC_Start(); 
	IIC_Send_Byte((SlaveAddress<<1)|0);//发送器件地址+写命令	
	if(IIC_Wait_Ack())	//等待应答
	{
		IIC_Stop();		 
		return 1;		
	}
    IIC_Send_Byte(REG_Address);	//写寄存器地址
    IIC_Wait_Ack();		//等待应答 
	IIC_Send_Byte(REG_data);//发送数据
	if(IIC_Wait_Ack())	//等待ACK
	{
		IIC_Stop();	 
		return 1;		 
	}		 
    IIC_Stop();	 
	return 0;
}
uint8_t IIC_Read_1Byte(uint8_t SlaveAddress,uint8_t REG_Address)
{
	uint8_t data;
	
    IIC_Start(); 
	IIC_Send_Byte((SlaveAddress<<1)|0);//发送器件地址+写命令	
	IIC_Wait_Ack();		//等待应答 
    IIC_Send_Byte(REG_Address);	//写寄存器地址
    IIC_Wait_Ack();		//等待应答
    IIC_Start();
	IIC_Send_Byte((SlaveAddress<<1)|1);//发送器件地址+读命令	
    IIC_Wait_Ack();		//等待应答 
	data=IIC_Read_Byte(0);//读取数据,发送nACK 
    IIC_Stop();			//产生一个停止条件 
	return data;		
}


// IIC写n字节数据
uint8_t IIC_Write_nByte(uint8_t SlaveAddress, uint8_t REG_Address, uint16_t len, const uint8_t *buf)
{	
	IIC_Start();
	IIC_Send_Byte(SlaveAddress<<1); 
	if(IIC_Wait_Ack())
	{
		IIC_Stop();
		return 1;
	}
	IIC_Send_Byte(REG_Address); 
	IIC_Wait_Ack();
	while(len--) 
	{
		IIC_Send_Byte(*buf++); 
		IIC_Wait_Ack();
	}
	IIC_Stop();
	return 0;
}

// IIC读n字节数据
uint8_t IIC_Read_nByte(uint8_t SlaveAddress, uint8_t REG_Address, uint16_t len, uint8_t *buf)
{	
	IIC_Start();
	IIC_Send_Byte(SlaveAddress<<1); 
	if(IIC_Wait_Ack())
	{
		IIC_Stop();
		return 1;
	}
	IIC_Send_Byte(REG_Address); 
	IIC_Wait_Ack();
	
	IIC_Start();
	IIC_Send_Byte(SlaveAddress<<1 | 0x01); 
	IIC_Wait_Ack();
	while(len) 
	{
		if(len == 1)
		{
			*buf = IIC_Read_Byte(0);
		}
		else
		{
			*buf = IIC_Read_Byte(1);
		}
		buf++;
		len--;
	}
	IIC_Stop();
	return 0;
}




