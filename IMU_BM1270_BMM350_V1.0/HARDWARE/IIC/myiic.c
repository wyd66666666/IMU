#include "myiic.h"
#include "delay.h"

// SCL���ƺ���
void Set_SCL(uint8_t val)
{
    if (val)
        GPIOA->BSRR = GPIO_PIN_15;  // �ø�
    else
        GPIOA->BRR = GPIO_PIN_15;   // �õ�
}

// SDA���ƺ���
void Set_SDA(uint8_t val)
{
    if (val)
        GPIOB->BSRR = GPIO_PIN_7;   // �ø�
    else
        GPIOB->BRR = GPIO_PIN_7;    // �õ�
}

//IIC��ʼ��
void IIC_Init(void)
{
    GPIO_InitTypeDef GPIO_Initure;
    
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    //PA15��ʼ������ (SCL)
    GPIO_Initure.Pin = GPIO_PIN_15;
    GPIO_Initure.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_Initure.Pull = GPIO_PULLUP;
    GPIO_Initure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_Initure);
    
    //PB7��ʼ������ (SDA)
    GPIO_Initure.Pin = GPIO_PIN_7;
    GPIO_Initure.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_Initure.Pull = GPIO_PULLUP;
    GPIO_Initure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_Initure);
    
    Set_SDA(1);
    Set_SCL(1);
}

//����IIC��ʼ�ź�
void IIC_Start(void)
{
    SDA_OUT_1();     //sda�����
    Set_SDA(1);
    Set_SCL(1);
    delay_us(4);
    Set_SDA(0);      //START:when CLK is high,DATA change form high to low 
    delay_us(4);
    Set_SCL(0);      //ǯסI2C���ߣ�׼�����ͻ�������� 
}

//����IICֹͣ�ź�
void IIC_Stop(void)
{
    SDA_OUT_1();     //sda�����
    Set_SCL(0);
    Set_SDA(0);      //STOP:when CLK is high DATA change form low to high
    delay_us(4);
    Set_SCL(1);
    Set_SDA(1);      //����I2C���߽����ź�
    delay_us(4);
}

//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
uint8_t IIC_Wait_Ack(void)
{
    uint8_t ucErrTime=0;
    
    SDA_IN_1();      //SDA����Ϊ����
    Set_SDA(1);      // �ͷ�SDA��
    delay_us(1);
    Set_SCL(1);      // SCL�ߵ�ƽ
    delay_us(1);
    
    while(READ_SDA_1)  // ��ȡSDA״̬
    {
        ucErrTime++;
        if(ucErrTime>250)
        {
            IIC_Stop();
            return 1;
        }
    }
    Set_SCL(0);      //ʱ�����0
    return 0;
}

//����ACKӦ��
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

//������ACKӦ��
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

//IIC����һ���ֽ�
void IIC_Send_Byte(uint8_t txd)
{
    uint8_t t;
    SDA_OUT_1();
    Set_SCL(0);      //����ʱ�ӿ�ʼ���ݴ���
    
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

//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK
uint8_t IIC_Read_Byte(unsigned char ack)
{
    unsigned char i,receive=0;
    SDA_IN_1();      //SDA����Ϊ����
    
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
        IIC_NAck();  //����nACK
    else
        IIC_Ack();   //����ACK
        
    return receive;
}



uint8_t IIC_Write_1Byte(uint8_t SlaveAddress,uint8_t REG_Address,uint8_t REG_data)
//uint8_t MPU_Write_Byte(uint8_t reg,uint8_t data) 				 
{ 
    IIC_Start(); 
	IIC_Send_Byte((SlaveAddress<<1)|0);//����������ַ+д����	
	if(IIC_Wait_Ack())	//�ȴ�Ӧ��
	{
		IIC_Stop();		 
		return 1;		
	}
    IIC_Send_Byte(REG_Address);	//д�Ĵ�����ַ
    IIC_Wait_Ack();		//�ȴ�Ӧ�� 
	IIC_Send_Byte(REG_data);//��������
	if(IIC_Wait_Ack())	//�ȴ�ACK
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
	IIC_Send_Byte((SlaveAddress<<1)|0);//����������ַ+д����	
	IIC_Wait_Ack();		//�ȴ�Ӧ�� 
    IIC_Send_Byte(REG_Address);	//д�Ĵ�����ַ
    IIC_Wait_Ack();		//�ȴ�Ӧ��
    IIC_Start();
	IIC_Send_Byte((SlaveAddress<<1)|1);//����������ַ+������	
    IIC_Wait_Ack();		//�ȴ�Ӧ�� 
	data=IIC_Read_Byte(0);//��ȡ����,����nACK 
    IIC_Stop();			//����һ��ֹͣ���� 
	return data;		
}


// IICдn�ֽ�����
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

// IIC��n�ֽ�����
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




