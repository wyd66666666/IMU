/******************** (C) COPYRIGHT 2017 ANO Tech ********************************
 * ����    �������ƴ�
 * ����    ��www.anotc.com
 * �Ա�    ��anotc.taobao.com
 * ����QȺ ��190169595
 * ����    ���ɿس�ʼ��
**********************************************************************************/
#include "Drv_BSP.h"
#include "Drv_led.h"
#include "Drv_Timer.h"
#include "Drv_Uart.h"
#include "Drv_Timer.h"

u8 All_Init()
{
	DrvSysInit();
	//��ʱ
	MyDelayMs(100);
	//LED���ܳ�ʼ��
	DvrLedInit();
	MyDelayMs(100);
	//���ڳ�ʼ������������Ϊ������
	DrvUart1Init(900000);
	//DrvUart2Init(500000);
	//DrvUart3Init(500000);
	//DrvUart4Init(500000);
	//DrvUart5Init(500000);
	MyDelayMs(100);
	//��ʼ����ʱ�ж�
	DrvTimerFcInit();
	//��ʼ����ɣ�����1
	return (1);
}

/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
