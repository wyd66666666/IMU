/******************** (C) COPYRIGHT 2017 ANO Tech ********************************
 * 作者    ：匿名科创
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
 * 描述    ：飞控初始化
**********************************************************************************/
#include "Drv_BSP.h"
#include "Drv_led.h"
#include "Drv_Timer.h"
#include "Drv_Uart.h"
#include "Drv_Timer.h"

u8 All_Init()
{
	DrvSysInit();
	//延时
	MyDelayMs(100);
	//LED功能初始化
	DvrLedInit();
	MyDelayMs(100);
	//串口初始化，函数参数为波特率
	DrvUart1Init(900000);
	//DrvUart2Init(500000);
	//DrvUart3Init(500000);
	//DrvUart4Init(500000);
	//DrvUart5Init(500000);
	MyDelayMs(100);
	//初始化定时中断
	DrvTimerFcInit();
	//初始化完成，返回1
	return (1);
}

/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
