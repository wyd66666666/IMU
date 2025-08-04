#include "delay.h"
/*
extern TIM_HandleTypeDef htim2;

void delay_us(uint16_t us)
{
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    HAL_TIM_Base_Start(&htim2);
    while(__HAL_TIM_GET_COUNTER(&htim2) < us);
    HAL_TIM_Base_Stop(&htim2);
}
*/
// 初始化DWT
uint8_t DWT_Delay_Init(void)
{
    // 使能DWT
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    // 复位计数器
    DWT->CYCCNT = 0;
    // 使能CYCCNT
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    // 检查是否使能成功
    if (DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk)
        return 0; // 成功
    else
        return 1; // 失败
}


void delay_us(uint32_t us)
{
    uint32_t cycles = (SystemCoreClock / 1000000) * us;
    uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < cycles);
}

void delay_ms(uint32_t ms)
{
    while (ms--)
    {
        delay_us(1000);
    }
}

