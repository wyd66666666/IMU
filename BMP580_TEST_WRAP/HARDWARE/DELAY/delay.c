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
// ��ʼ��DWT
uint8_t DWT_Delay_Init(void)
{
    // ʹ��DWT
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    // ��λ������
    DWT->CYCCNT = 0;
    // ʹ��CYCCNT
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    // ����Ƿ�ʹ�ܳɹ�
    if (DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk)
        return 0; // �ɹ�
    else
        return 1; // ʧ��
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

