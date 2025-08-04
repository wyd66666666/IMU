#ifndef __DELAY_H
#define __DELAY_H

#include "main.h"

void delay_us(uint32_t us);
uint8_t DWT_Delay_Init(void);
void delay_ms(uint32_t ms);

#endif
