#ifndef __YL69_H__
#define __YL69_H__

#include <stm32f4xx.h>
#include <misc.h>

#define REG_START_FLAG 0x10
uint16_t Read_YL69_A0 (void);
void Init_YL69_GPIO(void);
uint8_t Get_End_Conversion_Flag(ADC_TypeDef *  ADCx);

#endif