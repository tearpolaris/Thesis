#ifndef __YL69_H__
#define __YL69_H__

#include <stm32f4xx.h>
#include <misc.h>
#include <stdio.h>

#define REG_START_FLAG 0x10
#define ADC_VOLT 3300

uint16_t Read_YL69_A0 (void);
void Init_YL69_GPIO(void);
uint8_t Get_End_Conversion_Flag(ADC_TypeDef *  ADCx);
float Percent_Soil_YL69 (void);
void Init_General_Setting_YL69(void);

#endif

