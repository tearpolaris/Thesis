#include "YL69.h"
ADC_CommonInitTypeDef* ADC_Com_Str;
ADC_InitTypeDef*       ADC_Str;
GPIO_InitTypeDef GPIO_YL69;


uint16_t Read_YL69_A0 (void) {
   Init_YL69_GPIO();
	 //Config Pin A0 - ADC1 channel 0
	 ADC_CommonStructInit(ADC_Com_Str);//Init ADC clock = PHB2/2
   ADC_CommonInit(ADC_Com_Str);
   ADC_StructInit(ADC_Str);//Init Scan mode disable - Single conversion; Disable continous mode
	 //Disable external Trigger, Data 12-bit, Right Align, Number of Conversion 1
	 ADC_Cmd(ADC1, ENABLE);//Wake up from Power-down
	 ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_84Cycles);

	 ADC_SoftwareStartConv(ADC1);
	 while (!Get_End_Conversion_Flag(ADC1)) {
	 }
	 return ADC_GetConversionValue(ADC1);
}

void Init_YL69_GPIO(void) {
	  //Enable Clock for PORT A
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
		//Enable ADC1 Clock
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	  GPIO_YL69.GPIO_Pin = GPIO_Pin_0;//ADC1 channel 0
	  //GPIO_YL69.GPIO_OType = GPIO_OType_PP;
	  //GPIO_YL69.GPIO_PuPd  = GPIO_PuPd_DOWN;
	  GPIO_YL69.GPIO_Mode  = GPIO_Mode_AN ;	 
	  GPIO_Init(GPIOA, &GPIO_YL69);
}

uint8_t Get_End_Conversion_Flag(ADC_TypeDef *  ADCx) {
		return ((ADCx->SR & 0x02) >> 1);
}

float Percent_Soil_YL69 (uint16_t Conversion_Value) {
	  //Digital Code = V_out/V_ref*(2^N -1)
	  float Analog_Value = (float)(Conversion_Value)*100/4095;//12-bit ADC 3300/4095 = 
	  printf("Gia tri dien tro la: %f\n", Analog_Value);
	  return (100 - Analog_Value);//Range 0 ~ 1023
}
