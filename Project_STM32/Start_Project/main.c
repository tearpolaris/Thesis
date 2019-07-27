#include <stm32f4xx_rcc.h>
#include <stdio.h>
#include "DHT22.h"
#include "YL69.h"
#include "BH1750.h"
#include "LCD.h"
#include "ESP8266.h"

int main(void) {	
	//char convert_data[8];
  char *str = "AT\r\n";
	TIM_TimeBaseInitTypeDef  TIM_TimeBase;
	//TIM_OCInitTypeDef  TIM_OC;
	Init_TIM2(TIM_TimeBase);
	//Init_Output_Compare(TIM_OC);
	TIM_Cmd(TIM2, ENABLE);
  Test_Interrupt();
  Init_USART1_RXNE_Interrupt(USART1);
	//GPIO_SetBits(GPIOD, GPIO_Pin_13);
	Init_ESP_GPIO();
	Init_UART_Config();
	//USART_SendData (USART1, 0x41);
  Transmit_UART(USART1, (uint8_t*)str, strlen(str));
	GPIO_SetBits(GPIOD, GPIO_Pin_12);
	while(1) {
	}
		//return 0;
}

