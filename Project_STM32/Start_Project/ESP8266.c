#include "ESP8266.h"
void Init_ESP_GPIO(void) {
	  //Enable Clock for PORT A
	  GPIO_InitTypeDef GPIO_UART_ESP;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
		//Enable System  Config Controller Clock
	  GPIO_UART_ESP.GPIO_Pin = GPIO_Pin_9;
    GPIO_UART_ESP.GPIO_Speed = GPIO_Speed_100MHz;
	  GPIO_UART_ESP.GPIO_OType = GPIO_OType_PP;
	  GPIO_UART_ESP.GPIO_PuPd  = GPIO_PuPd_DOWN;
	  GPIO_UART_ESP.GPIO_Mode  = GPIO_Mode_AF; 
	  GPIO_Init(GPIOA, &GPIO_UART_ESP);
	  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
}

void Transmit_UART_Enable(USART_TypeDef* USARTx,  FunctionalState NewState) {
	 if (NewState != DISABLE) {
		  USARTx->CR1 |= UART_TRANS_ENABLE;
	 }
	 else {
		  USARTx->CR1 &= (uint16_t)~((uint16_t)UART_TRANS_ENABLE);
	 }
}
void Init_UART_Config(void) {
	 USART_InitTypeDef Init_UART_Str;
	 USART_Cmd(USART1, ENABLE);
	 USART_StructInit(&Init_UART_Str);
	 Transmit_UART_Enable(USART1, ENABLE);
	 USART_SendData (USART1, 0x41);
   while (!USART_GetFlagStatus(USART1, USART_FLAG_TC)) {
	 }
	 USART_Cmd(USART1, DISABLE);
}

