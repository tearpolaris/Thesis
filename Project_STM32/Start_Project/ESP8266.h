//OVER8 = 0 => Oversampling by 16, giam sai so do lech clock
//ONEBIT = 0 => Lay mau 3 lan tai trung tam cua bit
#include <stm32f4xx_usart.h>

#define UART_TRANS_ENABLE 0x08
void Init_UART_Config(void);
void Transmit_UART_Enable(USART_TypeDef* USARTx,  FunctionalState NewState);
void Clear_UART_TxE(USART_TypeDef* USARTx);
void Init_ESP_GPIO(void);