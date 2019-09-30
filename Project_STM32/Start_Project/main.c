#include <stm32f4xx_rcc.h>
#include <stdio.h>
#include "DHT22.h"
#include "YL69.h"
#include "BH1750.h"
#include "LCD.h"
#include "ESP8266.h"


int main(void) {	
    //ESP8266_Str* ESP8266 = (ESP8266_Str*)calloc(1, sizeof(ESP8266_Str));
    ESP8266_Str ESP8266;
    ESP8266_Result ret;
    TIM_TimeBaseInitTypeDef  TIM_TimeBase;
    Init_TIM2(TIM_TimeBase);
    TIM_Cmd(TIM2, ENABLE);
    Test_Interrupt();
    Init_Debug_GPIO();
    Debug_init_UART_Config(USART6, 9600);
    //Transmit_UART(USART6, (uint8_t*)"Beautiful weather", strlen("Beautiful weather"));
    ret = ESP8266_Init(&ESP8266, 115200);
    if (ret == ESP8266_OK) {
        Transmit_UART(USART6, (uint8_t*)"Initialization ESP8266 successfully\n", (strlen)("Initialization ESP8266 successfully\n"));
        //GPIO_SetBits(GPIOD, GPIO_Pin_15);
    } 
    ESP8266_Setting_WebServer(&ESP8266, "D-Link_DIR-612", "nhatro2129", 80);
    //GPIOD->ODR = (uint16_t)ESP8266.AP_IP[0];
    while(1) {
    }
		//return 0;
}

