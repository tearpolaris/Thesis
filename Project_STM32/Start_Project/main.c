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
    uint16_t humid, temp;
    //TIM_TimeBaseInitTypeDef  TIM_TimeBase;
    //Init_TIM2(TIM_TimeBase);
    //TIM_Cmd(TIM2, ENABLE);
    Test_Interrupt();
    //Init_Debug_GPIO();
    //Debug_init_UART_Config(USART6, 9600);
    //Transmit_string_UART(USART6, "Beautiful weather");
    //ret = ESP8266_Init(&ESP8266, 115200);
    //if (ret == ESP8266_OK) {
    //    Transmit_UART(USART6, (uint8_t*)"\rInitialization ESP8266 successfully\n", (strlen)("Initialization ESP8266 successfully\n"));
    //}
    //else {
    //    return -1;
    //} 
    //ESP8266_Setting_WebServer(&ESP8266, "D-Link_DIR-612", "nhatro2129", 80);
    //Handle_Request_Browser(&ESP8266);    
    //GPIO_SetBits(GPIOD, GPIO_Pin_12);
    Initialization_General_DHT22();
    DHT22_Get_Humid_Temp(&humid, &temp);
    Init_General_Setting_YL69();
    ret = ESP8266_Init(&ESP8266, 115200);
    if (ret == ESP8266_OK) {
        //Transmit_UART(USART6, (uint8_t*)"\rInitialization ESP8266 successfully\n", (strlen)("Initialization ESP8266 successfully\n"));
    }
    else {
        return -1;
    } 
    ESP8266_Setting_WebServer(&ESP8266, "D-Link_DIR-612", "nhatro2129", 80);
    Handle_Request_Browser(&ESP8266);  
    while(1) {
    }
		return 0;
}

