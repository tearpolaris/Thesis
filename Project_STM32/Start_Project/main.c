#include <stm32f4xx_rcc.h>
#include <stdio.h>
#include "DHT22.h"
#include "YL69.h"
#include "BH1750.h"
#include "LCD.h"
#include "ESP8266.h"


int main(void) {	
	  //char convert_data[8]
    //ESP8266_Str* ESP8266 = (ESP8266_Str*)calloc(1, sizeof(ESP8266_Str));
    ESP8266_Str ESP8266;
    ESP8266_Result ret;
    TIM_TimeBaseInitTypeDef  TIM_TimeBase;
    Init_TIM2(TIM_TimeBase);
    TIM_Cmd(TIM2, ENABLE);
    Test_Interrupt();
    ret = ESP8266_Init(&ESP8266, 115200);
    if (ret == ESP8266_OK) {
    } 
    while(1) {
    }
		//return 0;
}

