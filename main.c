#include <stm32f4xx_rcc.h>
#include <stdio.h>
#include "DHT22.h"


int main(void) {	
 
	  //uint8_t check_sum;

	 Init_Read_DHT22();
		TIM_SetCounter(TIM2, 0);
		
	  //GPIOC->ODR = dat_humid;
		//GPIOB->ODR = dat_temp;
				
		
		
	  //TIM_GenerateEvent(TIM2, TIM_EventSource_Update);
	  printf("Super hero\n");
     //GPIOD->BSRRL = GPIO_Pin_12;
	  while(1) {
		}
	 
		//return 0;
}

