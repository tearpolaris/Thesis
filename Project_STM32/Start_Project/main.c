#include <stm32f4xx_rcc.h>
#include <stdio.h>
#include "DHT22.h"


int main(void) {	
	 Init_Read_DHT22();
	  //GPIOC->ODR = dat_humid;
		//GPIOB->ODR = dat_temp;
	  while(1) {
		}
		//return 0;
}

