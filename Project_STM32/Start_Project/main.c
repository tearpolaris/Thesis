#include <stm32f4xx_rcc.h>
#include <stdio.h>
#include "DHT22.h"
#include "YL69.h"
#include "BH1750.h"
#include "LCD.h"


int main(void) {	
	char convert_data[8];
	TIM_TimeBaseInitTypeDef  TIM_TimeBase;
	//TIM_OCInitTypeDef  TIM_OC;
	Init_TIM2(TIM_TimeBase);
	//Init_Output_Compare(TIM_OC);
	TIM_Cmd(TIM2, ENABLE);
	Test_Interrupt();
		Init_LCD_GPIO();
	Init_LCD();
	Set_Line1();
	 //Display_Content("Tram Anh 9 phut");
	Convert_Data_LCD(convert_data, 357);
	Display_Content(convert_data);
	Write_Data(0x20);
	Write_Data(0x9);
	Write_Data(0x43);
	
	Set_Line2();
	Display_Content("Do am: 68");
		Write_Data(0x25);
	  //Display_Content();
	//Write_Data(0x25);
	//printf("%s\n", convert_data);
	//Write_Data(0x31);
	//Init_LCD_GPIO();
	//GPIO_SetBits(GPIOB, GPIO_Pin_0);
	//uint16_t Value_A0;
	//float Moisture;
	//Init_Read_DHT22();
	//Config_Clock_AHB_Multi_10();
	//Test_Interrupt();
	//Value_A0 = Read_YL69_A0();
	//printf("Value of analog is: %d\n", Value_A0);
	//Moisture = Percent_Soil_YL69 (Value_A0);
	//printf("Value of moisture is: %f\n", Moisture);
	GPIOD->BSRRL = GPIO_Pin_12;
	  //GPIOC->ODR = dat_humid;
		//GPIOB->ODR = dat_temp;
	 //Test_PullUp();
	 //GPIO_SetBits(GPIOC, GPIO_Pin_0);
	/*
	 printf("GPIOB is %X\n", GPIOB->IDR);
   
	printf("Status of busy BIT is: %X\n", I2C1->SR2);
	Init_BH1750_I2C();
	printf("Status of busy BIT is: %X\n", I2C1->SR2);

	printf("Status of busy BIT is: %X\n", I2C1->SR2);
	printf("Status of busy BIT is: %X\n", I2C1->SR2);
	BH1750_Init();
	BH1750_Read();
	GPIO_SetBits(GPIOD, GPIO_Pin_15);
	*/
	//I2C_Start(I2C1, ADDR_BH1750, I2C_Direction_Transmitter);
	while(1) {
	}
		//return 0;
}

