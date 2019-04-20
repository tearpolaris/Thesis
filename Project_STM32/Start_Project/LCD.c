#include "LCD.h"
void delay(uint32_t micros) {
	volatile uint32_t timer = TIM2->CNT;
	do {
		/* Count timer ticks */
		while ((TIM2->CNT - timer) == 0);
		/* Increase timer */
		timer = TIM2->CNT;
		/* Decrease microseconds */
	} while (--micros);
}
void Init_LCD_GPIO(void) {
   GPIO_InitTypeDef GPIO_LCD;
	  //Enable Clock for PORT A
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
		//Enable System  Config Controller Clock
	  GPIO_LCD.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1| GPIO_Pin_2 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_LCD.GPIO_Speed = GPIO_Speed_100MHz;
	  GPIO_LCD.GPIO_OType = GPIO_OType_PP;
	  GPIO_LCD.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	  GPIO_LCD.GPIO_Mode  = GPIO_Mode_OUT;
	
	  GPIO_Init(GPIOC, &GPIO_LCD);
	  GPIO_LCD.GPIO_Pin = 0xFF | GPIO_Pin_12 | GPIO_Pin_13;//Pin 0 -> Pin 7
	  GPIO_Init(GPIOB, &GPIO_LCD);
	  GPIO_SetBits(GPIOC, GPIO_Pin_6);
		//GPIO_SetBits(GPIOC, GPIO_Pin_7);
    //GPIO_SetBits(GPIOB, GPIO_Pin_7);
}
void Write_Port_Data (GPIO_TypeDef* GPIOx, uint8_t data) {
	GPIOx->ODR = (uint16_t)data;
}
void Read_BusyFlag_AC(void) {
	//uint8_t data;
	GPIO_InitTypeDef GPIO_LCD;
	GPIO_LCD.GPIO_Pin = 0xFF;//Pin 0 -> Pin 7
	GPIO_LCD.GPIO_Mode  = GPIO_Mode_IN;
	GPIO_LCD.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	//GPIO_LCD.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOB, &GPIO_LCD);
	GPIO_ResetBits(GPIOC, RS);
	GPIO_SetBits(GPIOC, RW);
	
	while ((GPIOB->IDR & 0x80) >> 7) {
	}
}
void Write_Data(uint8_t data) {
	GPIO_SetBits(GPIOC, RS);
	GPIO_ResetBits(GPIOC, RW);
	delay(10);
	Write_Port_Data(GPIOB, data);
	GPIO_SetBits(GPIOC, EN);

	//Read_BusyFlag_AC();
	delay(50);
	GPIO_ResetBits(GPIOC, EN);
}
uint8_t Read_Data(void) {
	uint8_t data;
	GPIO_SetBits(GPIOC, RS);
	GPIO_SetBits(GPIOC, RW);
	GPIO_SetBits(GPIOC, EN);
	data = (uint8_t)(GPIOB->IDR & 0xFF);
	GPIO_ResetBits(GPIOC, EN);
	return data;
}
void Write_Instruction(uint8_t instruct) {
	GPIO_ResetBits(GPIOC, RS|RW);
	delay(50);
	Write_Port_Data (GPIOB, instruct);
	GPIO_SetBits(GPIOC, EN);
	delay(50);
	GPIO_ResetBits(GPIOC, EN);
}

void Init_LCD(void) {
	delay(50000);
	GPIO_ResetBits(GPIOC, RST);
  delay(20);
	GPIO_SetBits(GPIOC, RST);
	delay(10);

	Write_Instruction(0x30);
	delay(200);
	Write_Instruction(0x30);
	delay(200);    
	Write_Instruction(0x0C);
	delay(200);
	Write_Instruction(0x01);
	delay(15000);
	Write_Instruction(0x06);
	delay(100);
}

void Display_Content(char* content) {
	 for (int i=0; i < strlen(content); i++) {
		 Write_Data((uint8_t)content[i]);
	 }
}

void Set_Line1(void) {
	Write_Instruction(0x80);
}

void Set_Line2(void) {
	Write_Instruction(0x90);
}

void Display_Clear(void) {
	Write_Instruction(0x01);
}

//void Set_Line3(void) {
//}
//
//void Set_Line4(void) {
//}
//Select PSB = 1, connect to VCC 8-bit parallel
//Connect RS = PC0, RW = PC1, ENABLE = PC2
//Connect DB[0:7] to GPIOD[0:7]
