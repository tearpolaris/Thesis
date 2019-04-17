#include "LCD.h"

void Write_Port_Data (uint8_t data) {
	GPIOD->ODR = (uint16_t)data;
}
void Read_BusyFlag_AC(void) {
	uint8_t data;
	GPIO_ResetBits(GPIOC, RS);
	GPIO_SetBits(GPIOC, RW);
	while ((GPIOD->IDR & 0x80) >> 7) {
	}
}
void Write_Data(void) {
	GPIO_SetBits(GPIOC, RS);
	GPIO_ResetBits(GPIOC, RW);
}
uint8_t Read_Data(void) {
	uint8_t data;
	GPIO_SetBits(GPIOC, RS);
	GPIO_SetBits(GPIOC, RW);
	GPIO_SetBits(GPIOC, EN);
	data = (uint8_t)(GPIOD->IDR & 0xFF);
	GPIO_ResetBits(GPIOC, EN);
	return data;
}
void Write_Instruction(uint8_t instruct) {
	GPIO_ResetBits(GPIOC, RS);
	GPIO_ResetBits(GPIOC, RW);
	GPIO_SetBits(GPIOC, EN);
	Write_Port_Data(instruct);
	Read_BusyFlag_AC();
	GPIO_ResetBits(GPIOC, EN);
}

void Init_LCD() {
	 Write_Instruction(0x0E);
	 Write_Instruction(0x30);
	 Write_Instruction(0x06);
	 
}