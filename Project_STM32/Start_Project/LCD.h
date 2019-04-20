#ifndef __LCD_H__
#define __LCD_H__

#include <stdio.h>
#include <stm32f4xx.h>
#include <misc.h>
#include <string.h>

#define RS   GPIO_Pin_0
#define RW   GPIO_Pin_1  
#define EN   GPIO_Pin_2 //Set 1 for Parallel Mode
#define RST  GPIO_Pin_7

#define DB0 		 GPIO_Pin_0
//0EH first DISPLAY ON, Cusor on
//06 second move right no shift 
//RS =0, RW = 0, DB = C0 //8-bit interface, basic instruction
//RS =  RW = 0, DB = 80H //DDRAM address
//RS = 0, RW =1, DB7 = BF, DB6:DB0 = AC
void Init_LCD_GPIO(void);
void Init_LCD(void);
void Read_BusyFlag_AC(void);
void Write_Port_Data (GPIO_TypeDef* GPIOx, uint8_t data);
void Write_Data(uint8_t data);
void Display_Content(char* content);
uint8_t Read_Data(void);
void Write_Instruction(uint8_t instruct);
void Set_Line1(void);
void Set_Line2(void);
void Display_Clear(void);
void delay(uint32_t micros);
//void Set_Line3(void);
//void Set_Line4(void);
#endif
