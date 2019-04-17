#ifndef __LCD_H__
#define __LCD_H__

#include <stdio.h>
#include <stm32f4xx.h>
#include <misc.h>

#define RS   GPIO_Pin_0
#define RW   GPIO_Pin_1  
#define EN   GPIO_Pin_2 //Set 1 for Parallel Mode

#define DB0 		 GPIO_Pin_0
//0EH first DISPLAY ON, Cusor on
//06 second move right no shift

//RS =0, RW = 0, DB = C0 //8-bit interface, basic instruction
//RS =  RW = 0, DB = 80H //DDRAM address
//RS = 0, RW =1, DB7 = BF, DB6:DB0 = AC
void Read_BusyFlag_AC(void);
void Write_Data(void);
uint8_t Read_Data(void);
void Write_Instruction(uint8_t instruct);
#endif
