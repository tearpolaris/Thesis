#ifndef __BH1750_H__
#define __BH1750_H__
#include <stdio.h>
#include <stm32f4xx.h>
#include <stm32f4xx_i2c.h>
#include <misc.h>

#define ADDR_BH1750 0x46 //0x23 << 1
void Config_Clock_AHB_Multi_10(void);
void Init_BH1750_I2C(void);
void I2C_Start(I2C_TypeDef* I2Cx, uint8_t ADDRESS, uint8_t direction);
void Test_PullUp(void);

void I2C_Write(I2C_TypeDef* I2Cx, uint8_t DATA);
/*======== ham nhan byte cao SLave=====================*/
uint8_t Nhan_Bytecao(I2C_TypeDef* I2Cx);

/*======== ham doc byte thap SLAVE =======*/

uint8_t Nhan_Bytethap(I2C_TypeDef* I2Cx);

/*======= ham dieu kien STOP=======*/
void I2C_Stop(I2C_TypeDef* I2Cx);

/*=========ham delay=====*/
void Delay(uint32_t time);

/*==========ham MASTER truyen=======*/
void BH1750_Init(void);
 
/*===========ham MASTER NHAN=========*/
uint16_t BH1750_Read(void);
#endif
