#ifndef __DHT22_H__
#define __DHT22_H__

#include <stdio.h>
#include <stdbool.h>
#include <stm32f4xx.h>
#include <stm32f4xx_syscfg.h>
#include <stm32f4xx_tim.h> //Timer Header File
#include <stm32f4xx_exti.h>
#include <misc.h>

#define DATA_DHT22 GPIOA->IDR & 0x2
#define COUNT      TIM2->CNT

//********** Delay in micro second **********//
static __INLINE void Delay(uint32_t micros);                    
//******************************************//

//********** Dem thoi gian TIMEOUT**********//
__INLINE bool CountTimeOut(uint32_t count);                   
//******************************************//

int fputc(int ch, FILE *f);

//***** Khoi tao GPIO Port A, DATA la Pin A1 *****//
void Init_DHT22_GPIO(void);
//************************************************//

//*********** Khoi tao TIMER 2 Pin A1 ************//
void Init_TIM2(TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure);
//*************************************************//

void Init_Interrupt(void);
void Init_Output_Compare(TIM_OCInitTypeDef  TIM_OC);
void Test_Interrupt(void);

//********* Handler Interrupt External Trigger 1 *************//
void EXTI1_IRQHandler(void);                      
//************************************************************//
	
//********** Get 40-bit Data DHT22 **********//
void Get_Data_DHT22(void);                     
//******************************************//

//********** Check sum Data DHT22 **********//
bool Check_Sum_DHT22(void);                   
//******************************************//

//********** Check sum Data DHT22 **********//
void Reset_Data_DHT22(void);                 
//******************************************//

//******** Main Init and Read Data DHT22 ******//
void Init_Read_DHT22(void);                 
//********************************************//

//************************ Code reference  ****************************//
 //GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM2);//Config pin AF as alternative function TIM2, TIM2 channel 1
 
#endif 
