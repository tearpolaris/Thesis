#ifndef __DHT22_H__
#define __DHT22_H__

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stm32f4xx.h>
#include <stm32f4xx_syscfg.h>
#include <stm32f4xx_tim.h> //Timer Header File
#include <stm32f4xx_exti.h>
#include <misc.h>

#define DATA_DHT22 GPIOA->IDR & 0x2
#define COUNT      TIM2->CNT
typedef enum TYPE_RET_DHT22 {DHT22_OK, //DHT22 hoat dong binh thuong
                             ERROR_RESPONSE_LOW, //Time to start response LOW is too long
                             ERROR_RESPONSE_UP, //Time to start response HIGH is too long
                             ERROR_TRANSMISSION_START, //Time to start transmission start bit is too long
                             ERROR_BIT_DATA, //Time to start bit data is too long
                             ERROR_CHECK_SUM}
             TYPE_DAT_DHT22 ;

//********** Delay in micro second **********//
static __INLINE void Delay(uint32_t micros);  
//__INLINE void Delay(uint32_t micros); 														 
//******************************************//

//********** Dem thoi gian TIMEOUT**********//
__INLINE bool CountTimeOut(uint32_t count);                   
//******************************************//

int fputc(int ch, FILE *f);

//***** Khoi tao GPIO Port A, DATA la Pin A1 *****//
void Init_DHT22_GPIO(GPIO_InitTypeDef* GPIO_DHT22);
//************************************************//

//*********** Khoi tao TIMER 2 Pin A1 ************//
void Init_TIM2(TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure);
//*************************************************//

void Init_Interrupt(void);
void Init_Output_Compare(TIM_OCInitTypeDef  TIM_OC);
void Test_Interrupt(void);

//********* Handler Interrupt External Trigger 1 *************//
//void EXTI1_IRQHandler(void);                      
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
void Initialization_General_DHT22(void);
TYPE_DAT_DHT22 Init_Read_DHT22(void);
void DHT22_Get_Humid_Temp(uint16_t* dat_humid, uint16_t* dat_temp);                 
//********************************************//

//********* Convert interget to string ********//
//void Convert_Data_LCD(char convert_data[8], uint16_t raw_data);
//*********************************************//

//*********** Display Info Humid and Temperature **********//
//void Display_Info_Humid_Temp(void);
//********************************************************//

//************************ Code reference  ****************************//
 //GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM2);//Config pin AF as alternative function TIM2, TIM2 channel 1
 
#endif 
