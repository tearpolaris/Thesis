#include "DHT22.h"
#include "LCD.h"

uint16_t dat_humid, dat_temp;
uint8_t toogle = 0;
uint8_t data[5] = { 0, 0, 0, 0, 0 };


GPIO_InitTypeDef GPIO_DHT22;
EXTI_InitTypeDef EXTI_Init_Struct;
TIM_OCInitTypeDef  TIM_Structure;
TIM_TimeBaseInitTypeDef  TIM_BaseStruct;
NVIC_InitTypeDef NVIC_Init_Struct;

static __INLINE void Delay(uint32_t micros) {
	volatile uint32_t timer = TIM2->CNT;
	do {
		/* Count timer ticks */
		while ((TIM2->CNT - timer) == 0);
		/* Increase timer */
		timer = TIM2->CNT;
		/* Decrease microseconds */
	} while (--micros);
}


__INLINE bool CountTimeOut(uint32_t count) {
	volatile uint32_t timer = TIM2->CNT;
	uint32_t timeout = 0;

	do {
		/* Count timer ticks */
		while ((TIM2->CNT - timer) == 0);

		/* Increase timer */
		timer = TIM2->CNT;
		timeout++;

		/* Decrease microseconds */
	} while (timeout < count);
	 
	return (timeout > count)? false:true;
}


int fputc(int ch, FILE *f)
{
  return(ITM_SendChar(ch));
}


void Init_DHT22_GPIO(void) {
	  //Enable Clock for PORT A
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
		//Enable System  Config Controller Clock
	  GPIO_DHT22.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_DHT22.GPIO_Speed = GPIO_Speed_100MHz;
	  GPIO_DHT22.GPIO_OType = GPIO_OType_PP;
	  GPIO_DHT22.GPIO_PuPd  = GPIO_PuPd_DOWN;
	  //GPIO_DHT22.GPIO_Mode  = GPIO_Mode_AF;
	  GPIO_DHT22.GPIO_Mode  = GPIO_Mode_OUT;
	 
	  GPIO_Init(GPIOA, &GPIO_DHT22);
}

void Init_Output_Compare(TIM_OCInitTypeDef  TIM_OC) {
	TIM_OC.TIM_OCMode     =  TIM_OCMode_PWM2;
	TIM_OC.TIM_OutputState =  TIM_OutputState_Enable;
  TIM_OC.TIM_Pulse  =  1000;
	//TIM_OC.TIM_Pulse  =  2;
	TIM_OC.TIM_OCPolarity = TIM_OCPolarity_High;
	
	//TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC2Init(TIM2, &TIM_OC);
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
	//TIM_ARRPreloadConfig(TIM2, DISABLE);
}
void Init_TIM2(TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure) {
	 //Enable clock peripheral TIM2
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	/* Enable clock for TIMx */	
	TIM_InternalClockConfig(TIM2);//Set clock source is Internal Clock CKNT
	 /* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 999;
	//TIM_TimeBaseStructure.TIM_Period = 2000;
	TIM_TimeBaseStructure.TIM_Prescaler = 0xF;//15 => Counter_Clock = 16MHz/(15 +1) = 1MHz
	//TIM_TimeBaseStructure.TIM_Prescaler = 0x3E7F;//15 => Counter_Clock = 16MHz/(15 +1) = 1MHz
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
}

void Init_Interrupt(void) {
	NVIC_InitTypeDef NVIC_Init_Struct;
	//Noi External Line toi GPIO Source
	//SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA , EXTI_PinSource1);
	
	//NIVC Config Channel
  NVIC_Init_Struct.NVIC_IRQChannel                   = EXTI1_IRQn;
	NVIC_Init_Struct.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_Init_Struct.NVIC_IRQChannelSubPriority        = 0x1;
	NVIC_Init_Struct.NVIC_IRQChannelCmd                = ENABLE;
	//Init Function
	NVIC_Init(&NVIC_Init_Struct);
	
	//External Mode Config
	EXTI_Init_Struct.EXTI_Mode = EXTI_Mode_Interrupt;//Mode Interrupt 
  EXTI_Init_Struct.EXTI_Line = EXTI_Line1;	
  EXTI_Init_Struct.EXTI_Trigger = EXTI_Trigger_Falling; 
	//EXTI_Init_Struct.EXTI_Trigger = EXTI_Trigger_Rising; 
  EXTI_Init_Struct.EXTI_LineCmd = ENABLE; 
	EXTI_Init(&EXTI_Init_Struct);
	
}

void Test_Interrupt(void) {
	GPIO_InitTypeDef GPIO_Interrupt;
	//Enable Clock for PORT D
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	
	GPIO_Interrupt.GPIO_Pin = 0xFFFF;//LED
  GPIO_Interrupt.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Interrupt.GPIO_OType = GPIO_OType_PP;
	GPIO_Interrupt.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Interrupt.GPIO_Mode  = GPIO_Mode_OUT;
	
	GPIO_Init(GPIOD, &GPIO_Interrupt);
	//GPIO_Init(GPIOC, &GPIO_Interrupt);
	//GPIO_Init(GPIOB, &GPIO_Interrupt);
	}

//********* Handler Interrupt External Trigger 1 *************//
//                                                            //
//************************************************************//
	
void EXTI1_IRQHandler(void) {
	  if (EXTI_GetITStatus(EXTI_Line1) != RESET) {
			  if (toogle == 1) {
				    GPIOD->BSRRL = GPIO_Pin_12;
			      EXTI_Init_Struct.EXTI_LineCmd = DISABLE;//Disable external line 1 no interrupt anymore
				    EXTI_Init(&EXTI_Init_Struct);
					  GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, 0);
			      GPIO_DHT22.GPIO_Mode  = GPIO_Mode_IN;
			      GPIO_DHT22.GPIO_PuPd  = GPIO_PuPd_UP;
					
			      GPIO_Init(GPIOA, &GPIO_DHT22);
			      GPIOD->BSRRL = GPIO_Pin_13;
				} 	   
        /* Clear interrupt flag */ 
        EXTI_ClearITPendingBit(EXTI_Line1);
			  toogle++;
    }
    /* Make sure that interrupt flag is set */	    
}

////Lay du lieu phan hoi tu DHT22
void Get_Data_DHT22(void) {
		uint32_t count_time;
	  unsigned char i;
	
	  count_time = 0;	
		for (i = 0; i < 40; i++) { //40-bit data		   
		    while(1) {
		    	//if ((DATA_DHT22) == 0x2) {
					if (((DATA_DHT22) == 0x2) && (count_time == 0)) {
						  //break;
						  TIM_SetCounter(TIM2, 0);
		    		  count_time++;
		    	}
		    	if ((count_time > 0) && ((DATA_DHT22) == 0)) {
						 count_time = 0;
						 break;
						 //printf("Error\n");
						 //return;
					}
				}
				data[i/8] <<= 1;
			  //if (count_time > 20 ) {
				if (COUNT > 30) {
				  data[i/8] |= 1;
				}
		}
}

bool Check_Sum_DHT22(void) {
	return (data[4] == (data[0] + data[1] + data[2] + data[3]))? true: false;
}

void Reset_Data_DHT22(void) {
	for (int i=0; i < 5; i++) {
			data[i] = 0;
	}
}

TYPE_DAT_DHT22 Init_Read_DHT22(void) {
	  uint32_t count_time_out = 0, ack_bit;
	
	  Init_TIM2(TIM_BaseStruct);
	  Test_Interrupt();
	  TIM_Cmd(TIM2, ENABLE);
		Delay(1000000);
	  Delay(1000000);
		Init_DHT22_GPIO();
	
		//Keo xuong 1ms
	  GPIO_ResetBits(GPIOA, GPIO_Pin_1);
	  Delay(1000);
	
		//Keo len 30us 
	  GPIO_SetBits(GPIOA, GPIO_Pin_1);
	  Delay(30);
	
	  //Giai phong bus cho DHT22 dieu khien
		GPIO_DHT22.GPIO_Mode  = GPIO_Mode_IN;
		GPIO_Init(GPIOA, &GPIO_DHT22);
	
	 //Wait for LOW response level
    while (1) {
			if (!(DATA_DHT22)) {
				 break;
			}
			if (count_time_out > 40) {
				 return ERROR_RESPONSE_LOW;  //break function due to error
			}
			count_time_out++;
			Delay(1);				 
		}
		GPIOD->BSRRL = GPIO_Pin_12; 
		
		//Wait for HIGH reponse level
		count_time_out = 0; 
		while (1) {
			 if ((DATA_DHT22) == 0x2) {
				  break;
			 }
			 if (count_time_out > 85) {
			     return ERROR_RESPONSE_UP; //break function due to error
			 }
			 count_time_out++;
			 Delay(1);
		}
		GPIOD->BSRRL = GPIO_Pin_13; 
		
		count_time_out = 0;		
		
		//Wait for Start Transmission Bit
		while(1) {
		   if (!(DATA_DHT22)) {
		    	break;
		   }
			 if (count_time_out > 75) {
				   return ERROR_TRANSMISSION_START; //break function due to error
			 }
			 count_time_out++;
			 Delay(1);
			 
		}
		GPIOD->BSRRL = GPIO_Pin_14;
		
		//Get 40-bit data
		for (unsigned char i = 0; i < 40; i++) { //40-bit data	
	      count_time_out = 0;			
		    while(1) {
					if (((DATA_DHT22) == 0x2) && (ack_bit == 0)) {
						  TIM_SetCounter(TIM2, 0);
		    		  ack_bit++;
		    	}
		    	if ((ack_bit > 0) && ((DATA_DHT22) == 0)) {
						 ack_bit = 0;
						 break;
						 //printf("Error\n");
						 //return;
					}
					if (!ack_bit) {
						  if (count_time_out > 75) {
						      return ERROR_TRANSMISSION_START; //break function due to error
							}
					}
					else {
						  if (count_time_out > 90) {
								  return ERROR_BIT_DATA; //break function due to error
							}
					}
					count_time_out++;
					Delay(1);
				}
				data[i/8] <<= 1;
			  //if (count_time > 20 ) {
				if (COUNT > 30) {
				  data[i/8] |= 1;
				}
		}
		
		//Check error sum
		if (!Check_Sum_DHT22()) {
		    return ERROR_CHECK_SUM; //break function due to error
		}
		GPIOD->BSRRL = GPIO_Pin_15;
		dat_humid = (data[0] << 8) | data[1];
		dat_temp  = (data[2] << 8) | data[3]; 
		printf("Do am la: %X\n", dat_humid); 
		printf("Nhiet do la: %X\n", dat_temp);
		printf("Byte sum is: %X\n", data[4]);
		
		Reset_Data_DHT22();//Reset DATA DHT22
		return DHT22_OK;
}
void Convert_Data_LCD(char convert_data[8], uint16_t raw_data) {
	  char tmp;
	  size_t length;
		sprintf(convert_data, "%d", raw_data);
	  length = strlen(convert_data);
		tmp = convert_data[length-1];
	  convert_data[length-1] = '.';
	  convert_data[length] = tmp;
	  convert_data[length+1] = '\0';
}

void Display_Info_Humid_Temp(void) {
	  char temp[8], humid[8];
	  Convert_Data_LCD(temp, dat_temp);
	  Convert_Data_LCD(humid, dat_humid);
	  Init_LCD();
	
	  //Display temperature
	  Set_Line1();
	  Display_Content("Nhiet do: ");
	  Display_Content(temp);
	  Write_Data(0x9);
	  Write_Data(0x43);
	
	  //Display Humid
	  Set_Line2();
	  Display_Content("Do am: ");
	  Display_Content(humid);
	  Write_Data(0x25);
}

//************************ Code reference  ****************************//
//*********************************************************************//
 //GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM2);//Config pin AF as alternative function TIM2, TIM2 channel 1
