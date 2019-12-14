#include "DHT22.h"
#include "LCD.h"

uint8_t data[5] = { 0, 0, 0, 0, 0 };


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


void Init_DHT22_GPIO(GPIO_InitTypeDef* GPIO_Init_Str) {
    //Enable Clock for PORT A
    //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    //Enable System  Config Controller Clock
    GPIO_Init_Str->GPIO_Pin = DHT22_PIN;
    GPIO_Init_Str->GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init_Str->GPIO_OType = GPIO_OType_PP;
    GPIO_Init_Str->GPIO_PuPd  = GPIO_PuPd_DOWN;
    //GPIO_Init22.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_Init_Str->GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_Init(GPIOA, GPIO_Init_Str);
}


void Init_TIM2(TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure) {
    uint32_t multipler, TIM2_Clk_Frequency, TIM2_Counter_Frequency;
     //Enable clock peripheral TIM2
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    /* Enable clock for TIMx */ 
    TIM_InternalClockConfig(TIM2);//Set clock source is Internal Clock CKNT
     /* Time base configuration */
    RCC_ClocksTypeDef RCC_Clocks;
    RCC_GetClocksFreq(&RCC_Clocks);
    if (RCC_Clocks.PCLK1_Frequency == RCC_Clocks.SYSCLK_Frequency) {
        multipler = 1;
    }  
    else {
        multipler = 2;
    }
    TIM2_Clk_Frequency = multipler * RCC_Clocks.PCLK1_Frequency;
    TIM2_Counter_Frequency = 1000000;//Frequency is 1 kHz
      
    //TIM_TimeBaseStructure.TIM_Prescaler = (TIM2_Clk_Frequency/TIM2_Counter_Frequency) - 1; //15999
    //TIM_TimeBaseStructure.TIM_Period = 0;

    TIM_TimeBaseStructure.TIM_Period = 999;
    TIM_TimeBaseStructure.TIM_Prescaler = 0xF;//15 => Counter_Clock = 16MHz/(15 +1) = 1MHz
    //TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;    
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    //TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0; 
    
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
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
}

//********* Handler Interrupt External Trigger 1 *************//
//                                                            //
//************************************************************//
////Lay du lieu phan hoi tu DHT22
void Get_Data_DHT22(void) {
    uint32_t count_time_out;
    unsigned char i, ack_bit;
    
    count_time_out = ack_bit = 0; 
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
					}
					if (!ack_bit) {
						  if (count_time_out > 75) {
                  return;
						      //return ERROR_TRANSMISSION_START; //break function due to error
							}
					}
					else {
						  if (count_time_out > 90) {
                  return;
								  //return ERROR_BIT_DATA; //break function due to error
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
}

bool Check_Sum_DHT22(void) {
    if (((data[0] + data[1] + data[2] + data[3]) & 0xFF) == data[4]) {
        return true;
    }
    else {
        return false;
    }
    //return (data[4] == (data[0] + data[1] + data[2] + data[3]))? true: false;
}

void Reset_Data_DHT22(void) {
    for (int i=0; i < 5; i++) {
        data[i] = 0;
    }
}

void Initialization_General_DHT22(void) {
    TIM_TimeBaseInitTypeDef  TIM_BaseStruct;

    Init_TIM2(TIM_BaseStruct);
    Test_Interrupt();
    TIM_Cmd(TIM2, ENABLE);
    Delay(2000000);
    if (GPIO_DHT22 == GPIOA) {
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    }
    else if (GPIO_DHT22 == GPIOB) {
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    }
    else if (GPIO_DHT22 == GPIOC) {
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    }
    else if (GPIO_DHT22 == GPIOD) {
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    }
}

TYPE_DAT_DHT22 Init_Read_DHT22() {
    //TIM_TimeBaseInitTypeDef  TIM_BaseStruct;
    GPIO_InitTypeDef GPIO_Init_DHT22;
    uint32_t ack_bit=0;
    uint8_t dat_save[40];
    uint32_t count_time_out = 0;
    Reset_Data_DHT22();
    //Initialization_General_DHT22();
    Delay(2000000);

    Init_DHT22_GPIO(&GPIO_Init_DHT22);
    
    //Keo xuong 1ms
    GPIO_ResetBits(GPIO_DHT22, DHT22_PIN);
    Delay(1000);
    
    //Keo len 30us 
    GPIO_SetBits(GPIO_DHT22, DHT22_PIN);
    Delay(30);
     //Giai phong bus cho DHT22 dieu khien
    GPIO_Init_DHT22.GPIO_Mode  = GPIO_Mode_IN;
    GPIO_Init(GPIO_DHT22, &GPIO_Init_DHT22);
    //Delay(40);
     //Wait for LOW response level
    while (1) {
        if (!DATA_DHT22) {
            break;
        }
        if (count_time_out > 40) {
            GPIOD->BSRRL = GPIO_Pin_12;
            return ERROR_RESPONSE_LOW;  //break function due to error
        }
        count_time_out++;
        Delay(1);                
    }
        //Wait for HIGH reponse level
    count_time_out = 0; 
    while (1) {
        if (DATA_DHT22) {
            break;
        }
        if (count_time_out > 85) {
            GPIO_SetBits(GPIOD, GPIO_Pin_13);
            return ERROR_RESPONSE_UP; //break function due to error
        }
        count_time_out++;
        Delay(1);
    } 
    count_time_out = 0;   

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
  
    for (unsigned char i = 0; i < 40; i++) { //40-bit data	
	      count_time_out = 0;			
		    while(1) {
		    	if ((DATA_DHT22) && (!ack_bit)) {
		    		  TIM_SetCounter(TIM2, 0);
		    		  ack_bit++;
		    	}
		    	if ((ack_bit > 0) && (!DATA_DHT22)) {
		    		 ack_bit = 0;
		    		 break;
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
		    if (COUNT > 40) {
		      data[i/8] |= 1;
		    }
		}

    //Check error sum
    if (!Check_Sum_DHT22()) {
        GPIO_SetBits(GPIOD, GPIO_Pin_12);
        return ERROR_CHECK_SUM; //break function due to error
    }
    return DHT22_OK;
}

void DHT22_Get_Humid_Temp(uint16_t* dat_humid, uint16_t* dat_temp) {
    TYPE_DAT_DHT22 ret_read;
    ret_read = Init_Read_DHT22();
    if (ret_read == DHT22_OK) {
        GPIO_SetBits(GPIOD, GPIO_Pin_14);
        *dat_humid = ((uint16_t)(data[0] << 8) | data[1]);
        *dat_temp  = (uint16_t)(data[2] << 8) | data[3];
        //GPIOD->ODR = data[0];
    }
    else {
        *dat_humid = 0;
        *dat_temp = 0;
    } 
}

//************************ Code reference  ****************************//
//*********************************************************************//
 //GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM2);//Config pin AF as alternative function TIM2, TIM2 channel 1
