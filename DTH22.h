#include <stm32f4xx.h>
#include <stm32f4xx_tim.h> //Timer Header File
#define DATA_DTH22 GPIO_Pin_0
 
TIM_OCInitTypeDef  TIM_OC;

void Init_Output_Compare() {
	TIM_OC.OCMode     =  TIM_OCMode_PWM1;
	TIM_OC.OuputState =  TIM_OutputState_Enable;
  TIM_OC.TIM_Pulse  =  0x3E80;
	TIM_OC.OCPolarity = TIM_OCPolarity_High
	TIM_OC1Init(TIM2, TIM_OC);
}
void Init_TIM() {
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //Enable Timer Clock = APB1 clock = 16 MHz 
	TIM_InternalClockConfig(TIM2);//Set clock source is Internal Clock CKNT
	TIM_CounterModeConfig(TIM2, TIM_CounterMode_Up); //Config Counter is UpCounting
	
}

enum ERROR_DTH22 {PULL_LOW = 0x1, PULL_UP, TRANSMISSION};
//Chan 2 noi vs chan PA0 cua STM32F411

//Gui tin hieu Start cua MCU den DTH22
//Su dung TIMER de tao xung
void Send_Start_Signal() {
	 uint32_t ExcessTime = 0;
	
	//Pull Low 1ms
	  while (time < 1ms) {
			PA0 = 0;//1ms
		}
		
		ExcessTime = 0;
		//Pull Up 40 ms
		while (time < 40us) {
			PA1 = 1;//40 ms
		}
		
		//sensor pull LOW
		ExcessTime = 0;
		while(1) {
			if (PA0 = 0x0) {
				break;
			}
			ExcessTime++;
		}
		
		if (ExcessTime > 10000) {
				return PULL_LOW;
		}
		
		//Sensor pull UP
		ExcessTime = 0;
		while (1) {
			if (PA0 = 0x1) {
				break;
			}
			ExcessTime++;
		}
		
		if (ExCessTime > 10000) {
				return PULL_UP;
		}
		
		//Start Transimission
		while(1) {
			if(PA0 == 0x0) {
				break;
			}
		}
		
		//Lay du lieu tu DTH22
		Get_Data_DTH22()
		
}

//Lay du lieu phan hoi tu DTH22
uint32_t Get_Data_DTH22() {
		uint32_t count_time, bit_count, data_DTH22 ;
	  unsigned char i;
	
	  count_time = bit_count = 0;
		for (i = 0; i < 40; i++) { //40-bit data
		    while(1) {
		    	if (DATA_DTH22 == 0x1) {
		    		break;
		    	}
		    }
		    
		    while(1) {
		    	if (DATA_DTH22 == 0x0) {
		    		count_time = 0x0;
		    	}
		    	count_time++;
				}
				if (count_time < 30) {
						data_DTH22 = data_DTH22 | (0 <<(40 - i));
				}
				else if ((count_time >= 30) && (count_time <  72)){
					  data_DTH22 = data_DTH22 | (1 << (40 - i));
				}
				else {
					  printf("Loi khi get bit\n");
				}
		}
		return data_DTH22;
}


//===== Extract Data and Check Sum Data =======//
//     Input: Data get from DTH22              //
//     Output: Nhiet do, do am                 //
//=============================================//
unsigned char Extract_Data(uint64_t data_DTH22) {
		uint16_t dat_humid, dat_temp;
	  uint8_t check_sum, chk_sum_cal;
	  unsigned char i, ret;
	
	  chk_sum_cal = 0;
		//dat_humid = (data_DTH22 & 0x0000000xFFFF000000) >> 24;
	  //dat_temp  = (data_DTH22 & 0x0000000000FFFF00) >> 8;
		dat_humid = (data_DTH22 & (uint64_t)0xFFFF000000) >> 24;
	  dat_temp  = (data_DTH22 & (uint64_t)0xFFFF00) >> 8;
	  check_sum = (data_DTH22 & (uint64_t)0xFF);
	 
	  for (i=0 ; i<4; i++) {
		   chk_sum_cal = chk_sum_cal | (uint8_t)(((data_DTH22) & ((uint64_t)0xFF00000000 >> (8*i))) >> (32-8*i)); 
		}
	  if (check_sum != chk_sum_cal) {
			 printf("Error when reading data\n");
			 ret = 0x0;
		}
		else {
			 ret = 0x1;
		}
		return ret;
	
}

//void Wait_Signal(unsigned char Exit_Loop) {
//		while(1) {
//				if (DATA_DTH22 == Exit_Loop) {
//						break;
//				}
//		}
//}

