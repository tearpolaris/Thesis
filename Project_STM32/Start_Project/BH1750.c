#include "BH1750.h"

uint16_t Ketqua[2];

void Config_Clock_AHB_Multi_10(void) {
	RCC->PLLCFGR &= (~RCC_PLLCFGR_PLLN);//Clear factor multi N
	RCC->PLLCFGR |= 0x1900; //N = 100;
	RCC->PLLCFGR &= (~RCC_PLLCFGR_PLLM);//Clear factor divide M
	RCC->PLLCFGR |= 0xA; //M = 10;
	RCC->PLLCFGR &= (~RCC_PLLCFGR_PLLP);//Clear P
	RCC->PLLCFGR |= 0x10000;//P = 4
	RCC_PLLCmd(ENABLE);
	while((RCC->CR & RCC_CR_PLLRDY) == 0) {
  }
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);//Select PLL as system source clock
	while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS ) != RCC_CFGR_SWS_PLL) {
  }
	SystemCoreClockUpdate();
}
	

void Test_PullUp(void) {
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;// Pin_6 la SCl, Pin_9 la SDA
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	
	GPIO_Init(GPIOC, &GPIO_InitStruct);
}


void Init_BH1750_I2C(void) {
	// cho phep xung SDA, SCL o port B
	printf("GPIOB is %X\n", GPIOB->IDR);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);		
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);

	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_9;// Pin_6 la SCl, Pin_9 la SDA
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	
		// cau hinh cac chan PORT B
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6,GPIO_AF_I2C1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9 ,GPIO_AF_I2C1);
	// ket noi cac chan I2C1 de ngoai vi AF

	I2C_InitTypeDef I2C_InitStruct;
	I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStruct.I2C_ClockSpeed = 400000;
	//I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_16_9;
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStruct.I2C_OwnAddress1 = 0x00;
	
	I2C_Init(I2C1,&I2C_InitStruct);
	I2C_Cmd(I2C1,ENABLE);
}


void I2C_Start(I2C_TypeDef* I2Cx, uint8_t ADDRESS, uint8_t direction){
	// kiem tra co ban hay khong
	printf("GPIOB is %X\n", GPIOB->IDR);
	while(I2C_GetFlagStatus(I2Cx,I2C_FLAG_BUSY));	
	GPIO_SetBits(GPIOD, GPIO_Pin_13);
	// MASTER gui tin hieu START
	I2C_GenerateSTART(I2Cx,ENABLE);	
		printf("GPIOB is %X\n", GPIOB->IDR);
	// sau do MASTER doi su kien EV5 chung to tin hieu START da dc gui hoan toan len BUS	
	while(!I2C_CheckEvent(I2Cx,I2C_EVENT_MASTER_MODE_SELECT));
	GPIO_SetBits(GPIOD, GPIO_Pin_12);
	// truyen dia chi Slave( BH1750) va che do TRUYEN hay NHAN du lieu	
	I2C_Send7bitAddress(I2Cx,ADDRESS,direction);	
		// kiem tra 
	if(direction == I2C_Direction_Transmitter){		
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));		
	}
	else if(direction == I2C_Direction_Receiver){	
			// kiem tra EV6		
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	
	}
	GPIO_SetBits(GPIOD, GPIO_Pin_14);
}

//************** Ham gui 1 byte den SLAVE (BH1750).....................*/
void I2C_Write(I2C_TypeDef* I2Cx, uint8_t DATA){
	// gui DATA tu MASTER den SLAVE(BH1750)	
	I2C_SendData(I2Cx, DATA);		
	// kiem tra su kien EV8_2	
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}


/*======== ham nhan byte cao SLave=====================*/


uint8_t Nhan_Bytecao(I2C_TypeDef* I2Cx){
			uint8_t data =0;
			I2C_AcknowledgeConfig(I2Cx, ENABLE);
			// kiem tra EV7
			while(!I2C_CheckEvent(I2Cx,I2C_EVENT_MASTER_BYTE_RECEIVED)); 
			data = I2C_ReceiveData(I2Cx);
			return data;
}

/*======== ham doc byte thap SLAVE =======*/

uint8_t Nhan_Bytethap(I2C_TypeDef* I2Cx){
			uint8_t data=0;
			I2C_AcknowledgeConfig(I2Cx,DISABLE);
			//kiem tra EV7
			while(!I2C_CheckEvent(I2Cx,I2C_EVENT_MASTER_BYTE_RECEIVED));
			data = I2C_ReceiveData(I2Cx);
			return data;
}
/*======= ham dieu kien STOP=======*/
void I2C_Stop(I2C_TypeDef* I2Cx){
		I2C_GenerateSTOP(I2Cx, ENABLE);
	
}
/*=========ham delay=====*/
void Delay(uint32_t time) {
	 uint32_t var, i;
	 for (var = 0; var < time; var++) {
		 for (i=0; i < 1000; i++) {
		 }
	 }
 }

/*==========ham MASTER truyen=======*/
void BH1750_Init(){
	  I2C_Start(I2C1,(0x23)<<1,I2C_Direction_Transmitter);
		I2C_Write(I2C1,0x10);
		while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));
		I2C_Stop(I2C1);
}

/*===========ham MASTER NHAN=========*/
uint16_t BH1750_Read(){
		uint16_t val =0;
		I2C_Start(I2C1,(0x23)<<1,I2C_Direction_Receiver);
		Ketqua[0]= Nhan_Bytecao(I2C1);
		Ketqua[1]= Nhan_Bytethap(I2C1);
		I2C_Stop(I2C1);
		val=((Ketqua[0]<<8)|Ketqua[1])/1.2;
		return val;
}
	
