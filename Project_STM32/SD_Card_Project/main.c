#include "ff.h"
#include "diskio.h"
#include <string.h>
#include <stdio.h>
#include <stm32f4xx_usart.h>

#define DEV_MMC 1
// Disk number    |      Device
// ---------------| --------------
//           0    |  RAM
//           1    |  MMC/SD Card
//           2    |  USB

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
//******************************************************************//
//****** Configuration baud rate and enable clockfor USART1 *******//
//*****************************************************************//
void Debug_init_UART_Config(USART_TypeDef* USARTx, uint32_t baud_rate) {
    if (USARTx == USART1) {
          RCC_APB2PeriphClockCmd (RCC_APB2Periph_USART1, ENABLE);
    }
    else if (USARTx == USART2) {
        RCC_APB1PeriphClockCmd (RCC_APB1Periph_USART2, ENABLE);
    }
    else if (USARTx == USART6) {
        RCC_APB2PeriphClockCmd (RCC_APB2Periph_USART6, ENABLE);
    }
      USART_InitTypeDef Init_UART_Str;
      
      USART_StructInit(&Init_UART_Str);
      Init_UART_Str.USART_BaudRate = baud_rate;
      USART_Init(USARTx, &Init_UART_Str);
      USARTx->CR1 |= 0x20;//Enable Interrupt UART
      USART_Cmd(USARTx, ENABLE);
}

void Init_Debug_GPIO(void) {
    //Enable Clock for PORT B
    GPIO_InitTypeDef GPIO_UART_Debug;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    //Enable System  Config Controller Clock
    GPIO_UART_Debug.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_UART_Debug.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_UART_Debug.GPIO_OType = GPIO_OType_PP;
    GPIO_UART_Debug.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_UART_Debug.GPIO_Mode  = GPIO_Mode_AF; 
    GPIO_Init(GPIOC, &GPIO_UART_Debug);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);
}

void Transmit_string_UART(USART_TypeDef* USARTx, char* dat) {
    uint16_t count = strlen(dat);
    while (count) {
        while (!USART_GetFlagStatus(USARTx, USART_FLAG_TXE)) {
        }
        USART_SendData (USARTx, (uint16_t)*dat);
        dat++;
        count--;
    }
    while (!USART_GetFlagStatus(USARTx, USART_FLAG_TC)) {
    }
}

void Transmit_UART(USART_TypeDef* USARTx, uint8_t* dat, uint16_t count) {
    while (count) {
      while (!USART_GetFlagStatus(USARTx, USART_FLAG_TXE)) {
      }
      USART_SendData (USARTx, (uint16_t)*dat);
      dat++;
    count--;
  }
    while (!USART_GetFlagStatus(USARTx, USART_FLAG_TC)) {
    }
}

int main() {
    BYTE read_buff[READ_BL_LEN];
    char send_UART[10];
    UINT num_byte;
    DSTATUS err;
    DRESULT res;
    FRESULT fr_sd;
    FATFS fs_sdcard;
    FIL fp;


    Test_Interrupt();
    Init_Debug_GPIO();
    Debug_init_UART_Config(USART6, 9600);
    //err = disk_initialize(DEV_MMC);
    //if (err != RES_OK) {
    //    Set_Pin(GPIOD, LED_ORANGE);  
    //    return -1;
    //}
    //
    //res = disk_read (DEV_MMC, read_buff, 46975, 2);
//    if (res != RES_OK) {
//        Set_Pin(GPIOD, LED_ORANGE);  
//        return -1;
//    }
    //for (int i = 0; i < 20; i++) {
    //    Transmit_UART(USART6, read_buff + i, 1);
    //}
    //Transmit_string_UART(USART6, "\r\n");
    //for (int i = 512; i < 520; i++) {
    //    sprintf(send_UART, "%#X ", read_buff[i]);
    //    Transmit_string_UART(USART6, send_UART);
    //}
    //Transmit_UART(USART6, read_buff, 10);
    //Transmit_string_UART(USART6, "Beautiful weather");
//        Transmit_UART(USART6, (BYTE*)"File result OK\n", 10);
    fr_sd = f_mount(&fs_sdcard, "1:/", 1);
    if (fr_sd != FR_OK) {
        //printf("File result OK\n");
        Set_Pin(GPIOD, LED_ORANGE);  
        return -1;   
    }
    fr_sd = f_open(&fp, "1:/test.txt", FA_READ);  
    if (fr_sd != FR_OK) {
        Set_Pin(GPIOD, LED_ORANGE);
        return -1;
    } 
    Set_Pin(GPIOD, LED_YELLOW);
    fr_sd = f_read(&fp, read_buff, 10, &num_byte); 
    if (fr_sd != FR_OK) {
        Set_Pin(GPIOD, LED_ORANGE);
        return -1;
    } 
    
    //if (memcmp(read_buff, "Ma khong biet em", 10)) {
    //    Set_Pin(GPIOD, LED_ORANGE);
    //    return -1;
    //}
    Transmit_UART(USART6, read_buff, 10);
//    f_close(&fp);
    //Set_Pin(GPIOD, LED_BLUE);
    //if (err == STA_INITIALIZED) { 
    //    disk_read (DEV_MMC, read_buff, 0, 3);
    //    if (read_buff[0] == 0xEB) {
    //        GPIO_SetBits(GPIOD, GPIO_Pin_12);
    //    }
    //}
    return 0;
}
