#include "spi_if.h"

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

int main() {
    //prvSetupHardware();
    //vTaskStartScheduler();
    uint8_t read_buff[READ_BL_LEN];
    ERR_Cmd_t err;

    Test_Interrupt();
    err = Init_SD_Card();

    if (err == ERR_OK) { 
        disk_read (read_buff, 0, 3);
        if (read_buff[0] == 0xEB) {
            GPIO_SetBits(GPIOD, GPIO_Pin_12);
        }
    }
    return 0;
}
