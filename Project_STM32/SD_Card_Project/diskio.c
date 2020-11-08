/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2019        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control modules to the FatFs module with a defined API.       */
/*-----------------------------------------------------------------------*/

#include "ff.h"         /* Obtains integer types */
#include "diskio.h"     /* Declarations of disk functions */

//*********** DISK STATUS **********//
int RAM_disk_status(void);
int MMC_disk_status(void);
int USB_disk_status(void);

//*********** DISK INITIALIZE ********//
int RAM_disk_initialize(void);
ERR_Cmd_t MMC_disk_initialize(void);
int USB_disk_initialize(void);

//*********** DISK WRITE *********//
int RAM_disk_write(const BYTE *buff, LBA_t sector, UINT count);
int MMC_disk_write(const BYTE *buff, LBA_t sector, UINT count);
int USB_disk_write(const BYTE *buff, LBA_t sector, UINT count);     

//*********** DISK READ **********//   
int RAM_disk_read (BYTE *buff, LBA_t sector, UINT count);
DRESULT MMC_disk_read(BYTE *buff, LBA_t sector, UINT count);
int USB_disk_read(BYTE *buff, LBA_t sector, UINT count);

/* Definitions of physical drive number for each drive */
#define DEV_RAM     0   /* Example: Map Ramdisk to physical drive 0 */
#define DEV_MMC     1   /* Example: Map MMC/SD card to physical drive 1 */
#define DEV_USB     2   /* Example: Map USB MSD to physical drive 2 */

SD_Card_t sd_info_str;
SD_Card_t *sd_info = &sd_info_str;

void Init_SPI_GPIO_Mode(void);
void Init_SPI_Register(void);
void Delay_us(TIM_TypeDef *TIMx, uint32_t micro);
void Set_Pin_CS(uint8_t cs_pin);
void CS_Select(void);
void CS_DeSelect(void);
void Get_Extended_R7(uint8_t *dat);

DRESULT Read_Single_Block(BYTE *buff, uint32_t data_addr);
DRESULT Read_Multiple_Block(BYTE *buff, LBA_t sector, UINT count);

uint16_t Send_Command(uint8_t command, uint32_t argument);
uint16_t Send_Byte(uint8_t dat);
uint16_t Send_ACMD_Command(uint8_t command, uint32_t argument);

uint32_t Wait_Idle_SDCard(void);
uint32_t Get_CSD_Register(void);

ERR_Cmd_t Get_CCS_Info_CMD58(SD_Card_t* sd);
ERR_Cmd_t Initialization_Complete_ACMD41(void);
ERR_Cmd_t If_operating_CMD8(uint32_t pattern);
ERR_Cmd_t Reset_CMD0(void);

#define WAIT_IDLE_CMD(time)    if (Wait_Idle_SDCard() > time) { \
                                   return ERR_CMD_BUSY;             \
                               }
/*-----------------------------------------------------------------------*/
/* SPI SD Card Interface                                                 */
/*-----------------------------------------------------------------------*/

//==============================================================//
//================ Initialize GPIO SPI Mode ====================//
//==============================================================//
void Init_SPI_GPIO_Mode(void) {
    assert_param(IS_SPI_ALL_PERIPH(SPI_SDCARD));
    assert_param(IS_GPIO_ALL_PERIPH(GPIO_SPI));

    GPIO_InitTypeDef Init_SPI;

    //Provide clock for SPI peripheral
    if (SPI_SDCARD == SPI1) {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
    }
    else if (SPI_SDCARD == SPI2) {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE); 
    }
    else if (SPI_SDCARD == SPI3) {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE); 
    }
    else if (SPI_SDCARD == SPI4) {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI4, ENABLE); 
    }

    //Provide clock for SCK, MISO, MOSI PIN
    if (GPIO_SPI == GPIOA) {
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    }
    else if (GPIO_SPI == GPIOB) {
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    }
    else if (GPIO_SPI == GPIOC) {
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    }
    else if (GPIO_SPI == GPIOE) {
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    }

    //Provide clock for CS PIN
    if (GPIO_CS == GPIOA) {
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    }
    else if (GPIO_CS == GPIOB) {
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    }
    else if (GPIO_CS == GPIOC) {
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    }
    else if (GPIO_CS == GPIOE) {
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    }


    Init_SPI.GPIO_Pin = SD_SCK | SD_MISO | SD_MOSI;
    Init_SPI.GPIO_Speed = GPIO_Speed_50MHz;
    Init_SPI.GPIO_OType = GPIO_OType_PP;
    Init_SPI.GPIO_PuPd = GPIO_PuPd_UP;
    Init_SPI.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init(GPIO_SPI, &Init_SPI);

    //Config pin MISO, MOSI, SCK
    GPIO_PinAFConfig(GPIO_SPI, SD_AF_SCK, GPIO_AF_SDCARD);
    GPIO_PinAFConfig(GPIO_SPI, SD_AF_MISO, GPIO_AF_SDCARD);
    GPIO_PinAFConfig(GPIO_SPI, SD_AF_MOSI, GPIO_AF_SDCARD);

    //Config pin CS 
    Init_SPI.GPIO_Pin = SD_CS;
    Init_SPI.GPIO_Speed = GPIO_Speed_50MHz;   
    Init_SPI.GPIO_OType = GPIO_OType_PP;
    Init_SPI.GPIO_PuPd = GPIO_PuPd_UP;
    Init_SPI.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_Init(GPIO_CS, &Init_SPI);
}

void Init_SPI_Register(void) {
    SPI_InitTypeDef SPI_Str_Init;

    SPI_Str_Init.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
    SPI_Str_Init.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_Str_Init.SPI_CPOL = SPI_CPOL_Low;
    SPI_Str_Init.SPI_DataSize = SPI_DataSize_8b;
    SPI_Str_Init.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_Str_Init.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_Str_Init.SPI_Mode = SPI_Mode_Master;
    SPI_Str_Init.SPI_NSS = SPI_NSS_Soft;

    SPI_Init(SPI_SDCARD, &SPI_Str_Init);
    SPI_Cmd(SPI_SDCARD, ENABLE);
}

void Init_Counter_SDCard(TIM_TypeDef *TIMx) {
    uint32_t multipler, TIM_SDCard_Frequency, TIM_Counter_Frequency, APB_Peripheral;
        TIM_TimeBaseInitTypeDef TIM_Base_Init_SDCard;

    if (TIMx == TIM2) {
        APB_Peripheral = RCC_APB1Periph_TIM2;    
    }
    else if (TIMx == TIM3) {
        APB_Peripheral = RCC_APB1Periph_TIM3;
    }
    else if (TIMx == TIM4) {
        APB_Peripheral = RCC_APB1Periph_TIM4;
    }
    else if (TIMx == TIM5) {
        APB_Peripheral = RCC_APB1Periph_TIM5;
    }

    RCC_APB1PeriphClockCmd(APB_Peripheral, ENABLE);
    TIM_InternalClockConfig(TIMx);
    RCC_ClocksTypeDef RCC_Clocks;
    RCC_GetClocksFreq(&RCC_Clocks);
    if (RCC_Clocks.PCLK1_Frequency == RCC_Clocks.SYSCLK_Frequency) {
        multipler = 1;
    }
    else {
        multipler = 2;
    }
    TIM_Counter_Frequency = multipler * RCC_Clocks.PCLK1_Frequency;
    TIM_SDCard_Frequency = 1000000;//1MHz = 1us

    TIM_Base_Init_SDCard.TIM_Period = 999;
    TIM_Base_Init_SDCard.TIM_Prescaler = TIM_Counter_Frequency/TIM_SDCard_Frequency - 1;
    TIM_Base_Init_SDCard.TIM_ClockDivision = 0;
    TIM_Base_Init_SDCard.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInit(TIMx, &TIM_Base_Init_SDCard);
}

void Delay_us(TIM_TypeDef *TIMx, uint32_t micro) {
    volatile uint32_t count_first = TIMx->CNT;
    while (micro--) {
        while (TIMx->CNT - count_first) {
        } 
        count_first = TIMx->CNT;
    }
}

void CS_Select(void) {
    Send_Byte(0xFF);
    GPIO_ResetBits(GPIO_CS, SD_CS);
    Send_Byte(0xFF);
    //Send_Byte(0xFF);
}

void CS_DeSelect(void) {
    Send_Byte(0xFF);
    GPIO_SetBits(GPIO_CS, SD_CS);
    Send_Byte(0xFF);
    //Send_Byte(0xFF);
}

void Set_Pin_CS(uint8_t cs_pin) {
    assert_param((cs_pin == HIGH) || (cs_pin == LOW));
    if (cs_pin == HIGH) {
        GPIO_SetBits(GPIO_SPI, SD_CS);
    }
    else {
        GPIO_ResetBits(GPIO_SPI, SD_CS);
    }
}

uint16_t Send_Command(uint8_t command, uint32_t argument) {
    assert_param(command <= 63);// valid range command from 0 to 63

    uint8_t dat, crc_chk, count_arg, repeat; 
    uint16_t response;
  
    count_arg = 3;
    repeat = 0xFF;

    crc_chk = 0xFF;
    if (command == RESET_IDLE) {
        crc_chk = 0x95;
    }
    else if (command == SEND_IF_COND) {
        crc_chk = 0x87;
    }

    for (int i = 0; i < COMMAND_LENGTH/8; i++) {
        if (i == 0) {
           dat = 0x40 | command; 
        }
        else if (i == COMMAND_LENGTH/8 -1) {
           dat = crc_chk; 
        }
        else {
           dat = argument >> (count_arg * 8);
           count_arg--;
        } 
        Send_Byte(dat);
    } 
 
    do {
        response = Send_Byte(0xFF);
    } while ((response & 0x80) && repeat--);
    return response;
}

void Get_Extended_R7(uint8_t *dat) {

    //Ignore first two bytes
    for (int i = 0; i < 2; i++) {
        Send_Byte(0xFF); 
    }

    dat[0] = Send_Byte(0xFF);
    dat[1] = Send_Byte(0xFF);
}

uint32_t Wait_Idle_SDCard(void) {
    uint32_t count = 0; 
    do {
        if (Send_Byte(0xFF) == 0xFF) {
            break;
        }
        count++;
    } while (count < NCR_TIME);
    return count;
}

uint16_t Send_Byte(uint8_t dat) {
    while (SPI_I2S_GetFlagStatus (SPI_SDCARD, SPI_I2S_FLAG_TXE) != SET) {
    }
    while (SPI_I2S_GetFlagStatus (SPI_SDCARD, SPI_I2S_FLAG_BSY) != RESET) {
    }

    SPI_I2S_SendData(SPI_SDCARD, (uint16_t)dat);

    while (SPI_I2S_GetFlagStatus (SPI_SDCARD, SPI_I2S_FLAG_RXNE) != SET) {
    }
    while (SPI_I2S_GetFlagStatus (SPI_SDCARD, SPI_I2S_FLAG_BSY) != RESET) {
    }
    return SPI_I2S_ReceiveData(SPI_SDCARD); 
}

uint16_t Send_ACMD_Command(uint8_t command, uint32_t argument) {
    uint16_t res = Send_Command(APP_CMD, 0x0);
    return Send_Command(command, argument);
}

//****************************************//
//********** RESET COMMAND - CMD0 ********//
//****************************************//
ERR_Cmd_t Reset_CMD0(void) {
    uint32_t count = 0;

    CS_Select(); //assert pin CS
    WAIT_IDLE_CMD(NCR_TIME)
    do {
        if (Send_Command(RESET_IDLE, 0x0) == 0x1) { //expected response 0x1
            break;
        }
        Delay_us(TIM_SDCARD, 1);
        count++;
    } while (count < NCR_TIME);

    CS_DeSelect(); //deassert pin CS
    if (count >= NCR_TIME) {
        return ERR_CMD_TMOUT;        
    }
    return ERR_CMD_OK;
}

//******************************************************//
//********** INTERFACE OPERATING COMMAND - CMD8 ********//
//******************************************************//
ERR_Cmd_t If_operating_CMD8(uint32_t pattern) {
    uint8_t res_r7[2], expected_vhs;
    uint32_t count;

    count = 0x0;
    expected_vhs = (uint8_t)pattern & 0xFF;

    CS_Select(); //assert pin CS
    WAIT_IDLE_CMD(NCR_TIME)

    do {
        if (Send_Command(SEND_IF_COND, pattern)  == 0x1) { 
            break;
        }
        Delay_us(TIM_SDCARD, 1);
        count++;
    } while (count < NCR_TIME);

    if (count >= NCR_TIME) {
        CS_DeSelect(); //deassert pin CS
        return ERR_CMD_TMOUT;        
    }
 
    Get_Extended_R7(res_r7);
    CS_DeSelect(); //deassert pin CS
    if (res_r7[1] != expected_vhs) {
        return ERR_CMD_PATTERN;
    }
    return ERR_CMD_OK;
}

//**********************************************************//
//********** CHECK INITIALIZATION COMPLETE - ACMD41 ********//
//**********************************************************//
ERR_Cmd_t Initialization_Complete_ACMD41(void) {
    uint32_t count;
    uint16_t res;

    count = 0x0;
    do {
        CS_Select();
        WAIT_IDLE_CMD(NCR_TIME)
        res = Send_Command(APP_CMD, 0x0);
        CS_DeSelect();
        if (res < 0x2) {
            CS_Select();
            WAIT_IDLE_CMD(NCR_TIME)
            res = Send_Command(SD_SEND_OP_COND, 0x40000000);
            CS_DeSelect();         
        }
        if (res == 0x0) {
            break;
        }
        Delay_us(TIM_SDCARD, 100); 
        count++;
    } while (count < AMCD41_TIMEOUT);

    if (count >= AMCD41_TIMEOUT) {
        return ERR_CMD_TMOUT;        
    }
    return ERR_CMD_OK;
}

//***********************************************//
//********** GET CCS INFORMATION - CMD58 ********//
//***********************************************//
ERR_Cmd_t Get_CCS_Info_CMD58(SD_Card_t* sd) {
    uint32_t count, response;
    count = 0x0;

    CS_Select();
    WAIT_IDLE_CMD(NCR_TIME)

    do {
        if (Send_Command(READ_OCR, 0x0)  == 0x0) {
            break;
        }
        Delay_us(TIM_SDCARD, 1);
        count++;
    } while (count < NCR_TIME);
    
    if (count >= NCR_TIME) {
        CS_DeSelect();
        return ERR_CMD_TMOUT;
    }

    response = Send_Byte(0xFF);
    for (int i = 0; i < 3; i++) {
        Send_Byte(0xFF);
    }

    if (response & 0x80) {
        if (response & 0x40) {
            sd->sd_type = HIGH_CAPACITY;
        }
        else {
            sd->sd_type = STANDARD_CAPACITY;
        }
    }
    CS_DeSelect();  
    return ERR_CMD_OK;  
}


/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
    BYTE pdrv       /* Physical drive nmuber to identify the drive */
)
{
    DSTATUS stat;
    int result;

    switch (pdrv) {
    case DEV_RAM :
        result = RAM_disk_status();

        // translate the reslut code here

        return stat;

    case DEV_MMC :
        result = MMC_disk_status();

        // translate the reslut code here

        return stat;

    case DEV_USB :
        result = USB_disk_status();

        // translate the reslut code here

        return stat;
    }
    return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
    BYTE pdrv               /* Physical drive nmuber to identify the drive */
)
{
    DSTATUS stat;
    int result;

    switch (pdrv) {
    case DEV_RAM :
        result = RAM_disk_initialize();

        // translate the reslut code here

        return stat;

    case DEV_MMC :
        result = MMC_disk_initialize();
        if (result == ERR_CMD_OK) {
            stat = STA_INITIALIZED;
        } else {
            stat = STA_NOINIT;
        }           
        // translate the reslut code here

        return stat;

    case DEV_USB :
        result = USB_disk_initialize();

        // translate the reslut code here

        return stat;
    }
    return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
    BYTE pdrv,      /* Physical drive nmuber to identify the drive */
    BYTE *buff,     /* Data buffer to store read data */
    LBA_t sector,   /* Start sector in LBA */
    UINT count      /* Number of sectors to read */
)
{
    DRESULT res;
    int result;

    switch (pdrv) {
    case DEV_RAM :
        // translate the arguments here

        result = RAM_disk_read(buff, sector, count);

        // translate the reslut code here

        return res;

    case DEV_MMC :
        // translate the arguments here

        res = MMC_disk_read(buff, sector, count);

        // translate the reslut code here

        return res;

    case DEV_USB :
        // translate the arguments here

        result = USB_disk_read(buff, sector, count);

        // translate the reslut code here

        return res;
    }

    return RES_PARERR;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if FF_FS_READONLY == 0

DRESULT disk_write (
    BYTE pdrv,          /* Physical drive nmuber to identify the drive */
    const BYTE *buff,   /* Data to be written */
    LBA_t sector,       /* Start sector in LBA */
    UINT count          /* Number of sectors to write */
)
{
    DRESULT res;
    int result;

    switch (pdrv) {
    case DEV_RAM :
        // translate the arguments here

        result = RAM_disk_write(buff, sector, count);

        // translate the reslut code here

        return res;

    case DEV_MMC :
        // translate the arguments here

        result = MMC_disk_write(buff, sector, count);

        // translate the reslut code here

        return res;

    case DEV_USB :
        // translate the arguments here

        result = USB_disk_write(buff, sector, count);

        // translate the reslut code here

        return res;
    }

    return RES_PARERR;
}

#endif


/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

DRESULT disk_ioctl (
    BYTE pdrv,      /* Physical drive nmuber (0..) */
    BYTE cmd,       /* Control code */
    void *buff      /* Buffer to send/receive control data */
)
{
    DRESULT res;
    int result;

    switch (pdrv) {
    case DEV_RAM :

        // Process of the command for the RAM drive

        return res;

    case DEV_MMC :

        // Process of the command for the MMC/SD card

        return res;

    case DEV_USB :

        // Process of the command the USB drive

        return res;
    }

    return RES_PARERR;
}

/*-----------------------------------------------------------------------*/
/* User Implementation Functions                                               */
/*-----------------------------------------------------------------------*/
int RAM_disk_status(void) {
    return 0;
}

int MMC_disk_status(void) {
    int ret;
    switch (sd_info->error) {
         /* OK */
        case ERR_CMD_OK:
            ret = RES_OK;
        break;

        /* not ready */
        case ERR_CMD_BUSY:
        case ERR_CMD_TMOUT:   
            ret = RES_NOTRDY;
        break;

        /* error R/W */
        case ERR_CMD_PATTERN:
        default:
            ret = RES_ERROR;
        break;
    } /* end switch */
    return ret;
}

int USB_disk_status(void) {
    return 0;
}

int RAM_disk_initialize(void) {
    return 0;
}

ERR_Cmd_t MMC_disk_initialize(void) {
    ERR_Cmd_t err_cmd;  

    Init_Counter_SDCard(TIM_SDCARD);

    //Initialize peripheral
    Init_SPI_GPIO_Mode();
    Init_SPI_Register();

    CS_DeSelect();
    Delay_us(TIM_SDCARD, 50000);//Wait for at least 1ms
    for (int i = 0; i < 10; i++) {
        Send_Byte(0xFF);
    }

    //---> CAN USE STATE MACHINE HERE
    if ((err_cmd = Reset_CMD0()) != ERR_CMD_OK) {
        sd_info->error = err_cmd;
        return err_cmd;
    }

    if ((err_cmd = If_operating_CMD8(0x1AA)) != ERR_CMD_OK) {
        sd_info->error = err_cmd;
        return err_cmd;
    }

    if ((err_cmd = Initialization_Complete_ACMD41()) != ERR_CMD_OK) {
        sd_info->error = err_cmd;        
        return err_cmd;
    }
    
    if ((err_cmd = Get_CCS_Info_CMD58(sd_info)) != ERR_CMD_OK) {
        sd_info->error = err_cmd;        
        return err_cmd;
    }

    sd_info->error = ERR_CMD_OK;
    return ERR_CMD_OK;
}

int USB_disk_initialize(void) {
    return 0;
}

int RAM_disk_write(const BYTE *buff, LBA_t sector, UINT count) {
    return 0; 
}

int MMC_disk_write(const BYTE *buff, LBA_t sector, UINT count) {
    return 0; 
}

int USB_disk_write(const BYTE *buff, LBA_t sector, UINT count) {
    return 0; 
}             
              
int RAM_disk_read (BYTE *buff, LBA_t sector, UINT count) {
    return 0;
}

DRESULT MMC_disk_read(BYTE *buff, LBA_t sector, UINT count) {
    uint32_t data_addr;    
    DRESULT res;

    if (!buff) {
        return RES_PARERR;
    }
  
    data_addr = sector;
    if (sd_info->sd_type == STANDARD_CAPACITY) {
        data_addr = sector * READ_BL_LEN;
    }  
    CS_Select();
    if (Wait_Idle_SDCard() > NCR_TIME) {
        return RES_NOTRDY;
    }

    if (count == 1) {
        res = Read_Single_Block(buff, data_addr);
    } 
    else {
        res = Read_Multiple_Block(buff, count, data_addr);
    }
    CS_DeSelect();
    return res;
}

int USB_disk_read(BYTE *buff, LBA_t sector, UINT count) {
    return 0;
}

/* Read Single Block */
/* RESPONSE FORMAT 
|R1 respose | Start block data token | Byte datas (512 bytes normally)| 16-bit CRC|
*/
DRESULT Read_Single_Block(BYTE *buff, uint32_t data_addr) {
    uint32_t tmout = 0x0;
    uint16_t response;

    if ((response = Send_Command(READ_SINGLE_BLOCK, data_addr)) == 0x0) {
        while (tmout < 3125) {
            if ((response = Send_Byte(0xFF)) == 0xFE) {
                break;
            }
            Delay_us(TIM_SDCARD, 1);
            tmout++;
        } //end while tmout

        if (tmout >= 3125) {
            if (response == 0xFF) {
                return RES_NOTRDY; //not response a valid R1
            }
            else {
                return RES_ERROR; //error when reading
            }
        } //end if tmout

        /* Start to read data */
        for (int i = 0; i < READ_BL_LEN; i++) {
            *(buff + i) = (uint8_t)Send_Byte(0xFF);
        }
        /* Read 2 bytes CRC */
        Send_Byte(0xFF); 
        Send_Byte(0xFF);
        return RES_OK; //return after reading complete
    } 
    return RES_ERROR; //error receving response is not valid
}

/* Read Multiple block */
DRESULT Read_Multiple_Block(BYTE *buff, UINT count, DWORD start_addr) {
    DWORD tmout, idx;
    WORD response;

    idx = 0x0; //initialize timeout and index
    if ((response = Send_Command(READ_MULTI_BLOCK, start_addr)) == 0x0) {
        while (count--) {
            tmout = 0;
            while (tmout < 3125) {
                if ((response = Send_Byte(0xFF)) == 0xFE) {
                    break;
                }
                Delay_us(TIM_SDCARD, 1);
                tmout++;
            } //end while tmout
            
            if (tmout >= 3125) {
                if (response == 0xFF) {
                    return RES_NOTRDY; //not response a valid R1
                }
                else {
                    return RES_ERROR; //error when reading
                }
            } //end if tmout
            
            /* Start to read data */
            for (int i = 0; i < READ_BL_LEN; i++) {
                *(buff + idx) = (uint8_t)Send_Byte(0xFF);
                idx++;
            }
            /* Read 2 bytes CRC */
            Send_Byte(0xFF);
            Send_Byte(0xFF);
        }
        if ((response = Send_Command(STOP_TRANSMISSION, 0x0)) == 0x0) {
           while (!Send_Byte(0xFF)) { //response R1b - read until non-zero byte is received 
           }
           return RES_OK; //return after reading complete
        }
    } 
    return RES_ERROR; //error receving response is not valid
}
