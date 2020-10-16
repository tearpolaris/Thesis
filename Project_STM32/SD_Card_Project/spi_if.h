#ifndef __SPI_IF_H__
#define __SPI_IF_H__

#include <stm32f4xx.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include <stdint.h>
#include <stdbool.h>

#define COMMAND_LENGTH 48
#define READ_BL_LEN    512
#define WRITE_BL_LEN   512

#define Set_Pin(GPIO, Pin)        GPIO_SetBits(GPIO, Pin)
#define SPI_IS_BUSY(SPIx) (((SPIx)->SR & (SPI_SR_TXE | SPI_SR_RXNE)) == 0 || ((SPIx)->SR & SPI_SR_BSY))


#define LED_YELLOW    GPIO_Pin_12
#define LED_ORANGE    GPIO_Pin_13
#define LED_RED       GPIO_Pin_14
#define LED_BLUE      GPIO_Pin_15

#define HIGH 1
#define LOW  0

#ifndef PIN_PACK
    #define PIN_PACK 4
#endif

#ifndef SD_CS
    #define SD_CS         GPIO_Pin_9
#endif

#ifndef GPIO_CS
    #define GPIO_CS             GPIOA
#endif

#if PIN_PACK == 1
    #define SPI_SDCARD          SPI1
    #define GPIO_AF_SDCARD      GPIO_AF_SPI1
    #define GPIO_SPI            GPIOA
    #define SD_SCK              GPIO_Pin_5
    #define SD_MISO             GPIO_Pin_6
    #define SD_MOSI             GPIO_Pin_7
    #define SD_AF_SCK           GPIO_PinSource5
    #define SD_AF_MISO          GPIO_PinSource6
    #define SD_AF_MOSI          GPIO_PinSource7
#elif PIN_PACK == 2
    #define SPI_SDCARD          SPI1
    #define GPIO_AF_SDCARD      GPIO_AF_SPI1
    #define GPIO_SPI            GPIOB
    #define SD_SCK              GPIO_Pin_3
    #define SD_MISO             GPIO_Pin_4
    #define SD_MOSI             GPIO_Pin_5
    #define SD_AF_SCK           GPIO_PinSource3
    #define SD_AF_MISO          GPIO_PinSource4
    #define SD_AF_MOSI          GPIO_PinSource5
#elif PIN_PACK == 3
    #define SPI_SDCARD          SPI2
    #define GPIO_AF_SDCARD      GPIO_AF_SPI2
    #define GPIO_SPI            GPIOB
    #define SD_SCK              GPIO_Pin_13
    #define SD_MISO             GPIO_Pin_14
    #define SD_MOSI             GPIO_Pin_15
    #define SD_AF_SCK           GPIO_PinSource13
    #define SD_AF_MISO          GPIO_PinSource14
    #define SD_AF_MOSI          GPIO_PinSource15
#elif PIN_PACK == 4
    #define SPI_SDCARD          SPI2
    #define GPIO_AF_SDCARD      GPIO_AF_SPI2
    #define GPIO_SPI            GPIOB
    #define SD_SCK              GPIO_Pin_10
    #define SD_MISO             GPIO_Pin_14
    #define SD_MOSI             GPIO_Pin_15
    #define SD_AF_SCK           GPIO_PinSource10
    #define SD_AF_MISO          GPIO_PinSource14
    #define SD_AF_MOSI          GPIO_PinSource15
#elif PIN_PACK == 5
    #define SPI_SDCARD          SPI3
    #define GPIO_AF_SDCARD      GPIO_AF_SPI3
    #define GPIO_SPI            GPIOB
    #define SD_SCK              GPIO_Pin_3
    #define SD_MISO             GPIO_Pin_4
    #define SD_MOSI             GPIO_Pin_5
    #define SD_AF_SCK           GPIO_PinSource3
    #define SD_AF_MISO          GPIO_PinSource4
    #define SD_AF_MOSI          GPIO_PinSource5
#elif PIN_PACK == 6
    #define SPI_SDCARD          SPI3
    #define GPIO_AF_SDCARD      GPIO_AF_SPI3
    #define GPIO_SPI            GPIOC
    #define SD_SCK              GPIO_Pin_10
    #define SD_MISO             GPIO_Pin_11
    #define SD_MOSI             GPIO_Pin_12
    #define SD_AF_SCK           GPIO_PinSource10
    #define SD_AF_MISO          GPIO_PinSource11
    #define SD_AF_MOSI          GPIO_PinSource12
#elif PIN_PACK == 7
    #define SPI_SDCARD          SPI4
    #define GPIO_AF_SDCARD      GPIO_AF_SPI4
    #define GPIO_SPI            GPIOE
    #define SD_SCK              GPIO_Pin_12
    #define SD_MISO             GPIO_Pin_13
    #define SD_MOSI             GPIO_Pin_14
    #define SD_AF_SCK           GPIO_PinSource12
    #define SD_AF_MISO          GPIO_PinSource13
    #define SD_AF_MOSI          GPIO_PinSource14
#else
    #error Invalid config. Please check PIN_PACK define again

#endif


#define TIM_SDCARD          TIM2
#define NCR_TIME            100
#define AMCD41_TIMEOUT      1000
#define INITIALIZE_TIME     31250

/* Results of Disk Functions */
typedef enum {
    RES_OK = 0,     /* 0: Successful */
    RES_ERROR,      /* 1: R/W Error */
    RES_WRPRT,      /* 2: Write Protected */
    RES_NOTRDY,     /* 3: Not Ready */
    RES_PARERR      /* 4: Invalid Parameter */
} DRESULT;


typedef enum SD_command {
    RESET_IDLE      = 0  ,
    SEND_IF_COND    = 0x8,
    READ_CSD        = 0x9,
    READ_SINGLE_BLOCK = 17,
    READ_MULTI_BLOCK = 18,
    SD_SEND_OP_COND = 41 ,
    APP_CMD         = 55 ,
    READ_OCR        = 58
} SD_command_t;

typedef enum ERR_SD_Init{
    ERR_OK = 0,
    ERR_CMD_TIMEOUT = -1, //Timeout response command
    ERR_CARD_INVALID = -2, //Unknown Card
    ERR_PATTERN_NOT_MATCH = -3, //Pattern not match
    TIMEOUT_INITIALIZE = -4 //Timeout initialization procedure
} ERR_SD_Init_t;

typedef enum ERR_Cmd {
  ERR_CMD_OK      =  0,
  ERR_CMD_BUSY    = -1,
  ERR_CMD_TMOUT   = -2,
  ERR_CMD_PATTERN = -3
} ERR_Cmd_t;

typedef enum Type_Capacity_Card {
    UNKNOWN_CARD = -1,
    STANDARD_CAPACITY = 0,
    HIGH_CAPACITY = 1
} Type_Capacity_Card_t;

typedef struct SD_Card {
    ERR_SD_Init_t error;
    Type_Capacity_Card_t sd_type;       
} SD_Card_t;


//***********************************************//
//**************** PUBLIC FUNCTION **************//
//***********************************************//
ERR_Cmd_t Init_SD_Card(void);
//DRESULT disk_read (uint8_t *buff, uint32_t sector, unsigned int count);

#endif
