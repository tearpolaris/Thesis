/*-----------------------------------------------------------------------/
/  Low level disk interface modlue include file   (C)ChaN, 2019          /
/-----------------------------------------------------------------------*/

#ifndef _DISKIO_DEFINED
#define _DISKIO_DEFINED

#ifdef __cplusplus
extern "C" {
#endif

#include <stm32f4xx.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>

/* Status of Disk Functions */
typedef BYTE	DSTATUS;

/* Results of Disk Functions */
typedef enum {
	RES_OK = 0,		/* 0: Successful */
	RES_ERROR,		/* 1: R/W Error */
	RES_WRPRT,		/* 2: Write Protected */
	RES_NOTRDY,		/* 3: Not Ready */
	RES_PARERR		/* 4: Invalid Parameter */
} DRESULT;


/*---------------------------------------*/
/* Prototypes for disk control functions */


DSTATUS disk_initialize (BYTE pdrv);
DSTATUS disk_status (BYTE pdrv);
DRESULT disk_read (BYTE pdrv, BYTE* buff, LBA_t sector, UINT count);
DRESULT disk_write (BYTE pdrv, const BYTE* buff, LBA_t sector, UINT count);
DRESULT disk_ioctl (BYTE pdrv, BYTE cmd, void* buff);

/* Disk Status Bits (DSTATUS) */

#define STA_INITIALIZED 0x0   /* Drive is initialized sucessfully */
#define STA_NOINIT		  0x01	/* Drive not initialized */
#define STA_NODISK		  0x02	/* No medium in the drive */
#define STA_PROTECT		  0x04	/* Write protected */


/* Command code for disk_ioctrl fucntion */

/* Generic command (Used by FatFs) */
#define CTRL_SYNC			0	/* Complete pending write process (needed at FF_FS_READONLY == 0) */
#define GET_SECTOR_COUNT	1	/* Get media size (needed at FF_USE_MKFS == 1) */
#define GET_SECTOR_SIZE		2	/* Get sector size (needed at FF_MAX_SS != FF_MIN_SS) */
#define GET_BLOCK_SIZE		3	/* Get erase block size (needed at FF_USE_MKFS == 1) */
#define CTRL_TRIM			4	/* Inform device that the data on the block of sectors is no longer used (needed at FF_USE_TRIM == 1) */

/* Generic command (Not used by FatFs) */
#define CTRL_POWER			5	/* Get/Set power status */
#define CTRL_LOCK			6	/* Lock/Unlock media removal */
#define CTRL_EJECT			7	/* Eject media */
#define CTRL_FORMAT			8	/* Create physical format on the media */

/* MMC/SDC specific ioctl command */
#define MMC_GET_TYPE		10	/* Get card type */
#define MMC_GET_CSD			11	/* Get CSD */
#define MMC_GET_CID			12	/* Get CID */
#define MMC_GET_OCR			13	/* Get OCR */
#define MMC_GET_SDSTAT		14	/* Get SD status */
#define ISDIO_READ			55	/* Read data form SD iSDIO register */
#define ISDIO_WRITE			56	/* Write data to SD iSDIO register */
#define ISDIO_MRITE			57	/* Masked write data to SD iSDIO register */

/* ATA/CF specific ioctl command */
#define ATA_GET_REV			20	/* Get F/W revision */
#define ATA_GET_MODEL		21	/* Get model name */
#define ATA_GET_SN			22	/* Get serial number */


/* SD CARD USER IMPLEMENTATION */
#define COMMAND_LENGTH 48
#define READ_BL_LEN    512
#define WRITE_BL_LEN   512
#define TIM_SDCARD          TIM2
#define NCR_TIME            100
#define AMCD41_TIMEOUT      1000
#define INITIALIZE_TIME     31250

//#define  ERR_CMD_OK       0
//#define  ERR_CMD_BUSY    -1
//#define  ERR_CMD_TMOUT   -2
//#define  ERR_CMD_PATTERN -3

#define Set_Pin(GPIO, Pin)        GPIO_SetBits(GPIO, Pin)
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

typedef enum SD_command {
    RESET_IDLE        = 0x0,
    SEND_IF_COND      = 0x8,
    READ_CSD          = 0x9,
    STOP_TRANSMISSION = 12 ,
    READ_SINGLE_BLOCK = 17 ,
    READ_MULTI_BLOCK  = 18 ,
    SD_SEND_OP_COND   = 41 ,
    APP_CMD           = 55 ,
    READ_OCR          = 58
} SD_command_t;

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
    ERR_Cmd_t error;
    Type_Capacity_Card_t sd_type;       
} SD_Card_t;

#ifdef __cplusplus
}
#endif

#endif
