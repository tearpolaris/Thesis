//OVER8 = 0 => Oversampling by 16, giam sai so do lech clock
//ONEBIT = 0 => Lay mau 3 lan tai trung tam cua bit
#include <stm32f4xx_usart.h>
#include <stdlib.h>

#define BUFFER_INITIALIZED 0x1
#define BUFFER_MALLOC      0x2

#define UART_TRANS_ENABLE 0x08
#define ESP8266_USART USART1
#define USART_ESP8266_SIZE 1024

#define ESP8266_COMMAND_IDLE           0
#define ESP8266_COMMAND_RST            1
#define ESP8266_COMMAND_CWJAP_SET      3
#define ESP8266_COMMAND_CWJAP_GET      4
#define ESP8266_COMMAND_CWLAP          5
#define ESP8266_COMMAND_CWSAP          6
#define ESP8266_COMMAND_CWLIF          7
#define ESP8266_COMMAND_CIPSTAMAC      8
#define ESP8266_COMMAND_CIPAPMAC       9
#define ESP8266_COMMAND_CIPSTA         10
#define ESP8266_COMMAND_CIPAP          11
#define ESP8266_COMMAND_CWMODE         12
//#define 
//#define 
//#define 
//#define 
//#define 
//#define 
//#define 
//#define 
//#define 


typedef enum {
    ESP8266_WifiConnectError_Timeout = 0x1,  
    ESP8266_WifiConnectError_WrongPassword = 0x2,
    ESP8266_WifiConnectError_APNotFound = 0x3,
    ESP8266_WifiConnectError_Failed = 0x4
} ESP8266_Wifi_connect_error_t;

typedef enum Wifi_Mode {STATION_MODE = 1, //Station mode access to AP
                        SOFTAP_MODE,      //AP devices connect to
                        AP_STATION_MODE} //Combine Station and AP
             Wifi_Mode; 

typedef struct Buffer_type {
  uint32_t size; //
	uint32_t write_idx; //position of write
	uint32_t read_idx;  //position of read, buffer is read will be free
	uint8_t* Data;      //buffer to save data, read from USART
	uint8_t Flags;      
	uint8_t End_String; //Character end of string
} Buffer_t;
	
typedef enum ESP8266_Result {ESP8266_OK = 0,
	                           ESP8266_BUSY, //In busy
	                           ESP8266_TIMEOUT, //Command Timeout
	                           ESP8266_INVALID_PARAMETER,
                             ESP8266_MALLOC_ERR}
             ESP8266_Result;

typedef struct ESP8266_Str{
    uint32_t time;
	  uint32_t start_time;
    uint32_t timeout;
	  uint32_t current_command;
    uint32_t last_received_time;
    char* command_response;
    ESP8266_Result last_result;
    ESP8266_Wifi_connect_error_t Wifi_connect_error;
    Wifi_Mode send_mode;
    Wifi_Mode mode;
    struct {
        uint8_t last_operation_status:1;
        uint8_t wait_for_wrapper:1; //wait for "> "
        uint8_t wifi_connected: 1;
        uint8_t wifi_got_ip:1;
    }Flags;
} ESP8266_Str;
	                         


												
#define IS_WIFI_MODE (WIFI_M)   (((WIFI_M) == STATION_MODE) || \
												         ((WIFI_M) == SOFTAP_MODE)  || \
												         ((WIFI_M) == AP_STATION_MODE))

/****************** Function for Buffer **********************/			
/*************************************************************/		
							
uint32_t Buffer_Write_Free(Buffer_t* buff);
uint32_t Buffer_Read_Free(Buffer_t* buff);
uint32_t Buffer_Write(Buffer_t* buff, uint32_t count, uint8_t* dat);
uint32_t Buffer_Read(Buffer_t* buff, uint32_t count, uint8_t* ret_buff);												
uint32_t Buffer_Write_String(Buffer_t* buffer, char* dat);
uint32_t Buffer_Read_String(Buffer_t* buffer, char *save_buff, uint32_t count);
int32_t Buffer_Find_Element(Buffer_t* buffer, uint8_t Element);
int32_t Buffer_Find(Buffer_t* buffer, char* string_find, uint32_t num_find);
int8_t Buffer_Init(Buffer_t* buffer, uint32_t size, uint8_t* buff_main);
void Buffer_Reset(Buffer_t* buffer);

/***************************************************************/
/***************************************************************/



/*************************** USART FUNCTION ********************************/		
/***************************************************************************/					
void Init_UART_Config(void);
void Transmit_UART_Enable(USART_TypeDef* USARTx,  FunctionalState NewState);
void Clear_UART_TxE(USART_TypeDef* USARTx);
void Init_ESP_GPIO(void);
void Transmit_UART(USART_TypeDef* USARTx, uint8_t* dat, uint16_t count);
char* Receive_UART(USART_TypeDef* USARTx, int num_char_receive);
void  Init_USART1_RXNE_Interrupt(USART_TypeDef* USARTx);
/**************************************************************************/
/**************************************************************************/

ESP8266_Result ESP8266_Update(ESP8266_Str* ESP8266);
void Set_Wifi_Mode(Wifi_Mode mode);
void Connect_To_AP(char* ssid, char* password);
ESP8266_Result Send_Command(ESP8266_Str* ESP8266, uint8_t command, char* command_str, char* start_respond);
void ParseReceived(ESP8266_Str* ESP8266, char* received, uint8_t from_usart_buffer, uint16_t bufflen);
void ParseCWJAP(ESP8266_Str* ESP8266, char* received);
								
