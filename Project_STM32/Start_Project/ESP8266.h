//OVER8 = 0 => Oversampling by 16, giam sai so do lech clock
//ONEBIT = 0 => Lay mau 3 lan tai trung tam cua bit
#include <stm32f4xx_usart.h>
#include <stm32f4xx_tim.h> 
#include <stdlib.h>

#define BUFFER_INITIALIZED 0x1
#define BUFFER_MALLOC      0x2

#define UART_TRANS_ENABLE 0x08
#define ESP8266_USART USART1
#define USART_ESP8266_SIZE 1024
#define ESP8266_CONNECTION_BUFFER_SIZE  5096

#define ESP8266_MAX_AP_DETECTED 15
#define ESP8266_MAX_CONNECTIONS 5
#define ESP8266_MAX_CONNECTION  5
#define ESP8266_MAX_SSID_CHAR   32


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
#define ESP8266_COMMAND_SEND_DATA      13
#define ESP8266_COMMAND_USART          14
#define ESP8266_COMMAND_CIPDOMAIN      15
#define ESP8266_COMMAND_CIPSTART       16
//#define 
//#define 
//#define 
//#define 
//#define 
//#define 
//#define 
//#define 
//#define 

TIM_TimeBaseInitTypeDef* TIM_Base_Init;

typedef enum {
    ESP8266_ECN_OPEN = 0x0,
    ESP8266_ECN_WEP,
    ESP8266_ECN_WPA_PSK,
    ESP8266_ECN_WPA2_PSK,
    ESP8266_ECN_WPA_WPA2_PSK
} Encrypt_Method_t;

typedef enum {
    ESP8266_ConnectType_TCP = 0x0,
    ESP8266_Connect_Type_SSL
} ESP8266_Connect_Type;

typedef struct {
    uint8_t active;
    uint8_t number;
    uint8_t cilent;
    uint16_t remote_port;
    uint8_t remote_IP[4];
    ESP8266_Connect_Type connect_type;
    uint32_t byte_received;
    uint32_t total_byte_received;
    uint8_t wait_for_wrapper;
    uint8_t wait_for_respond;
    char* data;
    uint16_t data_size;
    uint8_t last_part;
    uint32_t content_length;
    char* name;
    uint8_t header_done;
    uint8_t first_packet;
    uint8_t last_activity;
} ESP8266_Connection_t;

typedef struct {
    char* SSID;
    char* pass_word;
    uint8_t channel_ID;
    Encrypt_Method_t encrypt_method;
    uint8_t max_connection;
    uint8_t hidden;
} AP_Config_t; //use for command AT+CWJAP

typedef struct {
    Encrypt_Method_t ecn;
    char SSID[ESP8266_MAX_SSID_CHAR];
    int16_t RSSI;
    uint8_t MAC[6];
    uint8_t channel;
    uint8_t offset;
    uint8_t calibration;
} ESP8266_AP_t;

typedef struct {
    char SSID[ESP8266_MAX_SSID_CHAR];
    uint8_t MAC[6];
    uint8_t channel;
    uint8_t RSSI;
} ESP8266_Connected_Wifi_t;

typedef struct {
    ESP8266_AP_t AP[ESP8266_MAX_AP_DETECTED];
    uint8_t AP_valid;//number of valid AP
} ESP8266_Multi_AP_t;   

typedef struct {
    uint8_t IP[4];
    uint8_t MAC[4];
} ESP8266_Connected_Station_t;

typedef struct {
    ESP8266_Connected_Station_t Stations[ESP8266_MAX_CONNECTION];
    uint8_t count;
} ESP8266_Connected_Multi_Station_t;

typedef enum {
    ESP8266_WifiConnect_OK = 0x0,
    ESP8266_WifiConnectError_Timeout = 0x1,  
    ESP8266_WifiConnectError_WrongPassword = 0x2,
    ESP8266_WifiConnectError_APNotFound = 0x3,
    ESP8266_WifiConnectError_Failed = 0x4,
    ESP8266_SetAP_Connect_AP = 0x5,
    ESP8266_SetSTA_Connect_STA = 0x6,
    ESP8266_DNS_Connect_Fail = 0x7,
    ESP8266_Error_TCP_Connection = 0x8,
    ESP8266_Error_SendData_Fail = 0x9
} ESP8266_Wifi_connect_error_t;

typedef enum Wifi_Mode {STATION_MODE = 1, //Station mode access to AP
                        SOFTAP_MODE,      //AP devices connect to
                        AP_STATION_MODE} //Combine Station and AP
             Wifi_Mode; 

typedef enum Error_Type {
    NO_ERR = 0, //No error
    ERR_SET_AP_CONNECT_AP = 0x1, //Setting wifi mode access point but connecting to access point
    ERR_SET_STA_CONNECT_STA  = 0x2,
    DNS_FAIL  = 0x3
} Error_Type;

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
    uint32_t total_byte_received;
    uint32_t total_byte_sent;
    char* command_response;
    uint8_t STA_MAC[4];
    uint8_t STA_IP[4];
    uint8_t STA_gateway[4];
    uint8_t STA_netmask[4];
    uint8_t AP_IP[4];
    uint8_t AP_gateway[4];
    uint8_t AP_netmask[4];
    uint8_t AP_MAC[4];
    uint8_t DNS_IP[4];//IP address for a DNS name
    ESP8266_Result last_result;
    ESP8266_Wifi_connect_error_t Wifi_connect_error;
    Wifi_Mode send_mode;
    Wifi_Mode mode;
    AP_Config_t AP_Config;
    ESP8266_Connected_Wifi_t connected_Wifi;
    Encrypt_Method_t encrypt_method;
    ESP8266_Connected_Multi_Station_t connected_stations;
    ESP8266_Connection_t connection[ESP8266_MAX_CONNECTIONS];
    ESP8266_Connection_t* send_data_connection;
    union {
        uint8_t STA_IP_is_set:1;
        uint8_t STA_netmask_is_set:1;
        uint8_t STA_gateway_is_set:1;
        uint8_t AP_IP_is_set:1;
        uint8_t AP_netmask_is_set:1;
        uint8_t AP_gateway_is_set:1;
        uint8_t last_operation_status:1;
        uint8_t wait_for_wrapper:1; //wait for "> "
        uint8_t wifi_connected: 1;
        uint8_t wifi_got_ip:1;
        uint8_t DNS_connect_success:1;
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
char UART_GetChar (USART_TypeDef* USARTx);
char* Receive_UART(USART_TypeDef* USARTx, int num_char_receive);
void  Init_USART1_RXNE_Interrupt(USART_TypeDef* USARTx);
/**************************************************************************/

/*************************** COUNTER FUNCTION ********************************/		
/***************************************************************************/					
void Init_Counter_ESP8266(TIM_TypeDef* TIMx);//Frequency is 40 MHz
void Init_Interrupt_TIM3(void);
/**************************************************************************/

/*************************** CALL BACK USER FUNCTION ***********************/		
void ESP8266_CallBack_DNS_Fail (ESP8266_Str* ESP8266);
void ESP8266_CallBack_TCP_Connection_Fail(ESP8266_Str* ESP8266);
void ESP8266_Callback_Wifi_Connected(ESP8266_Str* ESP8266);
/***************************************************************************/	
	
/**************************** ESP8266 FUNCTION *****************************/
/***************************************************************************/	
void Initialize_ESP8266(ESP8266_Str* ESP8266);
ESP8266_Result ESP8266_Init(ESP8266_Str* ESP8266);
ESP8266_Result ESP8266_Update(ESP8266_Str* ESP8266);
ESP8266_Result ESP8266_WaitReady(ESP8266_Str* ESP8266);
void Set_Wifi_Mode(Wifi_Mode mode);
void Connect_To_AP(char* ssid, char* password);
ESP8266_Result Send_Command(ESP8266_Str* ESP8266, uint8_t command, char* command_str, char* start_respond);

void Process_SendData(ESP8266_Str* ESP8266);
void ParseReceived(ESP8266_Str* ESP8266, char* received, uint8_t from_usart_buffer, uint16_t bufflen);
void ParseCWJAP(ESP8266_Str* ESP8266, char* received);
void ParseCIPSTA(ESP8266_Str* ESP8266, char* received);
void ParseCWLAP(ESP8266_Str* ESP8266, char* received);
void ParseCWLIF(ESP8266_Str* ESP8266, char* received);
void ParseCWSAP(ESP8266_Str* ESP8266, char* received);
void ParseIP(char* IP_str, uint8_t* arr, uint8_t* cnt);
void ParseMAC(char* ptr, uint8_t* arr, uint8_t* cnt);

uint8_t Hex_To_Num(char ch);								
uint8_t Char_Is_Hex(char ch);
uint32_t Cal_Hex_Num(char* ptr, uint8_t* count);
uint32_t ParseNumber(char* ptr, uint8_t* cnt);
/***************************************************************************/	
