#include "ESP8266.h"
#include "LCD.h"
#include "DHT22.h"
#include "BH1750.h"
#include "YL69.h"

#define CHAR_TO_NUM(x)  ((x) - '0')
#define CHAR_IS_NUM(x)   ((x) >= '0') && ((x) <= '9')
//extern uint16_t dat_humid, dat_temp;
//===============================
#define ESP8266_UPDATE_MACRO(ESP8266)                    \
    if(ESP8266->timeout == 0) {                          \
        ESP8266->timeout = 30;                           \
    }                                                    \
    if (track_count > ESP8266->timeout) {             \
        ESP8266->current_command = ESP8266_COMMAND_IDLE; \
    }                                                    \
    if ((ESP8266->current_command == ESP8266_COMMAND_SEND_DATA) && (ESP8266->Flags.wait_for_wrapper)) { \
            found_wrapper = Buffer_Find(&USART_buffer, "> ", 2); \
            if (found_wrapper == 0) {                            \
                Buffer_Read_String(&USART_buffer, dummy, 2);     \
            }                                                    \
            if (found_wrapper >= 0) {                            \
                Process_SendData(ESP8266);                       \
            }                                                    \
    }                                                            \
    if (ESP8266->current_command == ESP8266_COMMAND_USART) {     \
        if (Buffer_Find(&USART_buffer, "OK\r\n", 4) >= 0) {      \
            Buffer_Reset(&USART_buffer);                         \
            ESP8266->current_command = ESP8266_COMMAND_IDLE;     \
            ESP8266->Flags.last_operation_status = 1;                  \
        }                                                        \
    }                                                            \
    count = sizeof(char_received)/sizeof(char);                  \
    string_length = Buffer_Read_String(&USART_buffer, char_received, count);     \
    while (string_length > 0) {                                                  \
        ParseReceived(ESP8266, char_received, 1, string_length);                 \
        string_length = Buffer_Read_String(&USART_buffer, char_received, count); \
    }
//===============================

#define ESP8266_CHECK_IDLE(ESP8266)                           \
    if ((ESP8266)->current_command != ESP8266_COMMAND_IDLE) { \
        ESP8266_UPDATE_MACRO(ESP8266)                         \
        ESP8266_RETURN_STATUS(ESP8266, ESP8266_BUSY);         \
     }

#define ESP8266_RETURN_STATUS(ESP8266, status) \
    ESP8266->last_result = status;             \
    return status;


//==================== WAIT READY MACRO =======================//
#define ESP8266_WAIT_READY_MACRO(ESP8266)                      \
    do {                                                       \
        if(ESP8266->Flags.wait_for_wrapper) {                  \
            if(Buffer_Find(&USART_buffer, "> ", 2) >= 0) {     \
                break;                                         \
            }                                                  \
        }                                                      \
        ESP8266_UPDATE_MACRO(ESP8266)                          \
    } while(ESP8266->current_command != ESP8266_COMMAND_IDLE); \
    track_count = 0;                                        \
    start_track = 0;                                           
//============================================================

#define  ESP8266_RESET_CONNECTIONS(ESP8266)  memset((uint8_t*)&ESP8266->connected_Wifi, 0, sizeof(ESP8266->connected_Wifi)); 

#define  ESP8266_SERVER_RESPONSE(ESP8266, connect_num, header_content, body_content)               \
    ESP8266_Length_TCP_Buffer(ESP8266, connect_num, strlen(header_content)+strlen(body_content));  \
    Delay_ESP8266(50);                                                                             \
    Transmit_string_UART(ESP8266_USART, header_content);                                           \
    Transmit_string_UART(ESP8266_USART, body_content);                                             \
    Delay_ESP8266(500);                                                                            \
    ESP8266->current_command = ESP8266_COMMAND_IDLE;                                               \
    ESP8266->IPD.in_IPD_mode = 0;                                                                  \
    ESP8266_Close_Connection(ESP8266, connect_num);                           


//====================================================
//uint16_t track_USART_data = 0;
static uint8_t track_count = 0;
static unsigned char start_track = 0;
static ESP8266_Multi_AP_t ESP8266_Multi_AP;
static Buffer_t USART_buffer;
//static Buffer_t TMP_buffer;
static uint8_t USART_data[USART_ESP8266_SIZE];
//static uint8_t TMP_data[TMP_BUFFER_SIZE];
static uint32_t ESP8266_baud_rate[] = {9600,  57600, 115200, 921600};
//static uint8_t tmp_buffer = 0;

/******************************** BUFFER FUNCTION **************************
1. This is circular buffer, have a fixed-number size
2. Have two pointer, head and tail (in code is write_idx - head pointer
and read_idx - tail pointer.
3. When a byte is added to buffer, write_idx will increase 1
4. When a byte is read, read_idx will increase. */


//==========================================================//
//========= Return number of free bytes to write ===========//
//==========================================================//
uint32_t Buffer_Write_Free(Buffer_t* buff) {
      uint32_t in, out, num_byte_free;

      in = buff->write_idx;
      out = buff->read_idx;
    
    if (buff == NULL) {
          return 0;
        }
        
        if (in == out) {
            num_byte_free = buff->size; //Already read all data in buffer
        }
        else if (out > in) {
            num_byte_free = out - in;
        }
        else {
            num_byte_free = buff->size - (in -out);
        }
     
     return (num_byte_free - 1);//Reserved 1 byte to write      
}

//==========================================================//
//========== Return number of free bytes to read ===========//
//==========================================================//
uint32_t Buffer_Read_Free(Buffer_t* buff) {
      //char debug[10];
      uint32_t in, out, num_byte_free;
    
      in = buff->write_idx;
      out = buff->read_idx;
    
    //sprintf(debug, "Write index and read index is %d and %d\r\n", in, out);
    //Transmit_string_UART(USART6, debug);
    if (buff == NULL) {
          return 0;
        }
        
        if (in == out) {
            num_byte_free = 0;//Already read all data
        }
        else if (in > out) {
            num_byte_free = in - out;
        }
        else {
            num_byte_free = buff->size - (out - in);
        }
        
    return num_byte_free;       
}

//==========================================================//
//=================  Write data to buffer  =================//
//==========================================================//
uint32_t Buffer_Write(Buffer_t* buff, uint32_t count, uint8_t* dat) {
      uint32_t max_allow_write, free, num_write;
    
    if ((buff == NULL) || (!count)) {
            return 0;
        }
    max_allow_write = count;
    free = Buffer_Write_Free(buff);
    num_write = 0;

    if (buff->write_idx >= buff->size) {
        buff->write_idx = 0;
    }
    if (!free) {
        return 0;//No empty exit
    }
    if (free < count) {
        max_allow_write = free;
    }

    while (max_allow_write--) {
        buff->Data[buff->write_idx] = *dat;
        buff->write_idx++;
        dat++;
        if (buff->write_idx >= buff->size) {
            buff->write_idx = 0;
        }
        num_write++;

    }
    return num_write;
}

//=======================================================================//
//======================  Read data from buffer  ========================//
//=======================================================================//
uint32_t Buffer_Read(Buffer_t* buff, uint32_t count, uint8_t* ret_buff) {
    uint32_t max_allow_read, free, num_read;
    
    if ((buff == NULL) || (!count)) {
      return 0;
    }
      
    max_allow_read = count;
    free = Buffer_Read_Free(buff);
    num_read = 0;
    
    if (buff->read_idx >= buff->size) {
        buff->read_idx = 0;
    }

    if (!free) {
        return 0;//No empty exit
    }

    if (free < count) {
        max_allow_read = free;
    }
        
    while (max_allow_read--) {
        *ret_buff = buff->Data[buff->read_idx++];
        ret_buff++;
            
        if (buff->read_idx >= buff-> size) {
                buff->read_idx = 0;
        }
        num_read++;
      }
    return num_read;
}

/******************* Reset buffer ******************/
void Buffer_Reset(Buffer_t* buffer) {
    if (buffer == NULL) {
        return;
    }
    buffer->write_idx = 0;
    buffer->read_idx = 0;
}

/***************** Write String Buffer *************/
uint32_t Buffer_Write_String(Buffer_t* buffer, char* dat) {
    return Buffer_Write(buffer, strlen(dat), (uint8_t*)dat);
}


//***********************************************************************//
//*******  Function: Read String Buffer                            ******//
//*******  Para list: 1. buffer read data from                     ******//
//*******             2. save_buffer to save data                  ******//
//*******                read from buffer                          ******//
//*******             3. count to know how many data 8-bit is read ******//                          
//*******  Return:    0 if end character is not in buffer yet      ******//
//***********************************************************************//

uint32_t Buffer_Read_String(Buffer_t* buffer, char *save_buff, uint32_t count) { 
    uint32_t read_reserved, write_reserved, idx;
    uint8_t ch;

    if (buffer == NULL) {
      return 0;
    }

    read_reserved = Buffer_Read_Free(buffer);
    write_reserved = Buffer_Write_Free(buffer);
    idx = 0;

    if ((!read_reserved) || ((read_reserved < count) 
         && (write_reserved > 0) 
         && (Buffer_Find_Element(buffer, buffer->End_String) < 0))) { //Data left remain on USART
        return 0;
    }
    
    while (idx < count) {
        Buffer_Read(buffer, 1, &ch); 
        save_buff[idx] = ch;
        if (ch == buffer->End_String) {
            break;
         }
         idx++;       
    }

    if (idx == count) {
        save_buff[idx] = 0;
    }
    else {
        save_buff[++idx] = 0;
    }
    return idx;     
}

/************************ Init Buffer **************************/
int8_t Buffer_Init(Buffer_t* buffer, uint32_t size, uint8_t* buff_main) {
    memset(buffer, 0, sizeof(Buffer_t));//fill zero start with pointer address of buffer, sizeof
    buffer->size = size;
    buffer->Data = buff_main;
    buffer->End_String = '\n';

    if(!buffer->Data) {
        buffer->Data = (uint8_t*)malloc(size*sizeof(uint8_t));
        if(!buffer->Data) {
            buffer->size = 0;
            return -1;
        }
        else { 
            buffer->Flags |= BUFFER_MALLOC;
        }
    }
    buffer->Flags |= BUFFER_INITIALIZED;
    return 0;
}

/******************** Find character in buffer *****************/
int32_t Buffer_Find_Element(Buffer_t* buffer, uint8_t Element) {
    uint32_t num, out, retval = 0;
    
    if (buffer == NULL) {
        return -1;
    }
    
    num = Buffer_Read_Free(buffer);
    out = buffer->read_idx;
    
    while (num > 0) {
        if (out >= buffer->size) {
            out = 0;
        }
        if ((uint8_t)buffer->Data[out] == (uint8_t)Element) {
            return retval;
        }
    
        out++;
        num--;
        retval++;
    }
    return -1;
}

/****************** Find String in buffer with specified number ************/
int32_t Buffer_Find(Buffer_t* buffer, char* string_find, uint32_t num_find) { 
    uint32_t out, idx, ret_val;   
    uint8_t found;
    int32_t read_reserved;

    read_reserved = Buffer_Read_Free(buffer);
    ret_val = found = idx = 0;
    if (read_reserved < num_find) {
        return -1;
    }
     
    out = buffer->read_idx;
    while (read_reserved > 0) {
        if (string_find[0] == buffer->Data[out]) {
            found = 1;
            idx = 1;
        }
        read_reserved--;
        out++;
        ret_val++;
         
        if (out >= buffer->size) {
            out = 0;
        }
         
        if (found) {
            while (idx < num_find) {
                if (out >= buffer->size) {
                    out = 0;
                }
                if ((uint8_t)string_find[idx] != buffer->Data[out]) {
                    ret_val += idx - 1;
                    found = 0;
                    break;
                }
                out++;
                read_reserved--;
                idx++;
            }                        
            if (idx == num_find) {
                return ret_val-1;
            }        
        }
    }
     return -1;
}

//***************************************************************************//
//******************** FUNCTION FOR INTERRUPT, INITIALIZE UART **************//
//***************************************************************************//
void Init_ESP_GPIO(void) {
    //Enable Clock for PORT B
    GPIO_InitTypeDef GPIO_UART_ESP;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    //Enable System  Config Controller Clock
    GPIO_UART_ESP.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_UART_ESP.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_UART_ESP.GPIO_OType = GPIO_OType_PP;
    GPIO_UART_ESP.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_UART_ESP.GPIO_Mode  = GPIO_Mode_AF; 
    GPIO_Init(GPIOB, &GPIO_UART_ESP);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);
}

//==================================================================//
//============= Initialize Vector Interrupt for USART1 =============//
//==================================================================//
void Init_USART1_RXNE_Interrupt(void) {
    NVIC_InitTypeDef NVIC_Init_Struct;
    NVIC_Init_Struct.NVIC_IRQChannel                   = USART1_IRQn;
    NVIC_Init_Struct.NVIC_IRQChannelPreemptionPriority = 0x00;
    NVIC_Init_Struct.NVIC_IRQChannelSubPriority        = 0x1;
    NVIC_Init_Struct.NVIC_IRQChannelCmd                = ENABLE;

    NVIC_Init(&NVIC_Init_Struct);
}

//==========================================================================//
//====== Handle for interrupt USART1 - write data = 1 char to buffer =======//
//==========================================================================//
void USART1_IRQHandler(void) {
    if (USART_GetITStatus(ESP8266_USART, USART_IT_RXNE)) {
        uint8_t dat = (uint8_t)USART_ReceiveData(ESP8266_USART);
        Buffer_Write(&USART_buffer, 1, &dat);
    }
}

//=================================================================//
//====== Configuration baud rate and enable clockfor USART1 =======//
//=================================================================//
void Init_UART_Config(uint32_t baud_rate) {
    RCC_APB2PeriphClockCmd (RCC_APB2Periph_USART1, ENABLE);
    USART_InitTypeDef Init_UART_Str;
    
    USART_StructInit(&Init_UART_Str);
    Init_UART_Str.USART_BaudRate = baud_rate;
    USART_Init(USART1, &Init_UART_Str);
    USART1->CR1 |= 0x20;//Enable Interrupt UART
    USART_Cmd(USART1, ENABLE);
}

//=================================================================//
//================== Transmit data using USARTx ===================//
//=================================================================//
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

//=================================================================//
//============= Transmit one character using USARTx ===============//
//=================================================================//
char UART_GetChar (USART_TypeDef* USARTx) {
    while (!USART_GetFlagStatus(USARTx, USART_FLAG_RXNE)) {
    }
    return (char)USART_ReceiveData(USARTx);
}

//=================================================================//
//============= Initialize counter to count timeout ===============//
//=================================================================// 
void Init_Counter_ESP8266(TIM_TypeDef* TIMx) {
    TIM_TimeBaseInitTypeDef TIM_Base_Init_ESP8266;
    uint32_t multipler, TIM3_Clk_Frequency, TIM3_Counter_Frequency;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_ClocksTypeDef RCC_Clocks;
    RCC_GetClocksFreq(&RCC_Clocks);
    if (RCC_Clocks.PCLK1_Frequency == RCC_Clocks.SYSCLK_Frequency) {
        multipler = 1;
    }  
    else {
        multipler = 2;
    }
    TIM3_Clk_Frequency = multipler * RCC_Clocks.PCLK1_Frequency;
    TIM3_Counter_Frequency = 1000;//Frequency is 1 kHz
      
    //TIM_InternalClockConfig(TIMx);
    //TIM_Base_Init_ESP8266->TIM_Period = 0x27; //1200
    TIM_Base_Init_ESP8266.TIM_Period = 0x3E7; //1000
    TIM_Base_Init_ESP8266.TIM_Prescaler = (TIM3_Clk_Frequency/TIM3_Counter_Frequency) - 1; //15999
    TIM_Base_Init_ESP8266.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_Base_Init_ESP8266.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_Base_Init_ESP8266.TIM_RepetitionCounter = 0x0; 
    TIM_TimeBaseInit(TIMx, &TIM_Base_Init_ESP8266);
    TIM_ITConfig(TIMx, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIMx, ENABLE);
    Init_Interrupt_TIM3();
}

//=================================================================//
//================= Initialize interrupt counter ==================//
//=================================================================// 
void Init_Interrupt_TIM3(void) {
    NVIC_InitTypeDef NVIC_Init_TIM_Struct;
    NVIC_Init_TIM_Struct.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_Init_TIM_Struct.NVIC_IRQChannelPreemptionPriority = 0x2;
    NVIC_Init_TIM_Struct.NVIC_IRQChannelSubPriority = 0x3;
    NVIC_Init_TIM_Struct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_Init_TIM_Struct);
}

//===================================================//
//========== Initialize interrupt counter ===========//
//===================================================//
void TIM3_IRQHandler(void) {
    if (TIM_GetITStatus(TIM3, TIM_IT_Update)) {
        if (start_track == 1) {
				    //GPIO_ToggleBits(GPIOD, GPIO_Pin_14);
            track_count++;
        }
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
    }
}

//===================================================//
//========== Initialize interrupt counter ===========//
//===================================================//
void TIMx_Reset_CNT(TIM_TypeDef* TIMx) {
    TIMx->CNT = 0x0;
}

//===================================================//
//=============== Delay using counter ===============//
//===================================================//
static void Delay_ESP8266(uint32_t milis) {
    volatile uint32_t count = TIM3->CNT;
    do {
        while ((TIM3->CNT - count) == 0) {
        }
        count = TIM3->CNT;
   } while (--milis);
}

/************************ FUNCTION FOR ESP8266 *******************/
//---------------------- FUNCTION ANALYZE STRING -------------------//
uint8_t Hex_To_Num(char ch) {
    uint8_t ret;
    if ((ch >= '0') && (ch <= '9')) {
        ret = ch - '0';
    }
    else if ((ch >= 'a') && (ch <= 'f')) {
        ret = ch - 'a' + 10;
    }
    else if ((ch >= 'A') && (ch <= 'F')) {
        ret = ch - 'A' + 10;
    }
    else {
        ret = 0;
    }
    return ret;
}

uint8_t Char_Is_Hex(char ch) {
    uint8_t ret;
    if ( ((ch >= '0') && (ch <= '9'))
         || ((ch >= 'a') && (ch <= 'f'))
         || ((ch >= 'A') && (ch <= 'F'))) {
        ret = 1;
    }
    else {
        ret = 0;
    }
    return ret;
}

uint32_t Cal_Hex_Num(char* ptr, uint8_t* count) {
    uint32_t sum = 0;
    uint8_t hex, idx;

    hex = Hex_To_Num(*ptr);
    idx = 0;
    while (hex) {
        sum <<= 4;
        sum += hex;
        ptr++;
        idx++;
        hex = Hex_To_Num(*ptr);
    }

    if (count) {
        *count = idx;
    }
    return sum;
}

//---------- Seperate number from string - such as channel ID in AT+CWSAP ---------//
//---------- Variable cnt is used to for the location that is pointed by pointer
uint32_t ParseNumber(char* ptr, uint8_t* cnt) {
    uint32_t sum;
    uint8_t minus_flag, i;

    sum = minus_flag = i = 0;

    if (*ptr == '-') {
        ptr++;
        minus_flag = 1;
        i++;
    }

    while (*ptr && (CHAR_IS_NUM(*ptr))) {
        sum = 10*sum + (CHAR_TO_NUM(*ptr));
        ptr++;
        i++;
    }

    if (cnt) {
        *cnt = i;
    }

    if (minus_flag) {
        return (0- sum);
    }
    else {
        return sum;
    }
}

//------------ Calculation MAC address                           -----------//
//------------ Input: 1. array to save MAC address is calculated -----------//
//------------        2. pointer of string received              -----------//
//------------        3. Var cnt to count number of segment MAC  -----------//
void ParseMAC(char* ptr, uint8_t* arr, uint8_t* cnt) {
    char* hexptr;
    uint8_t num_hex, tmp_count, sum;

    num_hex = tmp_count = sum = 0;
    hexptr = strtok(ptr, ":");//ac:ef:3d =>arr[0] = ac
    while (hexptr != NULL) {
        arr[num_hex++] = Cal_Hex_Num(ptr, &tmp_count);//one index of array correspond 1 number of MAC respectively
        sum += tmp_count;//count number of character received
        if (num_hex >= 6) {
            break;
        }
        sum++;//ignore ':'
        hexptr = strtok(ptr, ":");
    }

    if (cnt) {
        *cnt = sum;
    }
}


//======================================================//
//===========   Seperate segment of IP     =============//
//===========   IP: 192.168.1.2            =============//
//===========   arr[0] = 192, arr[1] = 168 =============//
//======================================================//
void ParseIP(char* IP_str, uint8_t* arr, uint8_t* cnt) {
    char* token;
    char Data[16];
    uint8_t num_char, idx, ct;

    num_char = idx = 0;
    ////Transmit_UART(USART6, (uint8_t*)IP_str, strlen(IP_str));
    memcpy(Data, IP_str, sizeof(Data) -1);
    Data[sizeof(Data)-1] = 0;
    token = strtok(Data, ".");
    while (token != NULL) {
        arr[idx++] = ParseNumber(token, &ct);
        num_char += ct;

        if (idx >= 4) {
            break;
        }

        num_char++;//ignore '.'
        token = strtok(NULL, ".");
    }
  
    if (cnt != NULL) {
        *cnt = num_char;
    }
}

void ParseCWSAP(ESP8266_Str* ESP8266, char* received) {
    char* ptr;
    uint8_t i, cnt;

    ptr = received;
    i = 0;

    while(*ptr) {
        if (*ptr == ':') {
            break;
        }
    }
    if (*ptr == 0) {
        return;
    }
    ptr++;//ignore ":"
    while(*ptr && !((*ptr != '"') && (*(ptr+1) != ',') && (*(ptr+2) != '"'))) {
        ESP8266->AP_Config.SSID[i++] = *ptr;
        ptr++;
    }
    ESP8266->AP_Config.SSID[i] = 0;
    i = 0;
   
    ptr += 3;
    while(*ptr && !((*ptr != '"') && (*(ptr+1) != ','))) {
        ESP8266->AP_Config.pass_word[i++] = *ptr;
        ptr++;
    }
    ESP8266->AP_Config.pass_word[i++] = 0;

    ptr += 2;//ignore ','
    ESP8266->AP_Config.channel_ID = ParseNumber(ptr, &cnt);
    
    ptr += cnt + 1; //ignore character of channel ID and ','
    ESP8266->AP_Config.encrypt_method =  (Encrypt_Method_t)ParseNumber(ptr, &cnt);
 
    ptr += cnt + 1;//ignore character of encrytion method and ','
    ESP8266->AP_Config.max_connection = ParseNumber(ptr, &cnt);

    ptr += cnt + 1;
    ESP8266->AP_Config.hidden = ParseNumber(ptr, &cnt);
}

//-----------------------------------------------------------------------------------------//   
//------------------ Get IP and Address of Station which is connect to ESP8266 ------------//
//------------------ ESP8266 is soft access point                              ------------//
//-----------------------------------------------------------------------------------------//   
void ParseCWLIF(ESP8266_Str* ESP8266, char* received) { 
    uint8_t count_connected, cnt;

    count_connected = ESP8266->connected_stations.count;
    if (count_connected  > ESP8266_MAX_CONNECTION) {
        return;
    }

    ParseIP(received, ESP8266->connected_stations.Stations[count_connected].IP, &cnt);
    ParseMAC(&received[cnt+1], ESP8266->connected_stations.Stations[count_connected].MAC, NULL);

    ESP8266->connected_stations.count++;
}

void ParseCWLAP(ESP8266_Str* ESP8266, char* received) {
    uint8_t part, start;
    char* ptr;

    part = 0;
    start = 7;//+CWLAP:  <----- position 7
    ptr = strtok(&received[start], ",");
    while (ptr != NULL) {
        switch (part) {//take seperately part
            case 0:
                ESP8266_Multi_AP.AP[ESP8266_Multi_AP.AP_valid].ecn = (Encrypt_Method_t)ParseNumber(ptr, NULL);
                break;
            case 1:
                ptr++;
                ptr[strlen(ptr) -1] = 0;
                strcpy(ESP8266_Multi_AP.AP[ESP8266_Multi_AP.AP_valid].SSID, ptr);
                break;
            case 2:
                ESP8266_Multi_AP.AP[ESP8266_Multi_AP.AP_valid].RSSI = ParseNumber(ptr, NULL);
                break;
            case 3:
                ptr++;
                ptr[strlen(ptr) -1] = 0;
                ParseMAC(ptr, ESP8266_Multi_AP.AP[ESP8266_Multi_AP.AP_valid].MAC , NULL);
                break;          
            case 4: 
                ESP8266_Multi_AP.AP[ESP8266_Multi_AP.AP_valid].channel = ParseNumber(ptr, NULL);
                break;
            case 5: 
                ESP8266_Multi_AP.AP[ESP8266_Multi_AP.AP_valid].offset = ParseNumber(ptr, NULL);
                break;
            case 6: 
                ESP8266_Multi_AP.AP[ESP8266_Multi_AP.AP_valid].calibration = ParseNumber(ptr, NULL);
                break;
            default:
                break;
        }
        ptr = strtok(NULL, ",");
    }
    ESP8266_Multi_AP.AP_valid++;
}

//**************************************************************************//
//************ Function: Parse received data command AT+CIPSTA *************//
//************ Variable type: 1 mean IP, 2 means gateway       *************//
//************                3 means netmask                  *************//
//**************************************************************************//
void ParseCIPSTA(ESP8266_Str* ESP8266, char* received) {
    uint8_t pos, type, command;
   
    ////Transmit_UART(USART6, (uint8_t*)received, strlen(received));
    command = ESP8266_COMMAND_CIPAP;
    if (!strncmp(received, "+CIPSTA_CUR:ip", strlen("+CIPSTA_CUR:ip"))) {
        pos = 14;
        type = 1;
        command = ESP8266_COMMAND_CIPSTA;
    }
    else if (!strncmp(received, "+CIPSTA_CUR:gateway", strlen("+CIPSTA_CUR:gateway"))) {
        pos = 19;
        type = 2;
        command = ESP8266_COMMAND_CIPSTA;
    }
    else if (!strncmp(received, "+CIPSTA_CUR:netmask", 19)) {
        pos = 19;
        type = 3;
        command = ESP8266_COMMAND_CIPSTA;
    }
    else if (!strncmp(received, "+CIPAP_CUR:ip", 13)) {
        pos = 13;
        type = 1;
    }
    else if (!strncmp(received, "+CIPAP_CUR:gateway", 18)) {
        pos = 18;
        type = 2;
    }
    else if (!strncmp(received, "+CIPAP_CUR:netmask", 18)) {
        pos = 18;
        type = 3;
    }
    else {
        return;
    }

    if (command == ESP8266_COMMAND_CIPSTA) {
        switch(type) {
            case 1:
                ParseIP(&received[pos+2], ESP8266->STA_IP, NULL);
                ESP8266->Flags.STA_IP_is_set = 1;                
                break;
            case 2:
                ParseIP(&received[pos+2], ESP8266->STA_gateway, NULL);
                ESP8266->Flags.STA_gateway_is_set = 1;
                break;
            case 3:
                ParseIP(&received[pos+2], ESP8266->STA_netmask, NULL);
                ESP8266->Flags.STA_netmask_is_set = 1;
                break; 
            default:
                break;              
        }
    }
    else {
        switch(type) {
            case 1:
                ParseIP(&received[pos+2], ESP8266->AP_IP, NULL);
                ESP8266->Flags.AP_IP_is_set = 1;
                break;
            case 2:
                ParseIP(&received[pos+2], ESP8266->AP_gateway, NULL);
                ESP8266->Flags.AP_gateway_is_set = 1;               
                break;
            case 3:
                ParseIP(&received[pos+2], ESP8266->AP_netmask, NULL);
                ESP8266->Flags.AP_netmask_is_set = 1;
                break;               
            default:
                break;
        }
    }
            
}
//******************************************************************//
//****** Connect to an access point - AP                      ******//
//****** Response: +CWJAP_CUR:<ssid>,<bssid>,<channel>,<rssi> ******//
//****** Example: +CJWAP_CUR="abc","ca:d7:19:d8:a6:44",4,2    ******//
//******************************************************************//
void ParseCWJAP(ESP8266_Str* ESP8266, char* received) {
    char* ptr = received;
    uint8_t i, cnt;

    i = 0;
    if (!strstr(received, "+CWJAP_")) {
        return;
    }

    while (*ptr && *ptr != '"') {
       ptr++;
    }
    if (*ptr == NULL) {
        return;
    }
    ptr++;//ignore "
    while (*ptr && ((*ptr != '"') || (*(ptr+1) != ',') || (*(ptr+2) != '"'))) {
        ESP8266-> connected_Wifi.SSID[i++] = *ptr;
    }

    ESP8266->connected_Wifi.SSID[i] = 0;
    ptr += 3;
 
    ParseMAC(ptr, ESP8266->connected_Wifi.MAC, &cnt);
    ptr += cnt + 2;//ignore '"' and ','

    ESP8266->connected_Wifi.channel = ParseNumber(ptr, &cnt);
    ptr += cnt + 1;//ignore amount of number character and ','

    ESP8266->connected_Wifi.RSSI = ParseNumber(ptr, &cnt);   
}

//******************************************************************//
//******** Initialize buffer and data structure of ESP8266 *********//
//******************************************************************//
ESP8266_Result ESP8266_Init(ESP8266_Str* const ESP8266, uint32_t baud_rate) {
    uint32_t string_length, count;
    int16_t found_wrapper;
    char char_received[128];
    char dummy[2];

    if(Buffer_Init(&USART_buffer, USART_ESP8266_SIZE, USART_data)) {
        return ESP8266_MALLOC_ERR;
    }
    //Initialize GPIO Pin 
    Init_ESP_GPIO();

    //Initialize baud rate and timeout 
    Init_UART_Config(baud_rate);
 
    //Init USART Interrupt
    Init_USART1_RXNE_Interrupt();

    //Init Counter to count timeout
    Init_Counter_ESP8266(TIM3);

    //Initialize data
    Initialize_data_ESP8266(ESP8266);
    ESP8266->timeout = 1;
    /*
    Send_Command(ESP8266, ESP8266_COMMAND_RST, "AT+RST\r\n", "ready\r\n");

    ESP8266_WAIT_READY_MACRO(ESP8266) 
    if (!ESP8266->Flags.last_operation_status) {
        for (int i=0; i < sizeof(ESP8266_baud_rate)/sizeof(ESP8266_baud_rate[0]); i++) {
            Init_UART_Config(ESP8266_baud_rate[i]);
            ESP8266->timeout = 1;
            Send_Command(ESP8266, ESP8266_COMMAND_RST, "AT+RST\r\n", "ready\r\n");
             ESP8266_WAIT_READY_MACRO(ESP8266)
            if (ESP8266->Flags.last_operation_status) {
                ESP8266->baud_rate = ESP8266_baud_rate[i];
                break;
           }
        }
    }
    else {
        ESP8266->baud_rate = baud_rate;
    }
    
    if (!ESP8266->Flags.last_operation_status) {
        ESP8266_RETURN_STATUS(ESP8266, ESP8266_DEVICE_NOT_CONNECTED);
    } 
    */
    ESP8266->timeout = 3;                    
    Send_Command(ESP8266, ESP8266_COMMAND_AT, "AT\r\n", "OK\r\n");
    ESP8266_WAIT_READY_MACRO(ESP8266)
      
    if (!ESP8266->Flags.last_operation_status) {
        ESP8266_RETURN_STATUS(ESP8266, ESP8266_DEVICE_NOT_CONNECTED);
    }   
    while (ESP8266_Multi_Connection(ESP8266, 1) != ESP8266_OK);
    //
    //while (ESP8266_Remote_IP_Port(ESP8266, 1) != ESP8266_OK);
    //
    while (ESP8266_Set_Wifi_Mode(ESP8266, AP_STATION_MODE) != ESP8266_OK);
    //
    //while (ESP8266_Get_Station_MAC(ESP8266) != ESP8266_OK);
    //
    //while (ESP8266_Get_AP_MAC(ESP8266) != ESP8266_OK);
    //
    //while (ESP8266_Get_AP_IP(ESP8266) != ESP8266_OK);
    //ESP8266_WAIT_READY_MACRO(ESP8266)
    ESP8266->timeout = 10;   

    if (!ESP8266->Flags.last_operation_status) {
        ESP8266_RETURN_STATUS(ESP8266, ESP8266_ERROR);
    }

    ESP8266_RETURN_STATUS(ESP8266, ESP8266_OK)    
}


//============================================================================================================//
//================================= Function Get Data From Web   =============================================//
//============================================================================================================//
ESP8266_Result ESP8266_Get_Data_Web(ESP8266_Str* ESP8266, char* SSID_Wifi, char* password, char* domain_name, char* connection_type) {
    uint32_t string_length, count;
    int16_t found_wrapper;
    char char_received[128];
    char dummy[2];
    char command[50];

    sprintf(command, "AT+CWJAP_CUR=\"%s\",\"%s\"\r\n", SSID_Wifi, password);
    Send_Command(ESP8266, ESP8266_COMMAND_CWJAP_SET, command, "OK\r\n");   
    ESP8266_WAIT_READY_MACRO(ESP8266)
    if (!ESP8266->Flags.last_operation_status) {
        ESP8266_RETURN_STATUS(ESP8266, ESP8266_ERROR);
    }

    sprintf(command, "AT+CIPDOMAIN=\"%s\"\r\n", domain_name);
    Send_Command(ESP8266, ESP8266_COMMAND_CIPDOMAIN, command, "+CIPDOMAIN");   
    ESP8266_WAIT_READY_MACRO(ESP8266)
    if (!ESP8266->Flags.last_operation_status) {
        ESP8266_RETURN_STATUS(ESP8266, ESP8266_ERROR);
    }

    sprintf(command, "AT+CIPSTART=\"%s\",\"%s\",80\r\n", connection_type, ESP8266->DNS_IP);
    Send_Command(ESP8266, ESP8266_COMMAND_CIPSTART, command, "OK\r\n");
    ESP8266_WAIT_READY_MACRO(ESP8266)
    if (!ESP8266->Flags.last_operation_status) {
        ESP8266_RETURN_STATUS(ESP8266, ESP8266_ERROR);
    }

    sprintf(command, "AT+CIPSEND=%d\r\n", strlen("abcxyz"));
    Send_Command(ESP8266, ESP8266_COMMAND_SEND_DATA, command, "OK\r\n");
    ESP8266_WAIT_READY_MACRO(ESP8266)
    if (!ESP8266->Flags.last_operation_status) {
        ESP8266_RETURN_STATUS(ESP8266, ESP8266_ERROR);
    }
    ESP8266_RETURN_STATUS(ESP8266, ESP8266_OK);
}

//============================================================================================================//
//===================================== Make ESP8266 as server   =============================================//
//============================================================================================================//
ESP8266_Result ESP8266_Setting_WebServer(ESP8266_Str* ESP8266, char* SSID_Wifi, char* password, uint16_t port) {
    uint32_t string_length, count;
    int16_t found_wrapper;
    char char_received[128];
    char dummy[2];
    char command[50];

    sprintf(command, "AT+CWJAP_CUR=\"%s\",\"%s\"\r\n", SSID_Wifi, password);
    Send_Command(ESP8266, ESP8266_COMMAND_CWJAP_SET, command, "OK\r\n");   
    ESP8266_WAIT_READY_MACRO(ESP8266)
    if (!ESP8266->Flags.last_operation_status) {
        ESP8266_RETURN_STATUS(ESP8266, ESP8266_ERROR);
    }

    Send_Command(ESP8266, ESP8266_COMMAND_CIPSTA, "AT+CIPSTA_CUR?\r\n", "OK\r\n");
	    do {                                                   
        if(ESP8266->Flags.wait_for_wrapper) {                  
            if(Buffer_Find(&USART_buffer, "> ", 2) >= 0) {     
                break;                                         
            }                                                  
        }                                                      
        if(ESP8266->timeout == 0) {                          
            ESP8266->timeout = 30;                           
        }                
                                               
        if (track_count > ESP8266->timeout) {  
            //GPIO_SetBits(GPIOD, GPIO_Pin_13);           
            ESP8266->current_command = ESP8266_COMMAND_IDLE; 
        }                                                    
        if ((ESP8266->current_command == ESP8266_COMMAND_SEND_DATA) && (ESP8266->Flags.wait_for_wrapper)) { 
                found_wrapper = Buffer_Find(&USART_buffer, "> ", 2); 
                if (found_wrapper == 0) {                            
                    Buffer_Read_String(&USART_buffer, dummy, 2);     
                }                                                    
                if (found_wrapper >= 0) {                            
                    Process_SendData(ESP8266);                       
                }                                                    
        }                                                            
        if (ESP8266->current_command == ESP8266_COMMAND_USART) {     
            if (Buffer_Find(&USART_buffer, "OK\r\n", 4) >= 0) {      
                Buffer_Reset(&USART_buffer);                         
                ESP8266->current_command = ESP8266_COMMAND_IDLE;     
                ESP8266->Flags.last_operation_status = 1;               
            }                                                        
        }                                                            
        count = sizeof(char_received)/sizeof(char);                  
        string_length = Buffer_Read_String(&USART_buffer, char_received, count);     
        while (string_length > 0) {                                                  
            ParseReceived(ESP8266, char_received, 1, string_length);                 
            string_length = Buffer_Read_String(&USART_buffer, char_received, count); 
        }
    } while(ESP8266->current_command != ESP8266_COMMAND_IDLE); 
    track_count = 0;                                        
    start_track = 0;
    //ESP8266_WAIT_READY_MACRO(ESP8266)
                           
    //GPIO_SetBits(GPIOD, GPIO_Pin_13);
    if (!ESP8266->Flags.last_operation_status) {
        ESP8266_RETURN_STATUS(ESP8266, ESP8266_ERROR);
        //GPIO_SetBits(GPIOD, GPIO_Pin_12);
    }

    ESP8266_Create_Server (ESP8266, port);
    //sprintf(command, "AT+CIPSERVER=1,%d\r\n", port);
    //Send_Command(ESP8266, ESP8266_COMMAND_CIPSERVER, command, "OK\r\n");
    //
    //ESP8266_WAIT_READY_MACRO(ESP8266)
    //if (!ESP8266->Flags.last_operation_status) {
    //    ESP8266_RETURN_STATUS(ESP8266, ESP8266_ERROR);
    //}

    //ESP8266_Server_Waiting_For_Request(ESP8266);
    ESP8266_RETURN_STATUS(ESP8266, ESP8266_OK);
}


//=========================================================================================================//
//====================================== Send Command to ESP8266 ==========================================//
//=========================================================================================================//
ESP8266_Result Send_Command(ESP8266_Str* ESP8266, uint8_t command, char* command_str, char* start_respond) {
    uint32_t string_length, count;
    int16_t found_wrapper;
    char char_received[128];
    char dummy[2];

    ESP8266_CHECK_IDLE(ESP8266)
    if (command_str != NULL) {
        Transmit_UART(USART1, (uint8_t*)command_str, strlen(command_str));
    }
    ESP8266->current_command = command;
    TIMx_Reset_CNT(TIM3);//Staring timer from 0
    start_track = 1;//Start count time
    track_count = 0;
    ESP8266_RETURN_STATUS(ESP8266, ESP8266_OK)  
}

//********************************************//
//****** Function process sending data *******//
//********************************************//
void Process_SendData(ESP8266_Str* ESP8266) {
    uint16_t max_buff, found;

    max_buff = 2046;
    ESP8266_Connection_t* connection = ESP8266->send_data_connection;
    unsigned int str_len = strlen(connection->data);

    ESP8266->Flags.wait_for_wrapper = 0;
    //ESP8266->wait_for_wrapper = 0;
    ESP8266->current_command = ESP8266_COMMAND_SEND_DATA;//Set current command is SEND DATA
    if (ESP8266_CONNECTION_BUFFER_SIZE < max_buff) {
        max_buff = ESP8266_CONNECTION_BUFFER_SIZE;
    }
 
    Transmit_UART(USART1, (uint8_t*)connection->data, str_len);
    //ESP8266->total_byte_sent += str_len;
    
    //Send terminated 
    Transmit_UART(USART1, (uint8_t*)("\\0"), 2);
    connection->wait_for_respond = 1;
 
}

//=====================================================================================================//
//====================            Function: Read responding string and parsing  =======================//
//=====================================================================================================// 
void ParseReceived(ESP8266_Str* ESP8266, char* received, uint8_t from_usart_buffer, uint16_t bufflen) {
    uint8_t byte_count;
    uint32_t ipd_pointer_dat = 0;//Add 30/09/19
    ESP8266_Connection_t* connect;

    if (!strcmp(received, "WIFI CONNECTED\r\n")) {
        ESP8266->Flags.wifi_connected = 1;
        ESP8266_Callback_Wifi_Connected(ESP8266);//Callback
    }
    else if (!strcmp(received, "WIFI DISCONNECT\r\n")) {
        ESP8266->Flags.wifi_connected = 0;
        ESP8266->Flags.wifi_got_ip = 0;
        ESP8266_RESET_CONNECTIONS(ESP8266);
        //ESP8266_Callback_Disconnected(ESP8266);
    }
    else if (!strcmp(received, "WIFI GOT IP\r\n")) {
        ESP8266->Flags.wifi_got_ip = 1;
        //ESP8266_Callback_WifiGotIP(ESP8266);
    }
    else {
    }

    if (strstr(received, "SEND OK\r\n") != NULL) {
        ESP8266->current_command = ESP8266_COMMAND_IDLE;
        for (int i=0; i < ESP8266_MAX_CONNECTION; i++) {
            if (ESP8266->connection[i].wait_for_respond) {
                ESP8266->connection[i].wait_for_respond = 0;
            }
        }
    }
    if ((bufflen == 2) && (received[0] == '\r') && (received[1] == '\n')) {
        return;
    }

    //----------- Add for IPD received network data 30/09/19 ---------//
    if (!strncmp(received, "+IPD", 5)) {
        uint16_t ipd_length = bufflen;
        ESP8266->IPD.in_IPD_mode = 1;

        ipd_pointer_dat = 5;

        ESP8266->IPD.connection_num = CHAR_TO_NUM(received[ipd_pointer_dat]);
        connect = &ESP8266->connection[ESP8266->IPD.connection_num];
        connect->number = ESP8266->IPD.connection_num;    

        ipd_pointer_dat += 2;
        connect->byte_received = ParseNumber(&received[ipd_pointer_dat], &byte_count);          
        if (!connect->total_byte_received) {
             connect->header_done = 0;
             connect->first_packet = 1;
        }
        else {
             connect->first_packet = 0;
        }
        connect->total_byte_received += connect->byte_received;

        ipd_pointer_dat += byte_count + 1;
        ParseIP(&received[ipd_pointer_dat], connect->remote_IP, &byte_count);

        //ipd_pointer_dat += byte_count + 1;
        ipd_pointer_dat = 0;
        while ((received[ipd_pointer_dat] != ':') && (ipd_pointer_dat < ipd_length)) {
            ipd_pointer_dat++; 
        }

        if ((ipd_length - ipd_pointer_dat) > connect->byte_received) {
            ipd_length = connect->byte_received + ipd_pointer_dat;
        }

        memcpy((uint8_t*)connect->data, (uint8_t*)&received[ipd_pointer_dat], ipd_length - ipd_pointer_dat);
        if ((ipd_length - ipd_pointer_dat) > connect->byte_received) {
            connect->data[connect->byte_received] = 0;
        }

        ESP8266->IPD.pointer_dat = ESP8266->IPD.pointer_total = ipd_length - ipd_pointer_dat;
  
        if (ipd_pointer_dat > connect->byte_received) {
            ESP8266->IPD.in_IPD_mode = 0;
            connect->data_size = ipd_pointer_dat;
            connect->last_part = 1;
        }      
    }
    //----------------------------------------------------------------/

    switch (ESP8266->current_command) {
        case ESP8266_COMMAND_CWMODE:
            if (!strcmp(received, "OK\r\n")) {
                ESP8266->current_command = ESP8266_COMMAND_IDLE;
                //ESP8266->mode = ESP8266->send_mode;
            }
            break;

        case ESP8266_COMMAND_CWJAP_SET:
            if (!strncmp(received, "+CWJAP_CUR:", 11)) {
                strcpy(ESP8266->command_response, "FAIL\r\n");
                ESP8266->Wifi_connect_error = (ESP8266_Wifi_connect_error_t)CHAR_TO_NUM(received[11]);
            }
            if (!strcmp(received, "OK\r\n")) {
                ESP8266->current_command = ESP8266_COMMAND_IDLE;
            }
            if (!strcmp(received, "FAIL\r\n")) {
                ESP8266->current_command = ESP8266_COMMAND_IDLE;
                ESP8266_Callback_WifiConnectFailed(ESP8266);
            }
        break;
        
        case ESP8266_COMMAND_CWJAP_GET:
            if (!strncmp(received, "+CWJAP_CUR:", 11)) {
                ParseCWJAP(ESP8266, received);
            }
            if (!strcmp(received, "OK\r\n")) {
                ESP8266->current_command = ESP8266_COMMAND_IDLE;
            }
            break;
      
        case ESP8266_COMMAND_CWLAP:
            if (!strncmp(received, "+CWLAP:",7)) {
                ParseCWLAP(ESP8266, received);
            }
            if (!strcmp(received, "OK\r\n")) {
                 ESP8266->current_command = ESP8266_COMMAND_IDLE;
            }
            break;

        case ESP8266_COMMAND_CWSAP:
            if (!strncmp(received, "+CWSAP", 6)) {
                ParseCWSAP(ESP8266, received);
            }
            if (!strcmp(received, "OK\r\n")) {
                 ESP8266->current_command = ESP8266_COMMAND_IDLE;
            } 
            break;

        case ESP8266_COMMAND_CWLIF:
            if (CHAR_IS_NUM(received[0])) {
                ParseCWLIF(ESP8266, received);
            }
            if (!strcmp(received, "OK\r\n")) {
                ESP8266->current_command = ESP8266_COMMAND_IDLE;
                //ESP8266_Callback_ConnectedStationsDetected(ESP8266, &ESP8266->ConnectedStations);
            }
            break;

        case ESP8266_COMMAND_CIPSTAMAC:
            if (!strncmp(received, "+CIPSTAMAC", 10)) {
                 ParseMAC(&received[12], ESP8266->STA_MAC, NULL);
            }       
            if (!strcmp(received, "OK\r\n")) {
                 ESP8266->current_command = ESP8266_COMMAND_IDLE;
            }
            break;

        case ESP8266_COMMAND_CIPAPMAC:
            if (!strncmp(received, "+CIPAPMAC", 9)) {
                ParseMAC(&received[12], ESP8266->AP_MAC, NULL);
            }
            if (!strcmp(received, "OK\r\n")) {
                ESP8266->current_command = ESP8266_COMMAND_IDLE;
            }
            break;

        case ESP8266_COMMAND_CIPSTA:
            if (!strncmp(received, "+CIPSTA_CUR",11)) {
                ParseCIPSTA(ESP8266, received);
            }
            if (!strcmp(received, "OK\r\n")) {
				//GPIO_SetBits(GPIOD, GPIO_Pin_12);
                ESP8266->current_command = ESP8266_COMMAND_IDLE;
                //ESP8266_Callback_WifiIPSet(ESP8266);
            }
            break;

        case ESP8266_COMMAND_CIPAP:
            if (!strncmp(received, "+CIPAP", 6)) {
                ParseCIPSTA(ESP8266, received);
            }
            if (!strcmp(received, "OK\r\n")) {
                ESP8266->current_command = ESP8266_COMMAND_IDLE;
                ESP8266_Callback_WifiIPSet(ESP8266);
            }
            break;
                
        case ESP8266_COMMAND_CIPDOMAIN:
            if (!strncmp(received, "+CIPDOMAIN", 10)) {
                ////Transmit_UART(USART6, (uint8_t*)received, strlen(received));
                //ParseIP(received+11, ESP8266->DNS_IP, &byte_count); 
                strcpy(ESP8266->DNS_IP, &received[11]);
                if (ESP8266->Wifi_connect_error == ESP8266_DNS_Connect_Fail) {
                    ESP8266->Wifi_connect_error =  ESP8266_WifiConnect_OK;
                }
            }
            else if (!strncmp(received, "DNS Fail", 8)) { 
                ESP8266->current_command = ESP8266_COMMAND_IDLE;
                ESP8266_CallBack_DNS_Fail (ESP8266);
            }
            break;

       case ESP8266_COMMAND_CIPSTART:
            if (!strcmp(received, "OK\r\n")) {
                if (ESP8266->Wifi_connect_error == ESP8266_Error_TCP_Connection) {
                    ESP8266->Wifi_connect_error =  ESP8266_WifiConnect_OK;
                }
            }
            else if (!strcmp(received, "ERROR\r\n")) {
                ESP8266->current_command = ESP8266_COMMAND_IDLE;
                ESP8266_CallBack_TCP_Connection_Fail(ESP8266);
            }
            else if (!strncmp(received, "ALREADY CONNECTED", 17)) {
                ESP8266->Flags.last_operation_status = 1;
            }
    }

    //ESP8266->last_received_time = ESP8266->time;
    if (!strcmp(received, "OK\r\n") || !strcmp(received, "ready\r\n")) {
        ESP8266->Flags.last_operation_status = 1;
        if (ESP8266->current_command == ESP8266_COMMAND_CIPCLOSE) {
            ESP8266->current_command = ESP8266_COMMAND_IDLE;
        }
    }
    if ((!strcmp(received, "ERROR\r\n")) || (!strcmp(received, "busy p...\r\n"))) {
        ESP8266->Flags.last_operation_status = 0;
        ESP8266->current_command = ESP8266_COMMAND_IDLE;
    }

    if (!strcmp(received, "SEND OK\r\n")) {
        ESP8266->current_command = ESP8266_COMMAND_IDLE;
        ESP8266->Flags.wait_for_wrapper = 0;
    }
}


//*******************************************************************************//
//************************      CALL BACK FUNCTION      *************************//
//*******************************************************************************//
void ESP8266_Callback_Wifi_Connected(ESP8266_Str* ESP8266) {
    ESP8266->Wifi_connect_error =  ESP8266_WifiConnect_OK;
}

//==============================================================================//
//=============== Call back function when getting DNS domain fail ==============//
//==============================================================================//
void ESP8266_CallBack_DNS_Fail (ESP8266_Str* ESP8266) {
    ESP8266->Wifi_connect_error = ESP8266_DNS_Connect_Fail;
}

//==============================================================================//
//============= Call back function when establishing TCP connection ============//
//==============================================================================//
void ESP8266_CallBack_TCP_Connection_Fail(ESP8266_Str* ESP8266) {
    ESP8266->Wifi_connect_error = ESP8266_Error_TCP_Connection;
}

//==============================================================================//
//============ Call back function when connecting to Wifi AP FAIL ==============//
//==============================================================================//
void ESP8266_Callback_WifiConnectFailed(ESP8266_Str* ESP8266) {  
   if (ESP8266->Wifi_connect_error == ESP8266_WifiConnectError_Timeout) {
       //Transmit_UART(USART6, (uint8_t*)"Connection Timeout\n", strlen("Connection Timeout\n"));
       //Transmit_UART(USART6, (uint8_t*)"Try to connect again\n", strlen("Try to connect again\n"));
   }
   else if (ESP8266->Wifi_connect_error == ESP8266_WifiConnectError_WrongPassword) {        
       //Transmit_UART(USART6, (uint8_t*)"Wrong password\n", strlen("Wrong Password\n"));
       //Transmit_UART(USART6, (uint8_t*)"Please check password again\n", strlen("Please check password again\n"));
   }
   else if (ESP8266->Wifi_connect_error == ESP8266_WifiConnectError_APNotFound) {
       //Transmit_UART(USART6, (uint8_t*)"Access Point is not found\n", strlen("Access Point is not found\n"));
       //Transmit_UART(USART6, (uint8_t*)"Please check SSID again\n", strlen("Please check SSID again\n"));
   }
   else if (ESP8266->Wifi_connect_error == ESP8266_WifiConnectError_Failed) {
       //Transmit_UART(USART6, (uint8_t*)"Connection Failed\n", strlen("Connection Failed\n"));
       //Transmit_UART(USART6, (uint8_t*)"Please check the connection again\n", strlen("Please check the connection again\n"));
   }
}

//==========================================================================//
//======== Call back function when Wifi AP is assigned IP address ==========//
//==========================================================================//
void ESP8266_Callback_WifiIPSet(ESP8266_Str* ESP8266) {
    char ch_IP[5];
    uint8_t idx = 0;
    //Transmit_UART(USART6, (uint8_t*)"\rEnter following URL to get HTML Web:\r\n", strlen("\rEnter following URL to get HTML Web:\r\n"));
    for (idx=0; idx < 4; idx++) { 
        if (ESP8266->current_command == ESP8266_COMMAND_CIPAP) {   
            sprintf(ch_IP, "%d", ESP8266->AP_IP[idx]);
        }
        else {
            sprintf(ch_IP, "%d", ESP8266->STA_IP[idx]);
       }
        //Transmit_UART(USART6, (uint8_t*)ch_IP, strlen(ch_IP)); 
        if (idx != 3) {
            //Transmit_UART(USART6, (uint8_t*)".", 1);
        }
    }
    //Transmit_UART(USART6, (uint8_t*)"\n\n", 2);
}

//==========================================================================================//
//============  Function: Paser received IPD data when ESP8266 acts as server ==============//
//==========================================================================================//
ESP8266_Result Parse_IPD_Data_Received(ESP8266_Str* ESP8266, char* received, uint16_t bufflen) {
    uint8_t byte_count;
    uint32_t ipd_pointer_dat = 0;//Add 30/09/19
    ESP8266_Connection_t* connect;
    char buff_read[128];

    if (!strncmp(received, "+IPD,", 5)) {
        //uint16_t ipd_length = bufflen;
        ESP8266->IPD.in_IPD_mode = 1;

        ipd_pointer_dat = 5;

        ESP8266->IPD.connection_num = CHAR_TO_NUM(received[ipd_pointer_dat]);
        connect = &ESP8266->connection[ESP8266->IPD.connection_num];
        connect->number = ESP8266->IPD.connection_num;    
        connect->active = 1;
        connect->call_data_received = 1;
        ipd_pointer_dat += 2;
        connect->byte_received = ParseNumber(&received[ipd_pointer_dat], &byte_count);          
        if (!connect->total_byte_received) {
             connect->header_done = 0;
             connect->first_packet = 1;
        }
        else {
             connect->first_packet = 0;
        }
        connect->total_byte_received += connect->byte_received;

        ipd_pointer_dat += byte_count + 1;

        //================================== Process IPD data ================================//
        if (strstr(received, "GET / HTTP/1.1")) { //Request get web page
            connect->command = GET_WEB_PAGE;   
        }
        else if (strstr(received, "GET /readADC")) {
            connect->command = ADC_READ_VALUE;
        }
        else if (strstr(received, "GET /readYL69")) {
            connect->command = YL69_READ_VALUE;
        }
        else if (strstr(received, "GET /readBH1750")) {
            connect->command = BH1750_READ_VALUE;
        }
        else if (strstr(received, "GET /setLight?Lightstate=1")) {
            connect->command = TURN_LIGHT_ON;
        }
        else if (strstr(received, "GET /setLight?Lightstate=0")) {
            connect->command = TURN_LIGHT_OFF;
        }
        else if (strstr(received, "GET /setMotor?Motorstate=1")) {
            connect->command = TURN_MOTOR_ON;
        }
        else if (strstr(received, "GET /setMotor?Motorstate=0")) {
            connect->command = TURN_MOTOR_OFF;
        } 
    }
    ESP8266_RETURN_STATUS(ESP8266, ESP8266_OK);
}


//========================================================================================================================//
//=============================                Send data to client - browser           ===================================//
//=============================                GET / : Send data web page HTML         ===================================//
//========================================================================================================================//
ESP8266_Result ESP8266_CallBack_Server_ConnectionData_Received(ESP8266_Str* ESP8266, uint8_t command, uint8_t connect_num) {
    char* content = "<!DOCTYPE html>"
                    "<html>"
		                "<head>"
					          "<meta charset=\"UTF-8\">"
					          "<meta name=\"description\" content=\"Thesis IoT\">"
					          "<meta name=\"author\" content=\"AV\">"
					          "<title>Luan van tot nghiep</title>"
					          "<link rel=\"icon\" href=\"data:image/x-icon;base64,iVBORw0KGgoAAAANSUhEUgAAABAAAAAQCAYAAAAf8/9hAAAAS0lEQVR42s2SMQ4AIAjE+P+ncSY"
                    "dasgNXMJgcyIIlVKPIKdvioAXyWBeJmVpqRZKWtj9QWAKZyWll50b8IcL9JUeQF50n28ckyb0ADG8RLwp05YBAAAAAElFTkSuQmCC\" type=\"image/x-icon\" />"
					          "<head/>"
                    "<body>"
                    "<div id=\"demo\">"
                    "<h1>The ESP8266 NodeMCU Update web page without refresh</h1>"
                    "	<button type=\"button\" onclick=\"LightOn(1)\">LGIHT ON</button>"
                    "	<button type=\"button\" onclick=\"LightOn(0)\">LIGHT OFF</button><br/>"
                    "	<button type=\"button\" onclick=\"MotorOn(1)\">MOTOR ON</button>"
                    "	<button type=\"button\" onclick=\"MotorOn(0)\">MOTOR OFF</button><br/>"
                    "</div>"
                    "<div>"
                    "DHT22 temperature value is : <span id=\"DHT22_temp\">0</span><br/>"
                    "DHT22 humid value is : <span id=\"DHT22_humid\">0</span><br/>"
                    "Moisture Value is : <span id=\"YL69Value\">0</span><br/>"
                    "BH1750 Value is: <span id=\"BH1750Value\">0</span><br/>"
                    "Motor State is : <span id=\"MotorState\">NA</span><br/>"
                    "Light State is : <span id=\"LightState\">NA</span>"
                    "</div>"
                    "<script>"
                    "function LightOn(state) {"
                    "  var xhttp = new XMLHttpRequest();"
                    "  xhttp.onreadystatechange = function() {"
                    "    if (this.readyState == 4 && this.status == 200) {"
                    "      document.getElementById(\"LightState\").innerHTML ="
                    "      this.responseText;"
                    "    }"
                    "  };"
                    "  xhttp.open(\"GET\", \"setLight?Lightstate=\"+state, true);"
                    "  xhttp.send();"
                    "}"
                    "function MotorOn(state) {"
                    "  var xhttp = new XMLHttpRequest();"
                    "  xhttp.onreadystatechange = function() {"
                    "    if (this.readyState == 4 && this.status == 200) {"
                    "      document.getElementById(\"MotorState\").innerHTML ="
                    "      this.responseText;"
                    "    }"
                    "  };"
                    "  xhttp.open(\"GET\", \"setMotor?Motorstate=\"+state, true);"
                    "  xhttp.send();"
                    "}"
                    //"setInterval(function() {"
                    //    "getDataADC();"
                    //    "getDataYL69();"
                    //    "getDataBH1750();"
                    //"}, 10000);"
                    "function getDataADC() {"
                    "var xhttp = new XMLHttpRequest();"
                    "xhttp.onreadystatechange = function() {"
                    "  if (this.readyState == 4 && this.status == 200) {"
                    "    var str = this.responseText.split(' ');"
                    "    document.getElementById(\"DHT22_temp\").innerHTML = str[0];"
                    "    document.getElementById(\"DHT22_humid\").innerHTML = str[1];"
                    "  }"
                    "};"
                    "xhttp.open(\"GET\", \"readADC\", true);"
                    "xhttp.send();"
                    "}"
                    "function getDataYL69() {"
                    "var xhttp = new XMLHttpRequest();"
                    "xhttp.onreadystatechange = function() {"
                    "  if (this.readyState == 4 && this.status == 200) {"
                    "    document.getElementById(\"YL69Value\").innerHTML ="
                    "    this.responseText;"
                    "  }"
                    "};"
                    "xhttp.open(\"GET\", \"readYL69\", true);"
                    "xhttp.send();"
                    "}"
                    "function getDataBH1750() {"
                    "var xhttp = new XMLHttpRequest();"
                    "xhttp.onreadystatechange = function() {"
                    "  if (this.readyState == 4 && this.status == 200) {"
                    "    document.getElementById(\"BH1750Value\").innerHTML ="
                    "    this.responseText;"
                    "  }"
                    "};"
                    "xhttp.open(\"GET\", \"readBH1750\", true);"
                    "xhttp.send();"
                    "}"
                    "</script>"
                    "<script type=\"text/javascript\" src=\"https://cdn.fusioncharts.com/fusioncharts/latest/fusioncharts.js\"></script>"
                    "</body>"
                    "</html>";

    char* response_HTTP =  "HTTP/1.1 200 OK\r\n"
                           "Content-Type: text/html"
                           "\r\n\r\n";

   char* response_ajax  = "HTTP/1.1 200 OK\r\n"
                          "Content-Type: text/plain"
                          "\r\n\r\n";
   char display_value[10];
   char* next_response_HTTP;
   static uint8_t ADC_num;
   uint8_t first_response;
   int32_t step, BH1750_dat;
   static uint16_t pre_humid, pre_temp;
   uint16_t humid, temp;
   float YL69_dat;
   size_t len_html, len_transmit;

   ADC_num = step = BH1750_dat = 0;
   humid = temp = 0;
   first_response = 1;
   len_html = strlen(content) + strlen(response_HTTP);
   len_transmit = 0;    
   next_response_HTTP = content;
    if (command == GET_WEB_PAGE) {
        do {
            if (len_html > ESP8266_MAX_PACKET_IPD) {
                len_transmit = ESP8266_MAX_PACKET_IPD; 
                len_html = len_html - ESP8266_MAX_PACKET_IPD;
            }
            else {
                len_transmit = len_html;
                len_html = 0;
           }
            ESP8266_Length_TCP_Buffer(ESP8266, connect_num, len_transmit);
            Delay_ESP8266(50);            
            
            //-------------- Sending reponse HTTP and Web Page ------------//
            if (first_response) {
						    Transmit_string_UART(ESP8266_USART, response_HTTP);
                Transmit_UART(ESP8266_USART, (uint8_t*)content, len_transmit - strlen(response_HTTP));
            }
            else {
                Transmit_UART(ESP8266_USART, (uint8_t*)next_response_HTTP, len_transmit);
            }

            if (first_response) {
                next_response_HTTP = content + len_transmit - strlen(response_HTTP);
                first_response = 0;
            }
            else {
                next_response_HTTP += len_transmit;
            }
            //-------------------------------------------------------------//
            
            //Waiting for receiving SEND OK
            Delay_ESP8266(300);
        
            //Close connection to browser         
        }
        while (len_html > 0);
        ESP8266->current_command = ESP8266_COMMAND_IDLE;
        ESP8266->IPD.in_IPD_mode = 0;//no in IPD mode anymore
        ESP8266_Close_Connection(ESP8266, connect_num);
    }
    else if (command == ADC_READ_VALUE) {
        DHT22_Get_Humid_Temp(&humid, &temp);
        if (humid) {
            pre_humid = humid;
            pre_temp = temp;
        }
        else {
            humid = pre_humid;
            temp = pre_temp;
        }
        sprintf(display_value, "%.1f %.1f", (float)temp/10, (float)humid/10);
        ESP8266_SERVER_RESPONSE(ESP8266, connect_num, response_ajax , display_value);
    }
    else if (command == YL69_READ_VALUE) {
        YL69_dat =  Percent_Soil_YL69 ();
        sprintf(display_value, "%.1f", YL69_dat);
        ESP8266_SERVER_RESPONSE(ESP8266, connect_num, response_ajax , display_value); 
    }
    else if (command == BH1750_READ_VALUE) {
        BH1750_dat = BH1750_Read();
        sprintf(display_value, "%d", BH1750_dat);
        ESP8266_SERVER_RESPONSE(ESP8266, connect_num, response_ajax, display_value); 
    }
    else if (command == TURN_LIGHT_ON) {
        GPIO_SetBits(GPIOD, GPIO_Pin_0);
        ESP8266_SERVER_RESPONSE(ESP8266, connect_num, response_ajax , "ON");  
    }
    else if (command == TURN_LIGHT_OFF) {
        GPIO_ResetBits(GPIOD, GPIO_Pin_0);
        ESP8266_SERVER_RESPONSE(ESP8266, connect_num, response_ajax , "OFF");
    }
    else if (command == TURN_MOTOR_ON) {
        GPIO_SetBits(GPIOD, GPIO_Pin_1);
        ESP8266_SERVER_RESPONSE(ESP8266, connect_num, response_ajax , "ON");
   }
    else if (command == TURN_MOTOR_OFF) {
        GPIO_ResetBits(GPIOD, GPIO_Pin_1);
        ESP8266_SERVER_RESPONSE(ESP8266, connect_num, response_ajax , "OFF");
    }
    ESP8266_RETURN_STATUS(ESP8266, ESP8266_OK);
}
//====================================================================//
//=============== Call back function for 5 connection ================//
//====================================================================//
void ESP8266_Call_Connection_CallBack(ESP8266_Str* ESP8266) {
    uint8_t connection_num;
    for (connection_num = 0; connection_num < ESP8266_MAX_CONNECTION; connection_num++) {
        if (ESP8266->connection[connection_num].active && ESP8266->connection[connection_num].call_data_received) {
            if ((!ESP8266->connection[connection_num].cilent) && (ESP8266->current_command != ESP8266_COMMAND_IDLE)) {
                continue;
            }

            if (ESP8266->connection[connection_num].cilent) {
                //ESP8266_CallBack_Client_ConnectionData_Received(ESP8266, &ESP8266->connection[connection_num], ESP8266->connection[connection_num].data);
            }
            else {
               // ESP8266_CallBack_Server_ConnectionData_Received(ESP8266, &ESP8266->connection[connection_num]);
            }   
        }    
    }
}
//*******************************************  END OF CALL BACK FUNTION *************************************************//

//------------------ Initialize ESP8266 ------------//
//---------- Reset all variables to 0 --------------//
void Initialize_data_ESP8266(ESP8266_Str* ESP8266) {
    if (ESP8266 == NULL) {
        ESP8266 = (ESP8266_Str*)malloc(1*sizeof(ESP8266_Str));
    }
    //ESP8266->time = 0;
    //ESP8266->start_time = 0;
    ESP8266->timeout = 0;
    ESP8266->current_command = ESP8266_COMMAND_IDLE;
    //ESP8266->last_received_time = 0;
    //ESP8266->total_byte_received = 0;
    //ESP8266->total_byte_sent = 0;

    if (ESP8266->command_response == NULL) {
        ESP8266->command_response = (char*)malloc(30 * sizeof(char));
    }

    for (int i = 0; i < sizeof(ESP8266->STA_IP)/sizeof(ESP8266->STA_IP[0]); i++) {
        ESP8266->STA_IP[i] = 0;
    }

    for (int i = 0; i < sizeof(ESP8266->STA_gateway)/sizeof(ESP8266->STA_gateway[0]); i++) {
        ESP8266->STA_gateway[i] = 0;
    }

    for (int i = 0; i < sizeof(ESP8266->STA_netmask)/sizeof(ESP8266->STA_gateway[0]); i++) {
        ESP8266->STA_netmask[i] = 0;
    }

    for (int i = 0; i < sizeof(ESP8266->STA_MAC)/sizeof(ESP8266->STA_MAC[0]); i++) {
        ESP8266->STA_MAC[i] = 0;
    }    

    for (int i = 0; i < sizeof(ESP8266->AP_IP)/sizeof(ESP8266->AP_IP[0]); i++) {
        ESP8266->AP_IP[i] = 0;
    }

    for (int i = 0; i < sizeof(ESP8266->AP_gateway)/sizeof(ESP8266->AP_gateway[0]); i++) {
        ESP8266->AP_gateway[i] = 0;
    }

    for (int i = 0; i < sizeof(ESP8266->AP_netmask)/sizeof(ESP8266->AP_gateway[0]); i++) {
        ESP8266->AP_netmask[i] = 0;
    }

    for (int i = 0; i < sizeof(ESP8266->AP_MAC)/sizeof(ESP8266->AP_MAC[0]); i++) {
        ESP8266->AP_MAC[i] = 0;
    }

    
    ESP8266->send_data_connection = (ESP8266_Connection_t*)malloc(1*sizeof(ESP8266_Connection_t));
    ESP8266->last_result = ESP8266_OK;
    ESP8266->Wifi_connect_error = ESP8266_WifiConnect_OK;

    ESP8266->Flags.STA_IP_is_set         = 0x0;
    ESP8266->Flags.STA_netmask_is_set    = 0x0;
    ESP8266->Flags.STA_gateway_is_set    = 0x0;
    ESP8266->Flags.STA_MAC_is_set        = 0x0;
    ESP8266->Flags.AP_IP_is_set          = 0x0;
    ESP8266->Flags.AP_netmask_is_set     = 0x0;
    ESP8266->Flags.AP_gateway_is_set     = 0x0;
    ESP8266->Flags.AP_MAC_is_set         = 0x0;
    ESP8266->Flags.last_operation_status = 0x0;
    ESP8266->Flags.wait_for_wrapper      = 0x0;
    ESP8266->Flags.wifi_connected        = 0x0;
    ESP8266->Flags.wifi_got_ip           = 0x0;
    ESP8266->Flags.DNS_connect_success   = 0x0;
}

//==============================================================//
//====================   Set Mode Wifi     =====================//      
//==============================================================//
ESP8266_Result ESP8266_Set_Wifi_Mode(ESP8266_Str* ESP8266, Wifi_Mode mode) {
    uint32_t string_length, count;
    int16_t found_wrapper;
    char char_received[128];
    char dummy[2];

      assert_param(IS_WIFI_MODE(mode));
      char ch_mode = (char)(mode + '0');

    Transmit_UART(ESP8266_USART, (uint8_t*)"AT+CWMODE_CUR=", strlen("AT+CWMODE_CUR="));
    Transmit_UART(ESP8266_USART, (uint8_t*)&ch_mode, 1);
    Transmit_UART(ESP8266_USART, (uint8_t*)"\r\n", 2);

    if (Send_Command(ESP8266, ESP8266_COMMAND_CWMODE, NULL, NULL) != ESP8266_OK) {
        return ESP8266->last_result;
    }

    ESP8266_WAIT_READY_MACRO(ESP8266);
    if (!ESP8266->Flags.last_operation_status) {
        ESP8266_RETURN_STATUS(ESP8266, ESP8266_ERROR);
    }
 
    ESP8266_RETURN_STATUS(ESP8266, ESP8266_OK);
}

//==============================================================//
//================   Get MAC Station ESP8266  ==================//      
//==============================================================//
ESP8266_Result ESP8266_Get_Station_MAC(ESP8266_Str* ESP8266) {
    Send_Command(ESP8266, ESP8266_COMMAND_CIPSTAMAC, "AT+CIPSTAMAC_CUR?\r\n", "+CIPSTAMAC_CUR");
    if (ESP8266->last_result == ESP8266_OK) {
        ESP8266->Flags.STA_MAC_is_set = 0;
        //ESP8266->STA_MAC_is_set = 0;
    }
    start_track = 0;
    return ESP8266->last_result;
}

//==============================================================//
//==============  Get MAC Access Point ESP8266    ==============//      
//==============================================================//
ESP8266_Result ESP8266_Get_AP_MAC(ESP8266_Str* ESP8266) {
    Send_Command(ESP8266, ESP8266_COMMAND_CIPAPMAC, "AT+CIPAPMAC_CUR?\r\n", "+CIPAPMAC_CUR");
    if (ESP8266->last_result == ESP8266_OK) {
        ESP8266->Flags.AP_MAC_is_set = 0;
        //ESP8266->AP_MAC_is_set = 0;
    }
    start_track = 0;
    return ESP8266->last_result;
}

//===================================================================================//
//=======================   Get IP Access Point ESP8266    ==========================//      
//===================================================================================//
ESP8266_Result ESP8266_Get_AP_IP(ESP8266_Str* ESP8266) {
    Send_Command(ESP8266, ESP8266_COMMAND_CIPAPMAC, "AT+CIPAP_CUR?\r\n", "+CIPAP_CUR");
    if (ESP8266->last_result == ESP8266_OK) {
        ESP8266->Flags.AP_IP_is_set = 0;
        ESP8266->Flags.AP_netmask_is_set = 0;
        ESP8266->Flags.AP_gateway_is_set = 0;
    }
    start_track = 0;
    return ESP8266->last_result;
}


//============================================================================//
//========================== Request sending data ============================//
//============================================================================//
ESP8266_Result ESP8266_Init_Sending_Data (ESP8266_Str* ESP8266, char* content) {
    char length = strlen(content) + '0';
    Transmit_UART(ESP8266_USART, (uint8_t*)"AT+CIPSEND=", strlen("AT+CIPSEND="));
    Transmit_UART(ESP8266_USART, (uint8_t*)&length, 1);
    Transmit_UART(ESP8266_USART, (uint8_t*)"\r\n", 2);

    if (Send_Command(ESP8266, ESP8266_COMMAND_SEND_DATA, NULL, NULL) != ESP8266_OK) {
        return ESP8266->last_result;
    }

    ESP8266->Flags.wait_for_wrapper = 1;
    return ESP8266->last_result;
}

//============================================================================//
//==================== Enable single/multiple connection =====================//
//==================== Command single:   AT+CIPMUX = 0   =====================//
//==================== Command multiple: AT+CIPMUX = 1   =====================//
//============================================================================//
ESP8266_Result ESP8266_Multi_Connection(ESP8266_Str* ESP8266, uint8_t mux) {
    char ch_mux = (char)(mux + '0');
   uint32_t string_length, count;
    int16_t found_wrapper;
    char char_received[128];
    char dummy[2];

    ESP8266_CHECK_IDLE(ESP8266);
    Transmit_UART(ESP8266_USART, (uint8_t*)"AT+CIPMUX=", strlen("AT+CIPMUX="));
    Transmit_UART(ESP8266_USART, (uint8_t*)&ch_mux, 1);   
    Transmit_UART(ESP8266_USART, (uint8_t*)"\r\n", 2);
  
    if (Send_Command(ESP8266, ESP8266_COMMAND_CIPMUX, NULL, NULL) != ESP8266_OK) {
        return ESP8266->last_result;
    }

    ESP8266_WAIT_READY_MACRO(ESP8266)
    if (!ESP8266->Flags.last_operation_status) {
        ESP8266_RETURN_STATUS(ESP8266, ESP8266_ERROR);
    }
     ESP8266_RETURN_STATUS(ESP8266, ESP8266_OK);
}

//====================================================================//
//=============== Function Show Remort IP and Port  ==================//
//====================================================================//
ESP8266_Result ESP8266_Remote_IP_Port (ESP8266_Str* ESP8266, uint8_t enable) {
   uint32_t string_length, count;
    int16_t found_wrapper;
    char char_received[128];
    char dummy[2];
    char ch_en = (char)(enable + '0');
    
    ESP8266_CHECK_IDLE(ESP8266);
    Transmit_UART(ESP8266_USART, (uint8_t*)"AT+CIPDINFO=", strlen("AT+CIPDINFO="));
    Transmit_UART(ESP8266_USART, (uint8_t*)&ch_en, 1);
    Transmit_UART(ESP8266_USART, (uint8_t*)"\r\n", 2);

    if (Send_Command(ESP8266, ESP8266_COMMAND_CIPDINFO, NULL, NULL) != ESP8266_OK) {
        return ESP8266->last_result;
    }

    ESP8266_WAIT_READY_MACRO(ESP8266)
    if (!ESP8266->Flags.last_operation_status) {
        ESP8266_RETURN_STATUS(ESP8266, ESP8266_ERROR);
    }
    ESP8266_RETURN_STATUS(ESP8266, ESP8266_OK); 
}

//====================================================================//
//=============== Function Enable Server ESP8266  ==================//
//====================================================================//
ESP8266_Result ESP8266_Create_Server (ESP8266_Str* ESP8266, uint16_t port) {
    uint32_t string_length, count;
    int16_t found_wrapper;
    char char_received[128];
    char dummy[2], port_ch[7];
  
    //ESP8266_CHECK_IDLE(ESP8266);
    sprintf(port_ch, "%d", port);

    Transmit_UART(ESP8266_USART, (uint8_t*)"AT+CIPSERVER=1,", strlen("AT+CIPSERVER=1,"));
    Transmit_UART(ESP8266_USART, (uint8_t*)port_ch, strlen(port_ch));
    Transmit_UART(ESP8266_USART, (uint8_t*)"\r\n", 2);

    if (Send_Command(ESP8266, ESP8266_COMMAND_CIPSERVER, NULL, NULL) != ESP8266_OK) {
        return ESP8266->last_result;
    }
    ESP8266_WAIT_READY_MACRO(ESP8266)
    if (!ESP8266->Flags.last_operation_status) {
        ESP8266_RETURN_STATUS(ESP8266, ESP8266_ERROR);
    }
    ESP8266_RETURN_STATUS(ESP8266, ESP8266_OK); 
}

//====================================================================//
//=============== Function Disable Server ESP8266  ==================//
//====================================================================//
ESP8266_Result ESP8266_Delete_Server (ESP8266_Str* ESP8266) {
    uint32_t string_length, count;
    int16_t found_wrapper;
    char char_received[128];
    char dummy[2];

    if (Send_Command(ESP8266, ESP8266_COMMAND_CIPSERVER, "AT+CIPSERVER=0\r\n", NULL) != ESP8266_OK) {
        return ESP8266->last_result;
    }
    ESP8266_WAIT_READY_MACRO(ESP8266)
    if (!ESP8266->Flags.last_operation_status) {
        ESP8266_RETURN_STATUS(ESP8266, ESP8266_ERROR);
    }
    ESP8266_RETURN_STATUS(ESP8266, ESP8266_OK); 
}

//====================================================================//
//=================== Function Connect to Wifi  =====================//
//====================================================================//
ESP8266_Result ESP8266_Connect_Wifi (ESP8266_Str* ESP8266, char* ssid, char* password) {
    uint32_t string_length, count;
    int16_t found_wrapper;
    char char_received[128];
    char dummy[2];

    Transmit_UART(ESP8266_USART, (uint8_t*)"AT+CWJAP_CUR=\"", strlen("AT+CWJAP_CUR=\""));
    Transmit_UART(ESP8266_USART, (uint8_t*)ssid, strlen(ssid));
    Transmit_UART(ESP8266_USART, (uint8_t*)"\",\"", strlen("\",\""));
    Transmit_UART(ESP8266_USART, (uint8_t*)password, strlen(password));
    Transmit_UART(ESP8266_USART, (uint8_t*)"\"\r\n", strlen("\"\r\n"));  
    if (Send_Command(ESP8266, ESP8266_COMMAND_CWJAP_SET, NULL, NULL) != ESP8266_OK) {
        return ESP8266->last_result;
    }
    ESP8266_WAIT_READY_MACRO(ESP8266)
    if (!ESP8266->Flags.last_operation_status) {
        ESP8266_RETURN_STATUS(ESP8266, ESP8266_ERROR);
    }
    ESP8266_RETURN_STATUS(ESP8266, ESP8266_OK); 
}

//====================================================================//
//================== Function Disconnect to Wifi  ====================//
//====================================================================//
ESP8266_Result ESP8266_Disconnect_Wifi (ESP8266_Str* ESP8266) {
     return Send_Command(ESP8266, ESP8266_COMMAND_CWQAP, "AT+CWQAP\r\n", "AT+CWQAP");
}

//====================================================================//
//================= Function Send datalength to Wifi  ================//
//====================================================================//
ESP8266_Result ESP8266_Length_TCP_Buffer(ESP8266_Str* ESP8266, uint8_t connection_num, uint32_t data_length) {
    char command[30];

    //sprintf(command, "AT+CIPSENDBUF=%d,%d\r\n", connection_num, data_length);
    sprintf(command, "AT+CIPSEND=%d,%d\r\n", connection_num, data_length);
    //if (Send_Command(ESP8266, ESP8266_COMMAND_CIPSENDBUF, command, NULL) != ESP8266_OK) {
    //    return ESP8266->last_result;
    //}
    Transmit_string_UART(ESP8266_USART, command);
    ESP8266->current_command = ESP8266_COMMAND_SEND_DATA;
    //ESP8266_WAIT_READY_MACRO(ESP8266)
    Delay_ESP8266(20);
    Stop_Track_TimeOut();
    ESP8266_RETURN_STATUS(ESP8266, ESP8266_OK); 
}

//====================================================================//
//============== Function close TCP/UPD/SSL connection  ==============//
//====================================================================//
ESP8266_Result ESP8266_Close_Connection(ESP8266_Str* ESP8266, uint8_t connection_num) {
    char command[30];

    sprintf(command, "AT+CIPCLOSE=%d\r\n", connection_num);
    if (Send_Command(ESP8266, ESP8266_COMMAND_CIPCLOSE, command, NULL) != ESP8266_OK) {
        return ESP8266->last_result;
    }

    //ESP8266_WAIT_READY_MACRO(ESP8266)
    Delay_ESP8266(20);
    if (!ESP8266->Flags.last_operation_status) {
        ESP8266_RETURN_STATUS(ESP8266, ESP8266_ERROR);
    }
    Stop_Track_TimeOut();
    ESP8266->current_command = ESP8266_COMMAND_IDLE;
    ESP8266_RETURN_STATUS(ESP8266, ESP8266_OK); 
}

//===================================================================//
//============= Function waiting for request from cilent ============//
//===================================================================//
ESP8266_Result ESP8266_Server_Waiting_For_Request(ESP8266_Str* ESP8266) {
    uint32_t string_length, count;
    char char_received[128];

    count = sizeof(char_received)/sizeof(char);                  
    string_length = Buffer_Read_String(&USART_buffer, char_received, count);   
    while (!ESP8266->IPD.in_IPD_mode) {
        if (string_length > 0) {
            Parse_IPD_Data_Received(ESP8266, char_received, count); 
        }                                              
        //ParseReceived(ESP8266, char_received, 1, string_length);                 
        string_length = Buffer_Read_String(&USART_buffer, char_received, count); 
    }
    ESP8266_RETURN_STATUS(ESP8266, ESP8266_OK);
} 

//===================================================================//
//================= Function send data to Web Browser ===============//
//===================================================================//
ESP8266_Result ESP8266_Server_Send_Data(ESP8266_Str* ESP8266) {
    ///uint32_t string_length, count;
    ///int16_t found_wrapper;
    ///char char_received[128], ch;
    ///char dummy[2];
    char ch;
    Buffer_t* buff;

    if (!ESP8266->IPD.in_IPD_mode) {
        //Transmit_UART(USART6, (uint8_t*)"Error! ESP8266 is not in IPD mode\n", strlen("Error! ESP8266 is not in IPD mode\n"));
        return ESP8266_ERROR;
    }
    buff = &USART_buffer;
    while ((ESP8266->IPD.pointer_dat < ESP8266->connection[ESP8266->IPD.connection_num].byte_received) 
            && (Buffer_Read_Free(buff))) {
        Buffer_Read(buff, 1, (uint8_t*)&ch);
        ESP8266->connection[ESP8266->IPD.connection_num].data[ESP8266->IPD.pointer_dat] = ch;

        ESP8266->IPD.pointer_dat++;
        ESP8266->IPD.pointer_total++;           

        //if ((ESP8266->IPD.pointer_dat >= ESP8266_CONNECTION_BUFFER_SIZE) && (ESP8266->IPD.pointer_dat != ESP8266->connection[ESP8266->IPD.connection_num].byte_received)) {
        //    ESP8266->connection[ESP8266->IPD.connection_num].data_size = ESP8266->IPD.pointer_dat;
        //    ESP8266->connection[ESP8266->IPD.connection_num].last_part = 0;
        //}
        //
        //if (ESP8266->connection[ESP8266->IPD.connection_num].cilent) {
        //    ESP8266_CallBack_Client_ConnectionData_Received(ESP8266, &ESP8266->connection[ESP8266->IPD.connection_num], ESP8266->connection[ESP8266->IPD.connection_num].data);
        //}
        //else {
        //    ESP8266_CallBack_Server_ConnectionData_Received(ESP8266, &ESP8266->connection[ESP8266->IPD.connection_num], ESP8266->connection[ESP8266->IPD.connection_num].data);
        //}                                 
    }

    if (ESP8266->IPD.pointer_total >= ESP8266->connection[ESP8266->IPD.connection_num].byte_received) {
        char* ptr;
        ESP8266->IPD.in_IPD_mode = 0;
        ESP8266->connection[ESP8266->IPD.connection_num].data_size = ESP8266->IPD.pointer_dat;
        ESP8266->connection[ESP8266->IPD.connection_num].last_part = 1;
        
        if ((ESP8266->connection[ESP8266->IPD.connection_num].first_packet) &&
             (ptr = strstr(ESP8266->connection[ESP8266->IPD.connection_num].data, "Content-Length: ")) != NULL) {
            ptr+= 16;
            ESP8266->connection[ESP8266->IPD.connection_num].content_length = ParseNumber(ptr, NULL);
        }
        ESP8266->connection[ESP8266->IPD.connection_num].call_data_received = 1;
    }
    ESP8266_Call_Connection_CallBack(ESP8266);
    ESP8266_RETURN_STATUS(ESP8266, ESP8266_OK);
}

//******************************************************************//
//******** Start timer and reset variable to track timeout ********//
//*****************************************************************//
void Start_Track_TimeOut(void) {
    TIMx_Reset_CNT(TIM3);
    track_count = 0;
    start_track = 1;
}

//******************************************************************//
//******** Diable timer and reset variable to track timeout ********//
//*****************************************************************//
void Stop_Track_TimeOut(void) {
    track_count = 0;
    start_track = 0;
}

//******************************************************************//
//******** Handle request from browser ********//
//*****************************************************************//
void Handle_Request_Browser(ESP8266_Str* ESP8266) {
    uint32_t string_length, count;
    char char_received[128];

    count = sizeof(char_received)/sizeof(char);                  
    string_length = Buffer_Read_String(&USART_buffer, char_received, count);  
    while (1) {
        if (string_length > 0) {
            Parse_IPD_Data_Received(ESP8266, char_received, count); 
        }      
        else if (ESP8266->IPD.in_IPD_mode) {
            for (int i = 0; i < ESP8266_MAX_CONNECTION; i++) {
                if ((ESP8266->connection[i].active) && (ESP8266->connection[i].call_data_received)) {
                    ESP8266_CallBack_Server_ConnectionData_Received(ESP8266, ESP8266->connection[i].command, i);
                }
                ESP8266->connection[i].call_data_received = 0;
            }
            ESP8266->IPD.in_IPD_mode = 0;
        }                                        
        //ParseReceived(ESP8266, char_received, 1, string_length);                 
        string_length = Buffer_Read_String(&USART_buffer, char_received, count);
        //Delay_ESP8266(30); 
    }
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


//void Web_Content_To_Browser(void) {
//}
