#include "ESP8266.h"
#include "LCD.h"

#define CHAR_TO_NUM(x)  ((x) - '0')
#define CHAR_IS_NUM(x)   ((x) >= '0') && ((x) <= '9')
#define ESP8266_CHECK_IDLE(ESP8266)                           \
    if ((ESP8266)->current_command != ESP8266_COMMAND_IDLE) { \
        ESP8266_Update(ESP8266);                              \
        ESP8266_RETURN_STATUS(ESP8266, ESP8266_BUSY);         \
     }

#define ESP8266_RETURN_STATUS(ESP8266, status) \
    ESP8266->last_result = status;             \
    return status;
     
static Buffer_t USART_buffer;
static uint8_t USART_data[USART_ESP8266_SIZE];

/******************************** BUFFER FUNCTION **************************
1. This is circular buffer, have a fixed-number size
2. Have two pointer, head and tail (in code is write_idx - head pointer
and read_idx - tail pointer.
3. When a byte is added to buffer, write_idx will increase 1
4. When a byte is read, read_idx will increase. */


//*************** Count number of bytes free *************
//***** Ham nay tra ve so byte con trong de co the ghi duoc
//***** Nguoc lai vs ham dem so byte con trong de doc  ****/

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

//*********** Count number of bytes free *********//
uint32_t Buffer_Read_Free(Buffer_t* buff) {
	  uint32_t in, out, num_byte_free;
	
	  in = buff->write_idx;
	  out = buff->read_idx;
	
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

/********************* Write data to buffer *********************/
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
		    buff->Data[buff->write_idx++] = *dat;
			  dat++;
			
			  if (buff->write_idx > buff-> size) {
				  buff->write_idx = 0;
				}
				num_write++;
		}
		return num_write;
}

/********************* Read data from buffer *********************/
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
			
		if (buff->read_idx > buff-> size) {
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

/***************** Read String Buffer *************/
uint32_t Buffer_Read_String(Buffer_t* buffer, char *save_buff, uint32_t count) { 
	uint32_t read_reserved, write_reserved, idx;
    uint8_t ch;

    if (buffer == NULL) {
	  return 0;
    }
		
	read_reserved = Buffer_Read_Free(buffer);
	write_reserved = Buffer_Write_Free(buffer);
    idx = 0;
		
	if ((!read_reserved) ||  
		((read_reserved < count) 
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
        save_buff[++idx] = 0; //???????
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
	 uint32_t read_reserved, out, idx;
	 uint8_t found;
	
	 read_reserved = Buffer_Read_Free(buffer);
	 if (read_reserved < num_find) {
	     return -1;
	 }
	 
	 out = buffer->read_idx;
	 while (read_reserved) {
	     if (string_find[0] == buffer->Data[out]) {
		      found = 1;
			    idx = 1;
		   }
		   read_reserved--;
		   out++;
		 
		   if (out > buffer->size) {
		       out = 0;
		   }
		 
		   if (found) {
			     while (idx < num_find) {
					     if (out >= buffer->size) {
						       out = 0;
						   }
				       if ((uint8_t)string_find[idx] != buffer->Data[out]) {
						       break;
						   }
						   out++;
						   read_reserved--;
				   }						 
				   if (idx == num_find) {
				       return 1;
				   }	     
		   }
	 }
	 return -1;
}
//***************************************************************************//


void Init_ESP_GPIO(void) {
	  //Enable Clock for PORT A
	  GPIO_InitTypeDef GPIO_UART_ESP;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
		//Enable System  Config Controller Clock
	  GPIO_UART_ESP.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_UART_ESP.GPIO_Speed = GPIO_Speed_100MHz;
	  GPIO_UART_ESP.GPIO_OType = GPIO_OType_PP;
	  //GPIO_UART_ESP.GPIO_PuPd  = GPIO_PuPd_DOWN;
	  GPIO_UART_ESP.GPIO_PuPd  = GPIO_PuPd_UP;
	  GPIO_UART_ESP.GPIO_Mode  = GPIO_Mode_AF; 
	  GPIO_Init(GPIOA, &GPIO_UART_ESP);
	  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
	  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
	
}

void Init_USART1_RXNE_Interrupt(USART_TypeDef* USARTx) {
	NVIC_InitTypeDef NVIC_Init_Struct;
	
  NVIC_Init_Struct.NVIC_IRQChannel                   = USART1_IRQn;
	NVIC_Init_Struct.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_Init_Struct.NVIC_IRQChannelSubPriority        = 0x1;
	NVIC_Init_Struct.NVIC_IRQChannelCmd                = ENABLE;

	NVIC_Init(&NVIC_Init_Struct);
}

void USART1_IRQHandler(void) {
    if (USART_GetITStatus(USART1, USART_IT_RXNE)) {
	      while (!USART_GetFlagStatus(USART1, USART_FLAG_RXNE)) {
	      }
        uint8_t dat = (uint8_t)USART_ReceiveData(USART1);
        Buffer_Write(&USART_buffer, 1, &dat);
        //Clear interrupt flag
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}

void Init_UART_Config(void) {
	 uint16_t received_data[2];
	 RCC_APB2PeriphClockCmd (RCC_APB2Periph_USART1, ENABLE);
	 USART_InitTypeDef Init_UART_Str;
	
	 USART_StructInit(&Init_UART_Str);
	 Init_UART_Str.USART_BaudRate = 115200;
	 USART_Init(USART1, &Init_UART_Str);
	 USART1->CR1 |= 0x20;//Enable Interrupt UART
	 USART_Cmd(USART1, ENABLE);
	 //Transmit_UART_Enable(USART1, ENABLE);
	 //while(1) {
	    //Transmit_UART(USART1, "A");
	 //}
	 //while (1) {	
	     /*****  Code received data *******/
	     USART_SendData (USART1, 0x41);
		   while (!USART_GetFlagStatus(USART1, USART_FLAG_RXNE)) {
	     }
			 received_data[0] = USART_ReceiveData(USART1);
			 //while (!USART_GetFlagStatus(USART1, USART_FLAG_RXNE)) {
	     //}
			 // received_data[1] = USART_ReceiveData(USART1);
			 GPIOD->ODR = received_data[0];
			 //GPIO_SetBits(GPIOD, GPIO_Pin_12);
			 printf("%d\n", received_data[0]);
			 //printf("%d\n", received_data[1]);
	     //GPIO_SetBits(GPIOC, GPIO_Pin_13);
	
	     /********************************/
			 
			 /* Code send data 
			 while (!USART_GetFlagStatus(USART1, USART_FLAG_TXE)) {
	     }
			 USART_SendData (USART1, 0x41);
			 while (!USART_GetFlagStatus(USART1, USART_FLAG_TXE)) {
	     }
			 */
	 //}
   
	 USART_Cmd(USART1, DISABLE);
}

void Transmit_UART(USART_TypeDef* USARTx, uint8_t* dat, uint16_t count) {
	while (count) {
	  while (!USART_GetFlagStatus(USARTx, USART_FLAG_TXE)) {
	  }
	  USART_SendData (USARTx, (uint16_t)*dat);
	  dat++;
  }
	while (!USART_GetFlagStatus(USART1, USART_FLAG_TC)) {
	}
}

char UART_GetChar (USART_TypeDef* USARTx) {
	while (!USART_GetFlagStatus(USART1, USART_FLAG_RXNE)) {
	}
	return (char)USART_ReceiveData(USART1);
}

char* Receive_UART(USART_TypeDef* USARTx, int num_char_receive) {
	  //ch;
	  while (num_char_receive > 0) {
		}
			
}

/************************ FUNCTION FOR ESP8266 *******************/
//---------------------- FUNCTION ANALYZE STRING -------------------//
void ParseCWJAP(ESP8266_Str* ESP8266, char* received) {

}
//------------------------------------------------------------------//
ESP8266_Result ESP8266_Init(ESP8266_Str* ESP8266_dt) {
    if(Buffer_Init(&USART_buffer, USART_ESP8266_SIZE, USART_data)) {
        return ESP8266_MALLOC_ERR;
    }
    Send_Command(ESP8266_dt, ESP8266_COMMAND_RST, "AT+RST\r\n", "ready\r\n");
)

    
}

ESP8266_Result  ESP8266_Check_Idle(ESP8266_Str* ESP8266) {
    if (ESP8266->current_command != ESP8266_COMMAND_IDLE) {
    }
}

//--------------------- Send Command to ESP8266 -----------------//
//---------------------------------------------------------------//
ESP8266_Result Send_Command(ESP8266_Str* ESP8266, uint8_t command, char* command_str, char* start_respond) {
    ESP8266_CHECK_IDLE(ESP8266);   
    if (command_str != NULL) {
        Transmit_UART(USART1, (uint8_t*)command_str, strlen(command_str));
         //Send String UART
    }
    ESP8266->current_command = command;
    ESP8266_RETURN_STATUS(ESP8266, ESP8266_OK);  
}

//--------------------- Wait for respond after send command ----------------//
//--------------------------------------------------------------------------//
ESP8266_Result ESP8266_WaitReady(ESP8266_Str* ESP8266) {
    do {
        if() {
            if() {
                break;
            }
        }
        ESP8266_Update(ESP8266);
    } while(ESP8266->current_command != ESP8266_COMMAND_IDLE);
     ESP8266_RETURN_STATUS(ESP8266, ESP8266_OK);        
}

ESP8266_Result ESP8266_Update(ESP8266_Str* ESP8266) {
    uint32_t string_length, count;
    char char_received[20];
    if(ESP8266->timeout == 0) {
        ESP8266->timeout = 30000;
    }

    if ((ESP8266->time - ESP8266->start_time) > ESP8266->timeout) {
        ESP8266->current_command = ESP8266_COMMAND_IDLE;
    }
  //--------------- Waiting for sending data -----------//
  //----------------------------------------------------//
    
    count = sizeof(char_received)/sizeof(char);
    string_length = Buffer_Read_String(&USART_buffer, char_received, sizeof(char_received)/sizeof(char));
    while (string_length > 0) {
        ParseReceived(ESP8266, char_received, 1, string_length);
        string_length = Buffer_Read_String(&USART_buffer, char_received, sizeof(char_received)/sizeof(char));
    }
   
}

void ParseReceived(ESP8266_Str* ESP8266, char* received, uint8_t from_usart_buffer, uint16_t bufflen) {
    if ((bufflen == 2) && (received[0] == '\r') && (received[1] == '\n')) {
        return;
    }

    if (!strcmp(received, "WIFI CONNECTED\r\n")) {
        ESP8266->Flags.wifi_connected = 1;
    }
    else if (!strcmp(received, "WIFI DISCONNECT\r\n")) {
        ESP8266->Flags.wifi_connected =01;
    }
 
    switch (ESP8266->current_command) {
        case ESP8266_COMMAND_CWMODE:
            if (!strcmp(received, "OK\r\n")) {
                ESP8266->current_command = ESP8266_COMMAND_IDLE;
                ESP8266->mode = ESP8266->send_mode;
            }
            break;

        case ESP8266_COMMAND_CWJAP_SET:
            if (!strncmp(received, "+CWJAP_CUR:", 11)) {
                strcpy(ESP8266->command_response, "FAIL\r\n");
                ESP8266->Wifi_connect_error = (ESP8266_Wifi_connect_error_t)CHAR_TO_NUM(received[1]);
            }
            if (!strcmp(received, "OK\r\n")) {
                ESP8266->current_command = ESP8266_COMMAND_IDLE;
            }
            if (!strcmp(received, "FAIL\r\n")) {
                ESP8266->current_command = ESP8266_COMMAND_IDLE;
                //ESP8266_Callback_WifiConnectFailed(ESP8266);
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
                 ParseMAC();
            }       
            if (!strcmp(received, "OK\r\n")) {
                 ESP8266->current_command = ESP8266_COMMAND_IDLE;
            }
            break;
        case ESP8266_COMMAND_CIPAPMAC:
            if (!strncmp(received, "+CIPAPMAC", 9)) {
                ParseMAC();
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
                //ESP8266_Callback_WifiIPSet(ESP8266);
            }
                
                
    }

    ESP8266->last_received_time = ESP8266->time;
    if (strcmp(received, "OK\n\n")) {
        ESP8266->Flags.last_operation_status = 1;
    }
    if ((!strcmp(received, "ERROR\r\n")) || strcmp(received, "busy p...\r\n")) {
        ESP8266->Flags.last_operation_status = 1;
        ESP8266->current_command = 0;
    }

    if (!strcmp(received, "SEND OK\r\n")) {
        ESP8266->current_command = ESP8266_COMMAND_IDLE;
        ESP8266->Flags.wait_for_wrapper = 0;
    }
}
//------------------ Set Mode Wifi *-------------//      
//-----------------------------------------------//
void Set_Wifi_Mode(Wifi_Mode mode) {
	assert_param(IS_WIFI_MODE(mode));
	char sbuf[20];
	sprintf(sbuf, "AT+CWMODE_CUR=%d", mode);

}

//*************** Connect to Access Point *************//
//******************************************************

void Connect_To_AP(char* ssid, char* password) {
	 char sbuf[30];
	 sprintf(sbuf, "AT+CWJAP_CUR=%s,%s", ssid, password);
}

/********************************************************************/