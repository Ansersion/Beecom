//   Copyright 2017 Ansersion
//
//   Licensed under the Apache License, Version 2.0 (the "License");
//   you may not use this file except in compliance with the License.
//   You may obtain a copy of the License at
//
//       http://www.apache.org/licenses/LICENSE-2.0
//
//   Unless required by applicable law or agreed to in writing, software
//   distributed under the License is distributed on an "AS IS" BASIS,
//   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//   See the License for the specific language governing permissions and
//   limitations under the License.
//

#ifndef WIFI_IRQ_H
#define WIFI_IRQ_H

#include <irq.h>
#include <wifi_common.h>
#include <mutex.h>

#define USART_WIFI 		USART2
#define USART_WIFI_BUF_SIZE 	128

#define WIFI_MSG_FLAG_GENERAL_OK 			0x00000001
#define WIFI_MSG_FLAG_GENERAL_ERR 			0x00000002
#define WIFI_MSG_FLAG_GOT_CONNECT 			0x00000004
#define WIFI_MSG_FLAG_GOT_CLOSED 			0x00000008

#define WIFI_MSG_FLAG_GOT_CLI 				0x00000010
#define WIFI_MSG_FLAG_GOT_IPD 				0x00000020

#define WIFI_MSG_FLAG_SENDING 				0x00000100

#define WIFI_MSG_MASK_ERR 					0xFF000000
#define WIFI_MSG_FLAG_INVALID_SOCK_SERV 	0x10000000
#define WIFI_MSG_FLAG_SERV_QUE_OVERFLOW 	0x20000000
#define WIFI_MSG_FLAG_BUF_OVERFLOW 			0x40000000
#define WIFI_MSG_FLAG_BUF_RESET 			0x80000000


enum CIPSTATUS_PARSE_STATE {
	CIPSTATUS_PARSE_NONE = 0,
	CIPSTATUS_PARSE_CHAR_PLUS,
	CIPSTATUS_PARSE_CHAR_COLON,
	CIPSTATUS_PARSE_CID,
	CIPSTATUS_PARSE_TCP_UDP,
	CIPSTATUS_PARSE_CIP_QUOTE_1,
	CIPSTATUS_PARSE_CIP_QUOTE_2,
	CIPSTATUS_PARSE_CIP,
	CIPSTATUS_PARSE_CPORT,
	CIPSTATUS_PARSE_SPORT,
	CIPSTATUS_PARSE_CSTATE,
};

enum IPD_STATE {
	IPD_PARSE_NONE = 0,
	IPD_START_PARSE,
	IPD_PARSE_ID,
	IPD_PARSE_COLON,
	IPD_PARSE_LEN,
	IPD_PARSE_CONTENT,
};

enum CIFSR_PARSE_STATE {
	CIFSR_PARSE_NONE = 0,
	CIFSR_PARSE_FIRST_COMMA,
	CIFSR_PARSE_IP_QUOTE_1,
	CIFSR_PARSE_IP_QUOTE_2,
	CIFSR_PARSE_IP,
	CIFSR_PARSE_SECOND_COMMA,
	CIFSR_PARSE_MAC_QUOTE_1,
	CIFSR_PARSE_MAC_QUOTE_2,
	CIFSR_PARSE_MAC,
};

enum CIPCLOSE_PARSE_STATE {
	CIPCLOSE_PARSE_NONE = 0,
	CIPCLOSE_PARSE_CHAR_EQUAL,
	CIPCLOSE_PARSE_SOCKET_ID,
	CIPCLOSE_PARSE_CHAR_D,
	CIPCLOSE_PARSE_RESULT,
};

enum CIPSEND_PARSE_STATE {
	CIPSEND_PARSE_NONE = 0,
	CIPSEND_PARSE_EQUAL_CHAR,
	CIPSEND_PARSE_WIFI_ID,
	CIPSEND_PARSE_RESULT_1,
	CIPSEND_PARSE_BRACKET,
	CIPSEND_PARSE_CHAR_s,
	CIPSEND_PARSE_STR_SEND,
	CIPSEND_PARSE_RESULT_2,
};

extern uint8_t UsartWifiBuf[];
extern uint32_t WifiRecvFlag;
extern BC_Mutex WifiRecvFlagMutex;

volatile void IrqUsartWifi(void);
sint32_t ParseIPD(uint8_t * buf, uint32_t buf_size, BC_SocketData * socket_data);
sint32_t ParseCIPSTATUS(uint8_t * buf, uint32_t buf_size, BC_SocketData * socket_data);
sint32_t ParseCIFSR(uint8_t * buf, uint32_t buf_size, uint8_t * addr_buf, uint32_t addr_buf_size);
sint32_t ParseCIPCLOSE(uint8_t * buf, uint32_t buf_size, BC_SocketData * socket_data);
sint32_t ParseCIPSEND(uint8_t * buf, uint32_t buf_size, BC_SocketData * socket_data);
sint32_t TryDispatch(uint32_t sockfd, uint32_t msg_type, uint8_t * msg, uint32_t msg_size);

#endif

