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

extern uint8_t UsartWifiBuf[];
extern uint32_t WifiRecvFlag;
extern BC_Mutex WifiRecvFlagMutex;

volatile void IrqUsartWifi(void);
sint32_t ParseIPD(uint8_t * buf, uint32_t buf_size, BC_SocketData * socket_data);
sint32_t ParseCIPSTATUS(uint8_t * buf, uint32_t buf_size, BC_SocketData * socket_data);
sint32_t ParseCIFSR(uint8_t * buf, uint32_t buf_size, uint8_t * addr_buf, uint32_t addr_buf_size);
sint32_t TryDispatch(uint32_t sockfd, uint32_t msg_type, uint8_t * msg, uint32_t msg_size);

#endif

