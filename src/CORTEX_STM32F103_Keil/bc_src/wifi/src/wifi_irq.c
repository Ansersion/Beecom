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

// STD headers
#include <string.h>
#include <ctype.h>
#include <stdlib.h>

// STM32 headers
#include <stm32f10x_usart.h>

// Beecom headers
#include <utils.h>
#include <bc_type.h>
#include <wifi_irq.h>
#include <mutex.h>
#include <wifi_common.h>

// test
#include <terminal.h>

// extern BC_SocketData sock_data[];
// extern BC_SocketData sock_serv;

uint8_t UsartWifiBuf[USART_WIFI_BUF_SIZE];
uint32_t WifiRecvFlag = 0;
BC_Mutex WifiRecvFlagMutex;

uint8_t WIFI_FLAG_OK_END[] = "OK\r\n";
uint32_t WIFI_FLAG_OK_END_SIZE = sizeof(WIFI_FLAG_OK_END) - 1;
uint8_t WIFI_FLAG_ERROR_END[] = "ERROR\r\n";
uint32_t WIFI_FLAG_ERROR_END_SIZE = sizeof(WIFI_FLAG_ERROR_END) - 1;
uint8_t WIFI_FLAG_CONN_END[] = "CONNECT\r\n";
uint32_t WIFI_FLAG_CONN_END_SIZE = sizeof(WIFI_FLAG_CONN_END) - 1;
uint8_t WIFI_FLAG_CLOSED_END[] = "CLOSED\r\n";
uint32_t WIFI_FLAG_CLOSED_END_SIZE = sizeof(WIFI_FLAG_CLOSED_END) - 1;

uint8_t WIFI_FLAG_STATUS_ST[] = "AT+CIPSTATUS";
uint32_t WIFI_FLAG_STATUS_ST_SIZE = sizeof(WIFI_FLAG_STATUS_ST) - 1;
uint8_t WIFI_FLAG_IPD_ST[] = "+IPD";
uint32_t WIFI_FLAG_IPD_ST_SIZE = sizeof(WIFI_FLAG_IPD_ST) - 1;
uint8_t WIFI_FLAG_SR_ST[] = "AT+CIFSR";
uint32_t WIFI_FLAG_SR_ST_SIZE = sizeof(WIFI_FLAG_SR_ST) - 1;
uint8_t WIFI_FLAG_CLOSED_ST[] = "AT+CIPCLOSE";
uint32_t WIFI_FLAG_CLOSED_ST_SIZE = sizeof(WIFI_FLAG_CLOSED_ST) - 1;
uint8_t WIFI_FLAG_SEND_ST[] = "AT+CIPSEND";
uint32_t WIFI_FLAG_SEND_ST_SIZE = sizeof(WIFI_FLAG_SEND_ST) - 1;
// static char x[4];

volatile void IrqUsartWifi(void)
{
	static uint32_t Index = 0;
	static uint16_t RxData=0;
	static sint32_t u32CurrentSockId = -1;
	static sint32_t u32SendSockId = -1;
	static sint32_t s8Ret;
	static BC_SocketData * SockServ = NULL;

	SockServ = GetSockDataIrq(SOCK_SERV_FD);
	if(!SockServ) {
		WifiRecvFlag |= WIFI_MSG_FLAG_INVALID_SOCK_SERV;
		return;
	}
	// x[3] = '\0';
	// x[2] = '\n';
	// x[1] = '\r';
	// static uint32_t u32SendSockId
	// BC_SocketData sock_data_tmp;

	if(USART_GetITStatus(USART_WIFI, USART_IT_RXNE) == RESET) {
		return;
	}
	RxData = USART_RECEIVE(USART_WIFI, RxData);

	UsartWifiBuf[Index++] = (uint8_t)RxData;
	if('\r' == UsartWifiBuf[0] || '\n' == UsartWifiBuf[0]) {
		if(!(WifiRecvFlag & WIFI_MSG_FLAG_SENDING)) {
		 Index = 0;
		}
		return;
	}
	if(Index >= USART_WIFI_BUF_SIZE - 1) {
		WifiRecvFlag |= WIFI_MSG_FLAG_BUF_OVERFLOW;
		UsartWifiBuf[USART_WIFI_BUF_SIZE-1] = '\0';
		Index = 0;
		return;
	}
	if(BC_OK == CheckDataFlag(UsartWifiBuf, Index, WIFI_FLAG_OK_END, WIFI_FLAG_OK_END_SIZE, TRUE)) {
		if(!(WifiRecvFlag & WIFI_MSG_FLAG_SENDING)) {
			WifiRecvFlag |= WIFI_MSG_FLAG_GENERAL_OK;
			UsartWifiBuf[Index] = '\0';
			Index = 0;
		}
		return;
	}
	if(BC_OK == CheckDataFlag(UsartWifiBuf, Index, WIFI_FLAG_ERROR_END, WIFI_FLAG_ERROR_END_SIZE, TRUE)) {
		if(!(WifiRecvFlag & WIFI_MSG_FLAG_SENDING)) {
			WifiRecvFlag |= WIFI_MSG_FLAG_GENERAL_ERR;
			UsartWifiBuf[Index] = '\0';
			Index = 0;
		}
		return;
	}

	// client connects to(close from) local server
	if(Index > 0) {
		if(isdigit(UsartWifiBuf[0])) {
			u32CurrentSockId = BC_Atoi(UsartWifiBuf[0]);
			if(BC_OK == TryDispatch(u32CurrentSockId, WIFI_MSG_FLAG_GOT_CONNECT, UsartWifiBuf, Index)) {
				Index = 0;
				return;
			} else if(BC_OK == TryDispatch(u32CurrentSockId, WIFI_MSG_FLAG_GOT_CLOSED, UsartWifiBuf, Index)) {
				Index = 0;
				return;
			}
		}
	}

	if(BC_OK == CheckDataFlag(UsartWifiBuf, Index, WIFI_FLAG_STATUS_ST, WIFI_FLAG_STATUS_ST_SIZE, FALSE)) {
		if(ParseCIPSTATUS(UsartWifiBuf+WIFI_FLAG_STATUS_ST_SIZE, Index-WIFI_FLAG_STATUS_ST_SIZE, &sock_data[SockServ->wifi_id]) == 0) {
			SockServ->wifi_recv_flag &= ~WIFI_MSG_FLAG_GOT_CONNECT;
			if(pdTRUE != xQueueSendFromISR(sock_data[SockServ->wifi_id].queue_handle, &sock_data[SockServ->wifi_id], NULL)) {
				// sock_data[
			}
			Index = 0;
			return;
		}
	}
	if(BC_OK == CheckDataFlag(UsartWifiBuf, Index, WIFI_FLAG_SR_ST, WIFI_FLAG_SR_ST_SIZE, FALSE)) {
		if(ParseCIFSR(UsartWifiBuf+WIFI_FLAG_SR_ST_SIZE, Index-WIFI_FLAG_SR_ST_SIZE, INADDR_ANY, 16) == 0) {
			if(pdTRUE != xQueueSendFromISR(SockServ->queue_handle, SockServ, NULL)) {
				// sock_data[


			}
			Index = 0;
			return;
		}
	}
	if(BC_OK == CheckDataFlag(UsartWifiBuf, Index, WIFI_FLAG_CLOSED_ST, WIFI_FLAG_CLOSED_ST_SIZE, FALSE)) {
		// x[0] = 'x';
		// uputn(USART_TERMINAL, x, 3);
		if(ParseCIPCLOSE(UsartWifiBuf+WIFI_FLAG_CLOSED_ST_SIZE, Index-WIFI_FLAG_CLOSED_ST_SIZE, SockServ) == 0) {
			// x[0] = 'z';
			// uputn(USART_TERMINAL, x, 3);
			sock_data[SockServ->wifi_id].wifi_recv_flag &= ~WIFI_MSG_FLAG_GOT_CONNECT;
			sock_data[SockServ->wifi_id].wifi_recv_flag |= WIFI_MSG_FLAG_GOT_CLOSED;
			// sock_data[0].wifi_recv_flag &= ~WIFI_MSG_FLAG_GOT_CONNECT;
			// sock_data[0].wifi_recv_flag |= WIFI_MSG_FLAG_GOT_CLOSED;
			if(pdTRUE != xQueueSendFromISR(sock_data[SockServ->wifi_id].queue_handle, &sock_data[SockServ->wifi_id], NULL)) {
			}
			// if(pdTRUE != xQueueSendFromISR(sock_data[0].queue_handle, &sock_data[0], NULL)) {
			// }
			// x[0] = 'z';
			// uputn(USART_TERMINAL, x, 3);
			WifiRecvFlag |= WIFI_MSG_FLAG_GENERAL_OK;
			// TODO: To check if overflow
			UsartWifiBuf[Index] = '\0';
			Index = 0;
			return;
		}
	}
	if((WifiRecvFlag & WIFI_MSG_FLAG_SENDING) || (BC_OK == CheckDataFlag(UsartWifiBuf, Index, WIFI_FLAG_SEND_ST, WIFI_FLAG_SEND_ST_SIZE, FALSE))) {
		WifiRecvFlag |= WIFI_MSG_FLAG_SENDING;
		s8Ret = ParseCIPSEND(UsartWifiBuf, Index, SockServ);
		if(0 == s8Ret) {
			if(pdTRUE != xQueueSendFromISR(sock_data[SockServ->wifi_id].queue_handle, &sock_data[SockServ->wifi_id], NULL)) {
			}
			// if(pdTRUE != xQueueSendFromISR(SockServ.queue_handle, &SockServ, NULL)) {
			// }
			WifiRecvFlag &= ~WIFI_MSG_FLAG_SENDING;
			WifiRecvFlag |= WIFI_MSG_FLAG_GENERAL_OK;
			// TODO: To check if overflow
			UsartWifiBuf[Index] = '\0';
			Index = 0;
			return;
		} else if(0 > s8Ret) {
			WifiRecvFlag &= ~WIFI_MSG_FLAG_SENDING;
			WifiRecvFlag |= WIFI_MSG_FLAG_GENERAL_ERR;
			// TODO: To check if overflow
			UsartWifiBuf[Index] = '\0';
			Index = 0;
			return;
		}
		// TODO: check forever sending
	}
	if(BC_OK == CheckDataFlag(UsartWifiBuf, Index, WIFI_FLAG_IPD_ST, WIFI_FLAG_IPD_ST_SIZE, FALSE) ) {
		if(ParseIPD(UsartWifiBuf+WIFI_FLAG_IPD_ST_SIZE, Index-WIFI_FLAG_IPD_ST_SIZE, SockServ) == 0) {
			memcpy(sock_data[SockServ->wifi_id].buf, SockServ->buf, SockServ->ipd_size);
			if(pdTRUE != xQueueSendFromISR(sock_data[SockServ->wifi_id].queue_handle, SockServ, NULL)) {
				// sprintf(msg_irs, "IPD QUE Err\r\n");
			} else {
				// sprintf(msg_irs, "IPD QUE OK\r\n");
				// WifiRecvFlag |= WIFI_MSG_FLAG_GOT_IPD;
			}
			Index = 0;
			return;
		}
	}
}

sint32_t ParseIPD(uint8_t * buf, uint32_t buf_size, BC_SocketData * socket_data)
{
	static sint32_t IPDState = IPD_START_PARSE;
	static sint32_t i = 0, old_i = 0;
	static sint32_t result = 0; // 0->Parse Completed, 1->Parsing..., 2->Error

	static uint16_t num = 0;

	if(!buf) {
		// sprintf(msg_irs,"ret=-1\r\n");
		return -1;
	}
	if(!socket_data) {
		// sprintf(msg_irs,"ret=-2\r\n");
		return -2;
	}

	result = 1;

	// e.g:
	// +IPD,0,9:ansersion
	switch(IPDState) {
		case IPD_START_PARSE:
			// sprintf(msg_irs,"IPD-START\r\n");
			for(i = old_i; i < buf_size; i++) {
				if(',' == buf[i]) {
					IPDState = IPD_PARSE_ID;
					old_i = i+1;
					break;
				}
			}
			break;
		case IPD_PARSE_ID:
			// no use now
			// sprintf(msg_irs,"IPD-ID\r\n");
			for(i = old_i; i < buf_size; i++) {
				if(',' == buf[i]) {
					socket_data->wifi_id = atoi((const char *)(&buf[old_i]));
					IPDState = IPD_PARSE_LEN;
					old_i = i+1;
					break;
				}
			}
			break;
		case IPD_PARSE_LEN:
			// sprintf(msg_irs,"IPD-COLON\r\n");
			for(i = old_i; i < buf_size; i++) {
				if(':' == buf[i]) {
					num = atoi((const char *)(&buf[old_i]));
					socket_data->ipd_size = num;
					IPDState = IPD_PARSE_CONTENT;
					old_i = i+1;
					break;
				}
			}
			break;
		case IPD_PARSE_CONTENT:
			// sprintf(msg_irs,"IPD-CONTENT, %d>%d\r\n", buf_size, old_i+num);
			if(buf_size >= old_i + num) {
				memcpy(socket_data->buf, &buf[old_i], num);
				buf[old_i+num] = '\0';
				socket_data->buf[num] = '\0';
				num = 0;
				old_i = 0;
				IPDState = IPD_START_PARSE;
				result = 0;
			}
			break;
		default:
			IPDState = IPD_START_PARSE;
			num = 0;
			old_i = 0;
			result = 2;
			break;
	}

	return result;
}

sint32_t ParseCIPSTATUS(uint8_t * buf, uint32_t buf_size, BC_SocketData * socket_data)
{
	static sint32_t StatusState = CIPSTATUS_PARSE_CHAR_PLUS;
	static sint32_t i = 0, old_i = 0;
	static sint32_t result = 0; // 0->Parse Completed, 1->Parsing..., 2->Error

	if(!buf) {
		// sprintf(msg_irs,"ret=-1\r\n");
		return -1;
	}
	if(!socket_data) {
		// sprintf(msg_irs,"ret=-2\r\n");
		return -2;
	}

	result = 1;
	// e.g:
	// +CIPSTATUS:0,"TCP","192.168.2.102",50797,7634,1
	switch(StatusState) {
		case CIPSTATUS_PARSE_CHAR_PLUS:
			// sprintf(msg_irs,"PLUS\r\n");
			for(i = old_i; i < buf_size; i++) {
				if('+' == buf[i]) {
					StatusState = CIPSTATUS_PARSE_CHAR_COLON;
					old_i = i+1;
					break;
				}
			}
			break;
		case CIPSTATUS_PARSE_CHAR_COLON:
			// sprintf(msg_irs,"COLON\r\n");
			for(i = old_i; i < buf_size; i++) {
				if(':' == buf[i]) {
					StatusState = CIPSTATUS_PARSE_CID;
					old_i = i+1;
					break;
				}
			}
			break;
		case CIPSTATUS_PARSE_CID:
			// no use now
			// sprintf(msg_irs,"CID\r\n");
			for(i = old_i; i < buf_size; i++) {
				if(',' == buf[i]) {
					StatusState = CIPSTATUS_PARSE_TCP_UDP;
					old_i = i+1;
					break;
				}
			}
			break;
		case CIPSTATUS_PARSE_TCP_UDP:
			// no use now
			// sprintf(msg_irs,"TCP_UDP\r\n");
			for(i = old_i; i < buf_size; i++) {
				if(',' == buf[i]) {
					StatusState = CIPSTATUS_PARSE_CIP_QUOTE_1;
					old_i = i+1;
					break;
				}
			}
			break;
		case CIPSTATUS_PARSE_CIP_QUOTE_1:
			// sprintf(msg_irs,"QUOTE_1\r\n");
			for(i = old_i; i < buf_size; i++) {
				if('"' == buf[i]) {
					StatusState = CIPSTATUS_PARSE_CIP_QUOTE_2;
					old_i = i+1;
					break;
				}
			}
			break;
		case CIPSTATUS_PARSE_CIP_QUOTE_2:
			// sprintf(msg_irs,"QUOTE_2\r\n");
			for(i = old_i; i < buf_size; i++) {
				if('"' == buf[i]) {
					StatusState = CIPSTATUS_PARSE_CIP;
					// PS: old_i is now at the first number of IP
					// old_i = i+1;
					break;
				}
			}
			break;
		case CIPSTATUS_PARSE_CIP:
			// sprintf(msg_irs,"CIP\r\n");
			for(i = old_i; i < old_i+16;i++) {
				if('.' == buf[i] || isdigit(buf[i])) {
					(socket_data->addr).sin_addr.s_addr[i-old_i] = buf[i]; 
					// INADDR_ANY[i-old_i] = buf[i];
				} else {
					(socket_data->addr).sin_addr.s_addr[i-old_i] = '\0';
					// INADDR_ANY[i-old_i] = '\0';
					break;
				}
			}
			for(i = old_i; i < buf_size; i++) {
				if(',' == buf[i]) {
					StatusState = CIPSTATUS_PARSE_CPORT;
					// sprintf(msg_irs,"%s\r\n", (socket_data->addr).sin_addr.s_addr);
					old_i = i+1;
					break;
				}
			}
			break;
		case CIPSTATUS_PARSE_CPORT:
			// sprintf(msg_irs,"CPORT\r\n");
			socket_data->addr.sin_port = atoi((const char *)(&buf[old_i]));
			for(i = old_i; i < buf_size; i++) {
				if(',' == buf[i]) {
					StatusState = CIPSTATUS_PARSE_SPORT;
					old_i = i+1;
					break;
				}
			}
			break;
		case CIPSTATUS_PARSE_SPORT:
		case CIPSTATUS_PARSE_CSTATE:
			// sprintf(msg_irs,"ret=0\r\n");
			StatusState = CIPSTATUS_PARSE_CHAR_PLUS;
			old_i = 0;
			result = 0;
			break;
		default:
			StatusState = CIPSTATUS_PARSE_CHAR_PLUS;
			old_i = 0;
			result = 2;
			break;
	}

	return result;
}

sint32_t ParseCIFSR(uint8_t * buf, uint32_t buf_size, uint8_t * addr_buf, uint32_t addr_buf_size)
{
	static sint32_t SrState = CIFSR_PARSE_FIRST_COMMA;
	static sint32_t i = 0, old_i = 0;
	static sint32_t result = 0; // 0->Parse Completed, 1->Parsing..., 2->Error

	if(!buf) {
		// sprintf(msg_irs,"ret=-1\r\n");
		return -1;
	}
	if(!addr_buf) {
		// sprintf(msg_irs,"ret=-2\r\n");
		return -2;
	}

	result = 1;
	// e.g:
	// +CIFSR:STAIP,"192.168.2.103"
	// +CIFSR:STAMAC,"60:01:94:08:3e:fc"
	switch(SrState) {
		case CIFSR_PARSE_FIRST_COMMA:
			for(i = old_i; i < buf_size; i++) {
				if(',' == buf[i]) {
					SrState = CIFSR_PARSE_IP_QUOTE_1;
					old_i = i+1;
					break;
				}
			}
			break;
		case CIFSR_PARSE_IP_QUOTE_1:
			for(i = old_i; i < buf_size; i++) {
				if('"' == buf[i]) {
					SrState = CIFSR_PARSE_IP_QUOTE_2;
					old_i = i+1;
					break;
				}
			}
			break;
		case CIFSR_PARSE_IP_QUOTE_2:
			for(i = old_i; i < buf_size; i++) {
				if('"' == buf[i]) {
					SrState = CIFSR_PARSE_IP;
					// PS: old_i is now at the first number of IP
					// old_i = i+1;
					break;
				}
			}
			break;
		case CIFSR_PARSE_IP:
			for(i = old_i; i < old_i+addr_buf_size;i++) {
				if('.' == buf[i] || isdigit(buf[i])) {
					addr_buf[i-old_i] = buf[i];
				} else {
					addr_buf[i-old_i] = '\0';
					break;
				}
			}
			for(i = old_i; i < buf_size; i++) {
				if('"' == buf[i]) {
					SrState = CIFSR_PARSE_SECOND_COMMA;
					old_i = i+1;
					break;
				}
			}
			break;
		case CIFSR_PARSE_SECOND_COMMA:
			for(i = old_i; i < buf_size; i++) {
				if(',' == buf[i]) {
					SrState = CIFSR_PARSE_MAC_QUOTE_1;
					old_i = i+1;
					break;
				}
			}
			break;
		case CIFSR_PARSE_MAC_QUOTE_1:
			for(i = old_i; i < buf_size; i++) {
				if('"' == buf[i]) {
					SrState = CIFSR_PARSE_MAC_QUOTE_2;
					old_i = i+1;
					break;
				}
			}
			break;
		case CIFSR_PARSE_MAC_QUOTE_2:
			for(i = old_i; i < buf_size; i++) {
				if('"' == buf[i]) {
					SrState = CIFSR_PARSE_MAC;
					// old_i = i+1;
					break;
				}
			}
			break;
		case CIFSR_PARSE_MAC:
			for(i = old_i; i < buf_size; i++) {
				if('"' == buf[i]) {
					SrState = CIFSR_PARSE_FIRST_COMMA;
					result = 0;
					old_i = 0;
					break;
				}
			}
			break;
		default:
			SrState = CIFSR_PARSE_FIRST_COMMA;
			old_i = 0;
			result = 2;
			break;
	}

	return result;
}

sint32_t TryDispatch(uint32_t sockfd, uint32_t msg_type, uint8_t * msg, uint32_t msg_size)
{
	static BC_SocketData * SockServ = NULL;
	SockServ = GetSockDataIrq(SOCK_SERV_FD);
	if(!SockServ) {
		WifiRecvFlag |= WIFI_MSG_FLAG_INVALID_SOCK_SERV;
		return;
	}

	if(!msg) {
		// SockServ.backlog = -1;
		return -1;
	}
	if(!ASSERT_SOCK_VALID(sockfd)) {
		// SockServ.backlog = -2;
		return -2;
	}

	switch(msg_type) {
		case WIFI_MSG_FLAG_GOT_CONNECT:
			// SockServ.backlog = -9;
			if(sock_data[sockfd].wifi_recv_flag & WIFI_MSG_FLAG_GOT_CONNECT) {
				// SockServ.backlog = -3;
				return -3;
			}
			// TODO: Check net-internet client sockfd
			if(BC_OK != CheckDataFlag(msg, msg_size, WIFI_FLAG_CONN_END, WIFI_FLAG_CONN_END_SIZE, TRUE)) {
				// SockServ.backlog = -4;
				return -4;
			}
			// TODO: Add mutex
			SockServ->wifi_id = sockfd;
			// SockServ.wifi_recv_flag |= 0x1000000000;
			sock_data[sockfd].wifi_recv_flag |= WIFI_MSG_FLAG_GOT_CONNECT;
			sock_data[sockfd].wifi_recv_flag &= ~WIFI_MSG_FLAG_GOT_CLOSED;
			if(BC_TRUE != xQueueSendFromISR(SockServ->queue_handle, SockServ, NULL)) {
				sock_data[sockfd].wifi_recv_flag |= WIFI_MSG_FLAG_SERV_QUE_OVERFLOW;
			} 
			return BC_OK;
		case WIFI_MSG_FLAG_GOT_CLOSED:
			// TODO: Add mutex
				// SockServ.backlog = -6;
			// if(!(sock_data[sockfd].wifi_recv_flag & WIFI_MSG_FLAG_GOT_CLOSED)) {
			// 	SockServ.backlog = -5;
			// 	return -3;
			// }
			// TODO: Check net-internet client sockfd
			if(BC_OK != CheckDataFlag(msg, msg_size, WIFI_FLAG_CLOSED_END, WIFI_FLAG_CLOSED_END_SIZE, TRUE)) {
				// SockServ.backlog = -7;
				return -4;
			}
			// TODO: Add mutex
			// SockServ.backlog = -8;
			SockServ->wifi_id = sockfd;
			// sock_data[sockfd].wifi_recv_flag |= WIFI_MSG_FLAG_GOT_CLOSED;
			sock_data[sockfd].wifi_recv_flag &= ~WIFI_MSG_FLAG_GOT_CONNECT;
			if(BC_TRUE != xQueueSendFromISR(SockServ->queue_handle, SockServ, NULL)) {
				sock_data[sockfd].wifi_recv_flag |= WIFI_MSG_FLAG_SERV_QUE_OVERFLOW;
			}
			return BC_OK;
		case WIFI_MSG_FLAG_GOT_IPD:
			return BC_OK;
		default:
			return -1;
	}
}

sint32_t ParseCIPCLOSE(uint8_t * buf, uint32_t buf_size, BC_SocketData * socket_data)
{
	static sint32_t CloseState = CIPCLOSE_PARSE_CHAR_EQUAL;
	static sint32_t i = 0, old_i = 0;
	static sint32_t result = 0; // 0->Parse Completed, 1->Parsing..., 2->Error

	if(!buf) {
		// sprintf(msg_irs,"ret=-1\r\n");
		return -1;
	}
	if(!socket_data) {
		// sprintf(msg_irs,"ret=-2\r\n");
		return -2;
	}

	result = 1;
	// e.g:
	// AT+CIPCLOSE=0
	// 0,CLOSED
	// 
	// OK 
	switch(CloseState) {
		case CIPCLOSE_PARSE_CHAR_EQUAL:
			// x[0] = 'X';
			// uputn(USART_TERMINAL, x, 3);
			for(i = old_i; i < buf_size; i++) {
				// x[0] = buf[i];
				// uputn(USART_TERMINAL, x, 3);
				if('=' == buf[i]) {
					CloseState = CIPCLOSE_PARSE_SOCKET_ID;
					old_i = i+1;
					break;
				}
			}
			break;
		case CIPCLOSE_PARSE_SOCKET_ID:
			// x[0] = 'Y';
			// uputn(USART_TERMINAL, x, 3);
			
			// if(!isdigit(buf[old_i])) {
			// 	socket_data->wifi_id = atoi((const char *)&buf[old_i]);
			// 	CloseState = CIPCLOSE_PARSE_CHAR_D;
			// 	old_i += 1;
			// }
			// } else {
			// 	result = 2;
			// 	CloseState = CIPCLOSE_PARSE_CHAR_EQUAL;
			// 	old_i = 0;
			// }
			for(i = old_i; i < buf_size; i++) {
				if(!isdigit(buf[i])) {
					socket_data->wifi_id = atoi((const char *)&buf[old_i]);
					CloseState = CIPCLOSE_PARSE_CHAR_D;
					old_i = i+1;
					break;
				}
			}
			break;
		case CIPCLOSE_PARSE_CHAR_D:
			// x[0] = 'Z';
			// uputn(USART_TERMINAL, x, 3);
			for(i = old_i; i < buf_size; i++) {
				if('D' == buf[i]) {
					CloseState = CIPCLOSE_PARSE_RESULT;
					old_i = i+1;
					break;
				}
			}
			break;
		case CIPCLOSE_PARSE_RESULT:
			// x[0] = 'D';
			// uputn(USART_TERMINAL, x, 3);
			for(i = old_i; i < buf_size; i++) {
				if(isalpha(buf[i])) {
					if(i>0 && 'O' == buf[i-1] &&
					   'K' == buf[i]) {
						CloseState = CIPCLOSE_PARSE_CHAR_EQUAL;
						old_i = 0;
						result = 0;
					} else if(i > 3 && 'E' == buf[i-4] 	&&
							  'R' == buf[i-3] 	&&
							  'R' == buf[i-2] 	&&
							  'O' == buf[i-1] 	&&
							  'R' == buf[i] 	
							) {
						CloseState = CIPCLOSE_PARSE_CHAR_EQUAL;
						old_i = 0;
						result = 0;
					}
				}
			}
			break;
		default:
			// x[0] = 'E';
			// uputn(USART_TERMINAL, x, 3);
			CloseState = CIPCLOSE_PARSE_CHAR_EQUAL;
			old_i = 0;
			result = 2;
			break;
	}

	return result;
}

sint32_t ParseCIPSEND(uint8_t * buf, uint32_t buf_size, BC_SocketData * socket_data)
{
	static sint32_t SendState = CIPSEND_PARSE_EQUAL_CHAR;
	static sint32_t i = 0, old_i = 0;
	static sint32_t result = 0; // 0->Parse Completed, 1->Parsing..., 2->Error
	static sint32_t wifi_id = -1;

	if(!buf) {
		// sprintf(msg_irs,"ret=-1\r\n");
		return -1;
	}
	if(!socket_data) {
		// sprintf(msg_irs,"ret=-2\r\n");
		return -2;
	}

	result = 1;
	// e.g:
	// AT+CIPSEND=0,3
	// 
	// OK 
	// >
	// Recv 3 bytes
	// 
	// SEND OK
	switch(SendState) {
		case CIPSEND_PARSE_EQUAL_CHAR:
			for(i = old_i; i < buf_size; i++) {
				if('=' == buf[i]) {
					SendState = CIPSEND_PARSE_WIFI_ID;
					old_i = i+1;
					break;
				}
			}
			break;
		case CIPSEND_PARSE_WIFI_ID:
			for(i = old_i; i < buf_size; i++) {
				if(',' == buf[i]) {
					SendState = CIPSEND_PARSE_RESULT_1;
					wifi_id = atoi((const char *)&buf[old_i]);
					old_i = i+1;
					break;
				}
			}
			break;
		case CIPSEND_PARSE_RESULT_1:
			for(i = old_i; i < buf_size; i++) {
				if('K' == buf[i] && i > 0 && 'O' == buf[i-1]) {
					SendState = CIPSEND_PARSE_BRACKET;
					// first OK end
					old_i = 0;
					break;
				}
			}
			break;
		case CIPSEND_PARSE_BRACKET:
			for(i = old_i; i < buf_size; i++) {
				if('>' == buf[i]) {
					SendState = CIPSEND_PARSE_CHAR_s;
					old_i = i+1;
					break;
				}
			}
			break;
		case CIPSEND_PARSE_CHAR_s:
			for(i = old_i; i < buf_size; i++) {
				if('s' == buf[i]) {
					SendState = CIPSEND_PARSE_STR_SEND;
					old_i = i+1;
					break;
				}
			}
			break;
		case CIPSEND_PARSE_STR_SEND:
			for(i = old_i; i < buf_size; i++) {
				if('D' == buf[i] && i > 3 &&
				   'N' == buf[i-1]  &&
				   'E' == buf[i-2]  &&
				   'S' == buf[i-3]) {
					SendState = CIPSEND_PARSE_RESULT_2;
					old_i = i+1;
					break;
				}
			}
			break;
		case CIPSEND_PARSE_RESULT_2:
			if(buf_size - old_i > 3) {
				if(' ' == buf[old_i] 	&& 
				   'O' == buf[old_i+1] 	&& 
				   'K' == buf[old_i+2]) {
					socket_data->wifi_id = wifi_id;
					result = 0;
				} else {
					result = 2;
				}	
				wifi_id = -1; // initialize
				SendState = CIPSEND_PARSE_EQUAL_CHAR;
				old_i = 0;
			}
			break;
		default:
			SendState = CIPSEND_PARSE_EQUAL_CHAR;
			old_i = 0;
			result = -3;
			break;
	}

	return result;
}

