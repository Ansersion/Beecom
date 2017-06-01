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

extern BC_SocketData sock_data[];
extern BC_SocketData sock_serv;

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

volatile void IrqUsartWifi(void)
{
	static uint32_t Index = 0;
	static uint16_t RxData=0;
	static sint32_t u32CurrentSockId = -1;
	// BC_SocketData sock_data_tmp;

	if(USART_GetITStatus(USART_WIFI, USART_IT_RXNE) == RESET) {
		return;
	}
	RxData = USART_RECEIVE(USART_WIFI, RxData);

	UsartWifiBuf[Index++] = (uint8_t)RxData;
	if('\r' == UsartWifiBuf[0] || '\n' == UsartWifiBuf[0]) {
		Index = 0;
		return;
	}
	if(Index >= USART_WIFI_BUF_SIZE - 1) {
		WifiRecvFlag |= WIFI_MSG_FLAG_BUF_OVERFLOW;
		UsartWifiBuf[USART_WIFI_BUF_SIZE-1] = '\0';
		Index = 0;
		return;
	}
	if(BC_OK == CheckDataFlag(UsartWifiBuf, Index, WIFI_FLAG_OK_END, WIFI_FLAG_OK_END_SIZE, TRUE)) {
		WifiRecvFlag |= WIFI_MSG_FLAG_GENERAL_OK;
		UsartWifiBuf[Index] = '\0';
		Index = 0;
		return;
	}
	if(BC_OK == CheckDataFlag(UsartWifiBuf, Index, WIFI_FLAG_ERROR_END, WIFI_FLAG_ERROR_END_SIZE, TRUE)) {
		WifiRecvFlag |= WIFI_MSG_FLAG_GENERAL_ERR;
		UsartWifiBuf[Index] = '\0';
		Index = 0;
		return;
	}

	// client connects to local server
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
		if(ParseCIPSTATUS(UsartWifiBuf+WIFI_FLAG_STATUS_ST_SIZE, Index-WIFI_FLAG_STATUS_ST_SIZE, &sock_data[sock_serv.wifi_id]) == 0) {
			sock_serv.wifi_recv_flag &= ~WIFI_MSG_FLAG_GOT_CONNECT;
			if(pdTRUE != xQueueSendFromISR(sock_data[sock_serv.wifi_id].queue_handle, &sock_data[sock_serv.wifi_id], NULL)) {
				// sock_data[
			}
			return;
		}
	}
	if(BC_OK == CheckDataFlag(UsartWifiBuf, Index, WIFI_FLAG_SR_ST, WIFI_FLAG_SR_ST_SIZE, FALSE)) {
		if(ParseCIFSR(UsartWifiBuf+WIFI_FLAG_SR_ST_SIZE, Index-WIFI_FLAG_SR_ST_SIZE, INADDR_ANY, 16) == 0) {
			if(pdTRUE != xQueueSendFromISR(sock_serv.queue_handle, &sock_serv, NULL)) {
				// sock_data[
			}
			return;
		}
	}
	// if(BC_OK == CheckDataFlag(UsartWifiBuf, Index, WIFI_FLAG_IPD_ST, WIFI_FLAG_IPD_ST_SIZE, FALSE) && (WifiRecvFlag & WIFI_MSG_FLAG_GOT_IPD) == 0) {
	// 	// if(ParseIPD(UsartWifiBuf, Index, &sock_data[0]) == 0) {
	// 	// 	// if(pdTRUE != xQueueSendFromISR(xQueue0, &sock_data[0], &xHigherPriorityTaskWoken)) {
	// 	// 	// 	// sprintf(msg_irs, "IPD QUE Err\r\n");
	// 	// 	// } else {
	// 	// 	// 	// sprintf(msg_irs, "IPD QUE OK\r\n");
	// 	// 	// 	WifiRecvFlag |= WIFI_MSG_FLAG_GOT_IPD;
	// 	// 	// }
	// 	// 	Index = 0;
	// 	// 	return;
	// 	// }
	// }
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
					IPDState = IPD_PARSE_COLON;
					old_i = i+1;
					break;
				}
			}
			break;
		case IPD_PARSE_COLON:
			// sprintf(msg_irs,"IPD-COLON\r\n");
			for(i = old_i; i < buf_size; i++) {
				if(':' == buf[i]) {
					IPDState = IPD_PARSE_LEN;
					// PS: old_i is now at the first number of length
					// old_i = i+1;
					break;
				}
			}
			break;
		case IPD_PARSE_LEN:
			num = atoi((const char *)(&buf[old_i]));
			socket_data->ipd_size = num;
			// sprintf(msg_irs,"IPD-LEN=%d, %d\r\n", num, old_i);
			for(i = old_i; i < buf_size; i++) {
				if(':' == buf[i]) {
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
	if(!msg) {
		sock_serv.backlog = -1;
		return -1;
	}
	if(!ASSERT_SOCK_VALID(sockfd)) {
		sock_serv.backlog = -2;
		return -2;
	}

	switch(msg_type) {
		case WIFI_MSG_FLAG_GOT_CONNECT:
			sock_serv.backlog = -9;
			if(sock_data[sockfd].wifi_recv_flag & WIFI_MSG_FLAG_GOT_CONNECT) {
				sock_serv.backlog = -3;
				return -3;
			}
			// TODO: Check net-internet client sockfd
			if(BC_OK != CheckDataFlag(msg, msg_size, WIFI_FLAG_CONN_END, WIFI_FLAG_CONN_END_SIZE, TRUE)) {
				sock_serv.backlog = -4;
				return -4;
			}
			// TODO: Add mutex
			sock_serv.wifi_id = sockfd;
			// sock_serv.wifi_recv_flag |= 0x1000000000;
			sock_data[sockfd].wifi_recv_flag |= WIFI_MSG_FLAG_GOT_CONNECT;
			sock_data[sockfd].wifi_recv_flag &= ~WIFI_MSG_FLAG_GOT_CLOSED;
			if(BC_TRUE != xQueueSendFromISR(sock_serv.queue_handle, &sock_serv, NULL)) {
				sock_data[sockfd].wifi_recv_flag |= WIFI_MSG_FLAG_SERV_QUE_OVERFLOW;
			} 
			return BC_OK;
		case WIFI_MSG_FLAG_GOT_CLOSED:
			// TODO: Add mutex
				sock_serv.backlog = -6;
			// if(!(sock_data[sockfd].wifi_recv_flag & WIFI_MSG_FLAG_GOT_CLOSED)) {
			// 	sock_serv.backlog = -5;
			// 	return -3;
			// }
			// TODO: Check net-internet client sockfd
			if(BC_OK != CheckDataFlag(msg, msg_size, WIFI_FLAG_CLOSED_END, WIFI_FLAG_CLOSED_END_SIZE, TRUE)) {
				sock_serv.backlog = -7;
				return -4;
			}
			// TODO: Add mutex
			sock_serv.backlog = -8;
			sock_serv.wifi_id = sockfd;
			// sock_data[sockfd].wifi_recv_flag |= WIFI_MSG_FLAG_GOT_CLOSED;
			sock_data[sockfd].wifi_recv_flag &= ~WIFI_MSG_FLAG_GOT_CONNECT;
			if(BC_TRUE != xQueueSendFromISR(sock_serv.queue_handle, &sock_serv, NULL)) {
				sock_data[sockfd].wifi_recv_flag |= WIFI_MSG_FLAG_SERV_QUE_OVERFLOW;
			}
			return BC_OK;
		case WIFI_MSG_FLAG_GOT_IPD:
			return BC_OK;
		default:
			return -1;
	}
}
