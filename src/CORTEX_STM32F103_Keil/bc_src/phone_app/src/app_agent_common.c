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

// FreeRTOS headers
#include <FreeRTOS.h>
#include <task.h>

// STM32 headers
#include <stm32f10x_usart.h>

// Beecom headers
#include <app_agent_common.h>
#include <app_agent_irq.h>
#include <terminal.h>
#include <mutex.h>

BC_SocketData sock_data[BC_MAX_SOCKET_NUM];
static sint8_t pu8CmdMsg[128];

static const sint8_t * WifiCmdReset = "AT+RST\r\n";
static const sint8_t * WifiCmdSetMode = "AT+CWMODE=%d\r\n";
static const sint8_t * WifiCmdSetSsidAndPwd = "AT+CWJAP=\"%s\",\"%s\"\r\n";
static const sint8_t * WifiCmdSetMux = "AT+CIPMUX=%d\r\n";

sint32_t BC_WifiReset(uint32_t * timeout)
{
	sint32_t ret = BC_OK;

	while(1) {
		strcpy(pu8CmdMsg, WifiCmdReset);
		uputs(USART_WIFI, pu8CmdMsg);
		vTaskDelay(10 / portTICK_RATE_MS); // at least delay 10ms
		if(timeout) {
			vTaskDelay(*timeout / portTICK_RATE_MS);
		}
		if(BCMutexLock(&WifiRecvFlagMutex) == BC_OK) {
			if(WifiRecvFlag & WIFI_MSG_FLAG_GENERAL_OK) {
				WifiRecvFlag &= ~WIFI_MSG_FLAG_GENERAL_OK;
				WifiRecvFlag |= WIFI_MSG_FLAG_BUF_RESET;
				BCMutexUnlock(&WifiRecvFlagMutex);
				ret = BC_OK;
				break;
			} else if(WifiRecvFlag & WIFI_MSG_FLAG_GENERAL_ERR) {
				WifiRecvFlag &= ~WIFI_MSG_FLAG_GENERAL_ERR;
				WifiRecvFlag |= WIFI_MSG_FLAG_BUF_RESET;
				BCMutexUnlock(&WifiRecvFlagMutex);
				ret = -4;
				break;
			}
			BCMutexUnlock(&WifiRecvFlagMutex);
		}
		if(timeout) {
			ret = -1;
			break;
		}
		
	}

	return ret;
}

sint32_t BC_WifiSetMode(enum WIFI_MODE mode, uint32_t * timeout)
{
	sint32_t ret = BC_OK;

	while(1) {
		sprintf(pu8CmdMsg, WifiCmdSetMode, mode);
		uputs(USART_WIFI, pu8CmdMsg);
		vTaskDelay(10 / portTICK_RATE_MS); // at least delay 10ms
		if(timeout) {
			vTaskDelay(*timeout / portTICK_RATE_MS);
		}
		if(BCMutexLock(&WifiRecvFlagMutex) == BC_OK) {
			if(WifiRecvFlag & WIFI_MSG_FLAG_GENERAL_OK) {
				WifiRecvFlag &= ~WIFI_MSG_FLAG_GENERAL_OK;
				WifiRecvFlag |= WIFI_MSG_FLAG_BUF_RESET;
				BCMutexUnlock(&WifiRecvFlagMutex);
				ret = BC_OK;
				break;
			} else if(WifiRecvFlag & WIFI_MSG_FLAG_GENERAL_ERR) {
				WifiRecvFlag &= ~WIFI_MSG_FLAG_GENERAL_ERR;
				WifiRecvFlag |= WIFI_MSG_FLAG_BUF_RESET;
				BCMutexUnlock(&WifiRecvFlagMutex);
				ret = -4;
				break;
			}
			BCMutexUnlock(&WifiRecvFlagMutex);
		}
		if(timeout) {
			ret = -1;
			break;
		}
		
	}

	return ret;
}

sint32_t BC_WifiSetSsidAndPwd(uint8_t * ssid, uint8_t * pwd, uint32_t * timeout)
{
	sint32_t ret = BC_OK;

	if(!ssid) {
		return -2;
	}
	if(!pwd) {
		return -3;
	}
	while(1) {
		sprintf(pu8CmdMsg, WifiCmdSetSsidAndPwd, ssid, pwd);
		uputs(USART_WIFI, pu8CmdMsg);
		vTaskDelay(10 / portTICK_RATE_MS); // at least delay 10ms
		if(timeout) {
			vTaskDelay(*timeout / portTICK_RATE_MS);
		}
		if(BCMutexLock(&WifiRecvFlagMutex) == BC_OK) {
			if(WifiRecvFlag & WIFI_MSG_FLAG_GENERAL_OK) {
				WifiRecvFlag &= ~WIFI_MSG_FLAG_GENERAL_OK;
				WifiRecvFlag |= WIFI_MSG_FLAG_BUF_RESET;
				BCMutexUnlock(&WifiRecvFlagMutex);
				ret = BC_OK;
				break;
			} else if(WifiRecvFlag & WIFI_MSG_FLAG_GENERAL_ERR) {
				WifiRecvFlag &= ~WIFI_MSG_FLAG_GENERAL_ERR;
				WifiRecvFlag |= WIFI_MSG_FLAG_BUF_RESET;
				BCMutexUnlock(&WifiRecvFlagMutex);
				ret = -4;
				break;
			}
			BCMutexUnlock(&WifiRecvFlagMutex);
		}
		if(timeout) {
			ret = -1;
			break;
		}
		
	}

	return ret;
}

sint32_t BC_WifiSetMux(uint32_t is_mux, uint32_t * timeout)
{
	sint32_t ret = BC_OK;

	while(1) {
		sprintf(pu8CmdMsg, WifiCmdSetMux, is_mux);
		uputs(USART_WIFI, pu8CmdMsg);
		vTaskDelay(10 / portTICK_RATE_MS); // at least delay 10ms
		if(timeout) {
			vTaskDelay(*timeout / portTICK_RATE_MS);
		}
		if(BCMutexLock(&WifiRecvFlagMutex) == BC_OK) {
			if(WifiRecvFlag & WIFI_MSG_FLAG_GENERAL_OK) {
				WifiRecvFlag &= ~WIFI_MSG_FLAG_GENERAL_OK;
				WifiRecvFlag |= WIFI_MSG_FLAG_BUF_RESET;
				BCMutexUnlock(&WifiRecvFlagMutex);
				ret = BC_OK;
				break;
			} else if(WifiRecvFlag & WIFI_MSG_FLAG_GENERAL_ERR) {
				WifiRecvFlag &= ~WIFI_MSG_FLAG_GENERAL_ERR;
				WifiRecvFlag |= WIFI_MSG_FLAG_BUF_RESET;
				BCMutexUnlock(&WifiRecvFlagMutex);
				ret = -4;
				break;
			}
			BCMutexUnlock(&WifiRecvFlagMutex);
		}
		if(timeout) {
			ret = -1;
			break;
		}

	}

	return ret;
}

sint32_t BC_Socket(sint32_t family, sint32_t type, sint32_t protocol)
{
	int i;
	for(i = 0; i < BC_MAX_SOCKET_NUM; i++) {
		if(BC_TRUE != sock_data[i].valid) {
			sock_data[i].valid = BC_TRUE;
			return i;
		}
	}
	return -1;
}

sint32_t BC_Bind(sint32_t sockfd, const BC_Sockaddr * myaddr, uint32_t addrlen)
{
	if(!ASSERT_SOCK_VALID(sockfd)) {
		return -1;
	}
	if(!myaddr) {
		return -2;
	}
	if(BC_TRUE != sock_data[sockfd].valid) {
		return -3;
	}
	memcpy(&(sock_data[sockfd].addr), myaddr, addrlen);
	return 0;
}

sint32_t BC_Listen(sint32_t sockfd, sint32_t backlog)
{
	uint32_t delay_ms = 50;
	uint32_t fail_count = 0;
	if(!ASSERT_SOCK_VALID(sockfd)) {
		return -1;
	}
	if(backlog < 0) {
		return -2;
	}
	if(BC_TRUE != sock_data[sockfd].valid) {
		return -3;
	}
	sock_data[sockfd].backlog = backlog;

	while(!(WifiRecvFlag & WIFI_MSG_FLAG_GENERAL_OK)) {
		sprintf(pu8CmdMsg, "AT+CIPSERVER=1,%d\r\n", sock_data[sockfd].addr.sin_port);
		uputs(USART_WIFI, pu8CmdMsg);
		vTaskDelay(delay_ms);
		if(fail_count++ > BC_MAX_WIFI_FAIL_COUNT) {
			sprintf(pu8CmdMsg, "AT+CIPSERVER=1,%d: Error\r\n", sock_data[sockfd].addr.sin_port);
			// uputs(USART_TERMINAL, pu8CmdMsg);
			return -4;
		}
	}
	if(sockfd != 0) {
		// uputs(USART_TERMINAL, "Error: Only sockfd(LinkNo.)==0 can be server\r\n");
		return -5;
	}
	// sock_data[sockfd].is_server = TRUE;
	WifiRecvFlag &= ~WIFI_MSG_FLAG_GENERAL_OK;

	return 0;
}

sint32_t BC_Accept(sint32_t sockfd, BC_Sockaddr * cliaddr, uint32_t * addrlen)
{
	BC_SocketData sock_data_tmp;

	if(!ASSERT_SOCK_VALID(sockfd)) {
		return -1;
	}
	if(!cliaddr) {
		return -2;
	}
	if(!addrlen) {
		return -3;
	}
	if(BC_TRUE != sock_data[sockfd].valid) {
		return -4;
	}
	// wait for the semphore

	// k_cliaddr = cliaddr;
	// k_addrlen = addrlen;

	// while(pdFALSE == xQueueReceive(xQueue0, &sock_data_tmp, 1000/portTICK_RATE_MS)) {
	// }
	uputs(USART_WIFI, "AT+CIPSTATUS\r\n");
	// while(pdFALSE == xQueueReceive(xQueue0, &sock_data_tmp, 1000/portTICK_RATE_MS)) {
	// 	// uputs(USART_TERMINAL, usart_wifi_buf);
	// }
	memcpy(cliaddr, &(sock_data_tmp.addr), sizeof(BC_Sockaddr));
	WifiRecvFlag &= ~WIFI_MSG_FLAG_GENERAL_OK;
	WifiRecvFlag &= ~WIFI_MSG_FLAG_GOT_CONNECT;

	return 0;
}

sint32_t BC_Connect(sint32_t sockfd, const BC_Sockaddr * servaddr, uint32_t addrlen)
{
	if(!ASSERT_SOCK_VALID(sockfd)) {
		return -1;
	}
	if(!servaddr) {
		return -2;
	}
	if(!sock_data[sockfd].valid) {
		return -3;
	}

	sprintf(pu8CmdMsg, "AT+CIPSTART=%d,\"TCP\",\"%s\",%d", sockfd, servaddr->sin_addr.s_addr, servaddr->sin_port);
	return 0;
}

sint32_t BC_Close(sint32_t sockfd)
{
	if(!ASSERT_SOCK_VALID(sockfd)) {
		return -1;
	}
	if(!sock_data[sockfd].valid) {
		return -2;
	}
	sprintf(pu8CmdMsg, "AT+CIPCLOSE=%d\r\n", sockfd);
	uputs(USART_WIFI, pu8CmdMsg);
	// sock_data[sockfd].valid = FALSE;
	return 0;
}

sint32_t BC_Recv(sint32_t sockfd, void * buff, uint32_t nbytes, sint32_t flags)
{
	BC_SocketData sock_data_tmp;
	if(!ASSERT_SOCK_VALID(sockfd)) {
		return -1;
	}
	if(!buff) {
		return -2;
	}
	if(!sock_data[sockfd].valid) {
		return -3;
	}
	sock_data_tmp.wifi_id = -1;
	// while(pdFALSE == xQueueReceive(xQueue0, &sock_data_tmp, 1000/portTICK_RATE_MS)) {
	// 	// TODO:
	// 	// Judge if remote close
	// }
	// memcpy(SrvBuf, sock_data_tmp.buf, sock_data_tmp.ipd_size);
	// vTaskDelay(1000);
	// printf("recv: got IPD");
	// vTaskDelay(1000);
	// printf("recv: wifi_id=%d\r\n", sock_data_tmp.wifi_id);
	// vTaskDelay(1000);
	// printf("recv: %s\r\n", sock_data_tmp.buf);
	// while(1) {
	// 	vTaskDelay(1000);
	// }
	// wait semphore
	// StartReceiveFlag = 1;
	// SET_START_RECV_FLAG(1);
	// while(StartReceiveFlag)
	// 	;
	// read USART_WIFI(in ISA)
	// taskYIELD
	return sock_data_tmp.ipd_size;
}

sint32_t BC_Send(sint32_t sockfd, const void * buff, uint32_t nbytes, sint32_t flags)
{
	static sint32_t i = 0;
	static uint8_t * buf_tmp = NULL;
	if(!ASSERT_SOCK_VALID(sockfd)) {
		return -1;
	}
	if(!buff) {
		return -2;
	}
	if(!sock_data[sockfd].valid) {
		return -3;
	}

	// memset(pu8CmdMsg, 0, sizeof(pu8CmdMsg));

	WifiRecvFlag &= ~WIFI_MSG_FLAG_GENERAL_OK;
	sprintf(pu8CmdMsg, "AT+CIPSEND=%d,%d\r\n", sockfd, nbytes);
	uputs(USART_WIFI, pu8CmdMsg);
	vTaskDelay(1000); // TODO
	WifiRecvFlag &= ~WIFI_MSG_FLAG_GENERAL_OK;
	buf_tmp = (uint8_t *)buff;
	for(i = 0; i < nbytes; i++) {
		USART_SendData(USART_WIFI, buf_tmp[i]);
		while(USART_GetFlagStatus(USART_WIFI,USART_FLAG_TC)!=SET);
	}

	return nbytes;
}


