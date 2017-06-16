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
#include <wifi_common.h>
#include <wifi_irq.h>
#include <terminal.h>
#include <mutex.h>
#include <wifi_clbk.h>
#include <bc_queue.h>

extern TaskHandle_t DataHubHandle;

WIFI_MODE wifi_mode = WIFI_MODE_INVALID;
WIFI_MODE wifi_server = WIFI_SERVER_INVALID; 

uint8_t wifi_ssid_test[] = "\"hb402-2g\"";
uint8_t wifi_pwd_test[] = "\"68704824\"";


BC_SocketData sock_data[BC_MAX_SOCKET_NUM];
// QueueHandle_t sock_queue[BC_MAX_SOCKET_NUM];
BC_SocketData sock_serv;
uint8_t INADDR_ANY[16];
/**************************
  It's really a dull way to
  define the buffer.
  All of "BC_MAX_SOCKET_NUM"
***************************/
uint8_t sock_buf_0[BC_MAX_SOCKET_BUF_SIZE];
uint8_t sock_buf_1[BC_MAX_SOCKET_BUF_SIZE];
uint8_t sock_buf_2[BC_MAX_SOCKET_BUF_SIZE];
uint8_t sock_buf_3[BC_MAX_SOCKET_BUF_SIZE];
uint8_t sock_buf_4[BC_MAX_SOCKET_BUF_SIZE];

static sint8_t pu8CmdMsg[128];
static sint8_t pu8CmdMsgClose[64];
static sint8_t pu8CmdMsgOpen[128];
static sint8_t pu8CmdMsgConnect[128];

static const sint8_t * WifiCmdReset = "AT+RST\r\n";
static const sint8_t * WifiCmdSetMode = "AT+CWMODE=%d\r\n";
static const sint8_t * WifiCmdSetNet = "AT+CWJAP=\"%s\",\"%s\"\r\n";
static const sint8_t * WifiCmdSetMux = "AT+CIPMUX=%d\r\n";
static const sint8_t * WifiCmdSetServ = "AT+CIPSERVER=%d,%d\r\n";
static const sint8_t * WifiCmdStatus = "AT+CIPSTATUS\r\n";
static const sint8_t * WifiCmdCloseSock = "AT+CIPCLOSE=%d\r\n";

static const sint8_t * WifiCmdCifSr = "AT+CIFSR\r\n";
static const sint8_t * WifiCmdCifSt = "AT+CIPSTATUS\r\n";

WIFI_MODE GetWifiModeInfo(void)
{
	return wifi_mode;
}

WIFI_SERVER GetWifiServerInfo(void)
{
	return wifi_server;
}

sint32_t BC_WifiReset(uint32_t * timeout)
{
	sint32_t ret = BC_OK;
	uint32_t fail_count = 0;

	while(1) {
		if(BCMutexLock(&WifiRecvFlagMutex) == BC_OK) {
			WifiRecvFlag = 0;
			BCMutexUnlock(&WifiRecvFlagMutex);
			break;
		}
	}

	while(1) {
		// check max failed count
		if(++fail_count > WIFI_RESET_MAX_FAIL_COUNT) {
			ret = -10;
			break;
		}
		// send command
		strcpy(pu8CmdMsg, WifiCmdReset);
		uputs(USART_WIFI, pu8CmdMsg);
		// delay a little time(at least) 
		vTaskDelay(10 / portTICK_RATE_MS); // at least delay 10ms
		// delay 'timeout' microseconds if any
		if(timeout) {
			vTaskDelay(*timeout / portTICK_RATE_MS);
		}
		if(BCMutexLock(&WifiRecvFlagMutex) == BC_OK) {
			// check if setting OK
			if(WifiRecvFlag & WIFI_MSG_FLAG_GENERAL_OK) {
				WifiRecvFlag &= ~WIFI_MSG_FLAG_GENERAL_OK;
				BCMutexUnlock(&WifiRecvFlagMutex);
				ret = BC_OK;
				break;
			// check if setting error
			} else if(WifiRecvFlag & WIFI_MSG_FLAG_GENERAL_ERR) {
				WifiRecvFlag &= ~WIFI_MSG_FLAG_GENERAL_ERR;
				BCMutexUnlock(&WifiRecvFlagMutex);
				ret = -4;
				break;
			}
			BCMutexUnlock(&WifiRecvFlagMutex);
		}
		// after 'timeout' microseconds there is no
		// result, so return error
		if(timeout) {
			ret = -1;
			break;
		}
		
	}

	return ret;
}

sint32_t BC_WifiSetMode(WIFI_MODE mode, uint32_t * timeout)
{
	sint32_t ret = BC_OK;
	uint32_t fail_count = 0;
	if(WIFI_MODE_INVALID == mode) {
		return -11;
	}

	while(1) {
		// check max failed count
		if(++fail_count > WIFI_SET_MODE_MAX_FAIL_COUNT) {
			ret = -10;
			break;
		}
		// send command
		sprintf(pu8CmdMsg, WifiCmdSetMode, mode);
		uputs(USART_WIFI, pu8CmdMsg);
		// delay a little time(at least) 
		vTaskDelay(10 / portTICK_RATE_MS); // at least delay 10ms
		// delay 'timeout' microseconds if any
		if(timeout) {
			vTaskDelay(*timeout / portTICK_RATE_MS);
		}
		if(BCMutexLock(&WifiRecvFlagMutex) == BC_OK) {
			// check if setting OK
			if(WifiRecvFlag & WIFI_MSG_FLAG_GENERAL_OK) {
				WifiRecvFlag &= ~WIFI_MSG_FLAG_GENERAL_OK;
				BCMutexUnlock(&WifiRecvFlagMutex);
				wifi_mode = mode;
				ret = BC_OK;
				break;
			// check if setting error
			} else if(WifiRecvFlag & WIFI_MSG_FLAG_GENERAL_ERR) {
				WifiRecvFlag &= ~WIFI_MSG_FLAG_GENERAL_ERR;
				BCMutexUnlock(&WifiRecvFlagMutex);
				ret = -4;
				break;
			}
			BCMutexUnlock(&WifiRecvFlagMutex);
		}
		// after 'timeout' microseconds there is no
		// result, so return error
		if(timeout) {
			ret = -1;
			break;
		}
		
	}

	return ret;
}

sint32_t BC_WifiSetNet(uint8_t * ssid, uint8_t * pwd, uint32_t * timeout)
{
	sint32_t ret = BC_OK;
	uint32_t fail_count = 0;

	while(1) {
		// check max failed count
		if(++fail_count > WIFI_SET_NET) {
			ret = -10;
			break;
		}
		// send command
		sprintf(pu8CmdMsg, WifiCmdSetNet, ssid, pwd);
		uputs(USART_WIFI, pu8CmdMsg);
		// delay a little time(at least) 
		vTaskDelay(10 / portTICK_RATE_MS); // at least delay 10ms
		// delay 'timeout' microseconds if any
		if(timeout) {
			vTaskDelay(*timeout / portTICK_RATE_MS);
		}
		if(BCMutexLock(&WifiRecvFlagMutex) == BC_OK) {
			// check if setting OK
			if(WifiRecvFlag & WIFI_MSG_FLAG_GENERAL_OK) {
				WifiRecvFlag &= ~WIFI_MSG_FLAG_GENERAL_OK;
				BCMutexUnlock(&WifiRecvFlagMutex);
				ret = BC_OK;
				break;
			// check if setting error
			} else if(WifiRecvFlag & WIFI_MSG_FLAG_GENERAL_ERR) {
				WifiRecvFlag &= ~WIFI_MSG_FLAG_GENERAL_ERR;
				BCMutexUnlock(&WifiRecvFlagMutex);
				ret = -4;
				break;
			}
			BCMutexUnlock(&WifiRecvFlagMutex);
		}
		// after 'timeout' microseconds there is no
		// result, so return error
		if(timeout) {
			ret = -1;
			break;
		}
		
	}

	return ret;
}

sint32_t BC_WifiSetMux(WIFI_MUX mux_mode, uint32_t * timeout)
{
	sint32_t ret = BC_OK;
	uint32_t fail_count = 0;

	if(mux_mode == WIFI_MUX_INVALID) {
		return -11;
	}

	while(1) {
		// check max failed count
		if(++fail_count > WIFI_SET_MUX_MAX_FAIL_COUNT) {
			ret = -10;
			break;
		}
		// send command
		sprintf(pu8CmdMsg, WifiCmdSetMux, mux_mode);
		uputs(USART_WIFI, pu8CmdMsg);
		// delay a little time(at least) 
		vTaskDelay(10 / portTICK_RATE_MS); // at least delay 10ms
		// delay 'timeout' microseconds if any
		if(timeout) {
			vTaskDelay(*timeout / portTICK_RATE_MS);
		}
		if(BCMutexLock(&WifiRecvFlagMutex) == BC_OK) {
			// check if setting OK
			if(WifiRecvFlag & WIFI_MSG_FLAG_GENERAL_OK) {
				WifiRecvFlag &= ~WIFI_MSG_FLAG_GENERAL_OK;
				BCMutexUnlock(&WifiRecvFlagMutex);
				ret = BC_OK;
				break;
			// check if setting error
			} else if(WifiRecvFlag & WIFI_MSG_FLAG_GENERAL_ERR) {
				WifiRecvFlag &= ~WIFI_MSG_FLAG_GENERAL_ERR;
				BCMutexUnlock(&WifiRecvFlagMutex);
				ret = -4;
				break;
			}
			BCMutexUnlock(&WifiRecvFlagMutex);
		}
		// after 'timeout' microseconds there is no
		// result, so return error
		if(timeout) {
			ret = -1;
			break;
		}
		
	}

	return ret;
}

sint32_t BC_WifiSetServ(WIFI_SERVER server_mode, uint16_t port, uint32_t * timeout)
{
	sint32_t ret = BC_OK;
	uint32_t fail_count = 0;

	if(server_mode == WIFI_SERVER_INVALID) {
		return -11;
	}

	while(1) {
		// check max failed count
		if(++fail_count > WIFI_SET_SERV_MAX_FAIL_COUNT) {
			ret = -10;
			break;
		}
		// send command
		sprintf(pu8CmdMsg, WifiCmdSetServ, server_mode, port);
		uputs(USART_WIFI, pu8CmdMsg);
		// delay a little time(at least) 
		vTaskDelay(10 / portTICK_RATE_MS); // at least delay 10ms
		// delay 'timeout' microseconds if any
		if(timeout) {
			vTaskDelay(*timeout / portTICK_RATE_MS);
		}
		if(BCMutexLock(&WifiRecvFlagMutex) == BC_OK) {
			// check if setting OK
			if(WifiRecvFlag & WIFI_MSG_FLAG_GENERAL_OK) {
				WifiRecvFlag &= ~WIFI_MSG_FLAG_GENERAL_OK;
				BCMutexUnlock(&WifiRecvFlagMutex);
				wifi_server = server_mode;
				ret = BC_OK;
				break;
			// check if setting error
			} else if(WifiRecvFlag & WIFI_MSG_FLAG_GENERAL_ERR) {
				WifiRecvFlag &= ~WIFI_MSG_FLAG_GENERAL_ERR;
				BCMutexUnlock(&WifiRecvFlagMutex);
				ret = -4;
				break;
			}
			BCMutexUnlock(&WifiRecvFlagMutex);
		}
		// after 'timeout' microseconds there is no
		// result, so return error
		if(timeout) {
			ret = -1;
			break;
		}
		
	}

	return ret;
}

sint32_t BC_WifiQuerySr(uint32_t * timeout)
{
	sint32_t ret = BC_OK;
	uint32_t fail_count = 0;

	while(1) {
		// check max failed count
		if(++fail_count > WIFI_CIF_SR_MAX_FAIL_COUNT) {
			ret = -10;
			break;
		}
		// send command
		strcpy(pu8CmdMsg, WifiCmdCifSr);
		uputs(USART_WIFI, pu8CmdMsg);
		// delay a little time(at least) 
		vTaskDelay(10 / portTICK_RATE_MS); // at least delay 10ms
		// delay 'timeout' microseconds if any
		if(timeout) {
			vTaskDelay(*timeout / portTICK_RATE_MS);
		}
		if(BCMutexLock(&WifiRecvFlagMutex) == BC_OK) {
			// check if setting OK
			if(WifiRecvFlag & WIFI_MSG_FLAG_GENERAL_OK) {
				WifiRecvFlag &= ~WIFI_MSG_FLAG_GENERAL_OK;
				BCMutexUnlock(&WifiRecvFlagMutex);
				ret = BC_OK;
				break;
			// check if setting error
			} else if(WifiRecvFlag & WIFI_MSG_FLAG_GENERAL_ERR) {
				WifiRecvFlag &= ~WIFI_MSG_FLAG_GENERAL_ERR;
				BCMutexUnlock(&WifiRecvFlagMutex);
				ret = -4;
				break;
			}
			BCMutexUnlock(&WifiRecvFlagMutex);
		}
		// after 'timeout' microseconds there is no
		// result, so return error
		if(timeout) {
			ret = -1;
			break;
		}
		
	}

	return ret;
}

sint32_t BC_WifiQuerySt(uint32_t * timeout)
{
	sint32_t ret = BC_OK;
	uint32_t fail_count = 0;
	printf("QUERY_ST\r\n");

	while(1) {
		// check max failed count
		if(++fail_count > WIFI_CIF_SR_MAX_FAIL_COUNT) {
			ret = -10;
			break;
		}
		// send command
		strcpy(pu8CmdMsg, WifiCmdCifSt);
		uputs(USART_WIFI, pu8CmdMsg);
		// delay a little time(at least) 
		vTaskDelay(10 / portTICK_RATE_MS); // at least delay 10ms
		// delay 'timeout' microseconds if any
		if(timeout) {
			vTaskDelay(*timeout / portTICK_RATE_MS);
		}
		if(BCMutexLock(&WifiRecvFlagMutex) == BC_OK) {
			// check if setting OK
			if(WifiRecvFlag & WIFI_MSG_FLAG_GENERAL_OK) {
				WifiRecvFlag &= ~WIFI_MSG_FLAG_GENERAL_OK;
				BCMutexUnlock(&WifiRecvFlagMutex);
				ret = BC_OK;
				break;
			// check if setting error
			} else if(WifiRecvFlag & WIFI_MSG_FLAG_GENERAL_ERR) {
				WifiRecvFlag &= ~WIFI_MSG_FLAG_GENERAL_ERR;
				BCMutexUnlock(&WifiRecvFlagMutex);
				ret = -4;
				break;
			}
			BCMutexUnlock(&WifiRecvFlagMutex);
		}
		// after 'timeout' microseconds there is no
		// result, so return error
		if(timeout) {
			ret = -1;
			break;
		}
		
	}

	return ret;
}

sint32_t BC_WifiGetStatus(uint32_t * timeout)
{
	sint32_t ret = BC_OK;
	uint32_t fail_count = 0;

	while(1) {
		// check max failed count
		if(++fail_count > WIFI_GET_STATUS_MAX_FAIL_COUNT) {
			ret = -10;
			break;
		}
		// send command
		strcpy(pu8CmdMsg, WifiCmdStatus);
		uputs(USART_WIFI, pu8CmdMsg);
		// delay a little time(at least) 
		vTaskDelay(10 / portTICK_RATE_MS); // at least delay 10ms
		// delay 'timeout' microseconds if any
		if(timeout) {
			vTaskDelay(*timeout / portTICK_RATE_MS);
		}
		if(BCMutexLock(&WifiRecvFlagMutex) == BC_OK) {
			// check if setting OK
			if(WifiRecvFlag & WIFI_MSG_FLAG_GENERAL_OK) {
				WifiRecvFlag &= ~WIFI_MSG_FLAG_GENERAL_OK;
				BCMutexUnlock(&WifiRecvFlagMutex);
				ret = BC_OK;
				break;
			// check if setting error
			} else if(WifiRecvFlag & WIFI_MSG_FLAG_GENERAL_ERR) {
				WifiRecvFlag &= ~WIFI_MSG_FLAG_GENERAL_ERR;
				BCMutexUnlock(&WifiRecvFlagMutex);
				ret = -4;
				break;
			}
			BCMutexUnlock(&WifiRecvFlagMutex);
		}
		// after 'timeout' microseconds there is no
		// result, so return error
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
	// sock_data[0]: for server
	// sock_data[BC_MAX_SOCKET_NUM-1]: for client to connect internet
	for(i = 0; i < BC_MAX_SOCKET_NUM - 1; i++) {
		if(BC_TRUE != sock_data[i].valid) {
			sock_data[i].valid = BC_TRUE;
			return i;
		}
	}
	return -1;
}

sint32_t BC_Bind(sint32_t sockfd, const BC_Sockaddr * myaddr, uint32_t addrlen)
{
	/// if(!ASSERT_SOCK_VALID(sockfd)) {
	/// 	return -1;
	/// }
	if(!myaddr) {
		return -2;
	}
	// if(BC_TRUE != sock_data[sockfd].valid) {
	// 	return -3;
	// }
	// memcpy(&(sock_data[sockfd].addr), myaddr, addrlen);
	return BC_OK;;
}

sint32_t BC_Listen(sint32_t sockfd, sint32_t backlog)
{
	// uint32_t delay_ms = 50;
	// uint32_t fail_count = 0;
	// if(!ASSERT_SOCK_VALID(sockfd)) {
	// 	return -1;
	// }
	if(backlog < 0) {
		return -2;
	}
	/// if(BC_TRUE != sock_data[sockfd].valid) {
	/// 	return -3;
	/// }
	/// sock_data[sockfd].backlog = backlog;

	return BC_OK;
}

sint32_t BC_Accept(sint32_t sockfd, BC_Sockaddr * cliaddr, uint32_t * addrlen)
{
	BC_SocketData sock_data_tmp;
	uint32_t fail_count = 0;
	uint32_t wifi_id = 0;
	BC_QueueElement qe;
	stWifiMsgUnit WifiMsgUnit;

	// if(!ASSERT_SOCK_VALID(sockfd)) {
	// 	return -1;
	// }
	if(!cliaddr) {
		return -2;
	}
	if(!addrlen) {
		return -3;
	}
	// if(BC_TRUE != sock_data[sockfd].valid) {
	// 	return -4;
	// }
	// wait for the semphore

	// k_cliaddr = cliaddr;
	// k_addrlen = addrlen;

	while(1) {
		if(pdFALSE == xQueueReceive(sock_serv.queue_handle, &sock_data_tmp, 5000/portTICK_RATE_MS)) {
			printf("addr1: %s\r\n", INADDR_ANY);
			continue;
		}
		if(!(sock_data[sock_data_tmp.wifi_id].wifi_recv_flag & WIFI_MSG_FLAG_GOT_CONNECT)) {
			printf("!WIFI_MSG_FLAG_GOT_CONNECT\r\n");
			continue;
		}

		printf("wifi_id: %d\t%x\r\n", sock_data_tmp.wifi_id, sock_data[sock_data_tmp.wifi_id].wifi_recv_flag);

		BC_MsgInit(&qe, BC_MOD_DEFAULT, BC_MOD_WIFI);
		WifiMsgUnit.WifiClbkCmd = WIFI_CLBK_CMD_QRY_ST;
		BC_MsgSetMsg(&qe, (uint8_t *)&WifiMsgUnit, sizeof(WifiMsgUnit));
		while(1) {
			while(BC_Enqueue(BC_ModInQueue[BC_MOD_DEFAULT], &qe, TIMEOUT_COMMON) == BC_FALSE) {
			}
			vTaskResume(DataHubHandle);

			if(pdFALSE == xQueueReceive(sock_data[sock_data_tmp.wifi_id].queue_handle, &sock_data_tmp, 5000/portTICK_RATE_MS)) {
				fail_count++;
				if(fail_count > 3) {
					fail_count = sock_data_tmp.wifi_id = BC_MAX_SOCKET_NUM+1;
					break;
				}
				printf("try again(addr2: %s)\r\n", INADDR_ANY);
			} else {
				break;
			}
		}
		printf("backlog-s: %d\r\n", sock_serv.backlog);
		printf("backlog-t: %d\r\n", sock_data_tmp.backlog);
		printf("recv-t: %d\r\n", sock_data_tmp.wifi_recv_flag);
		printf("client-ip: %s\r\n", sock_data_tmp.addr.sin_addr.s_addr);
		printf("client-port: %d\r\n", sock_data_tmp.addr.sin_port);
		if(BC_OK != CheckServAddr(INADDR_ANY)) {
			// BC_WifiSetServ(WIFI_SERVER_OPEN, BC_CENTER_SERV_PORT, NULL);
			// BC_WifiSetNet(wifi_ssid_test, wifi_pwd_test, NULL);
		} else {
			printf("CheckServAddr OK\r\n");
			break;
		}
	}
	printf("wifi_id:%d\r\n", sock_data_tmp.wifi_id);
	if(!ASSERT_SOCK_VALID(sock_data_tmp.wifi_id)) {
		return -4;
	}
	wifi_id = sock_data_tmp.wifi_id;
	sock_data[wifi_id].valid = BC_TRUE;
	memcpy(cliaddr, &(sock_data_tmp.addr), sizeof(BC_Sockaddr));
	memcpy(&(sock_data[wifi_id].addr), &(sock_data_tmp.addr), sizeof(BC_Sockaddr));

	return wifi_id;
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
	sint32_t ret = BC_OK;
	uint32_t fail_count = 0;

	if(!ASSERT_SOCK_VALID(sockfd)) {
		return -1;
	}
	if(!sock_data[sockfd].valid) {
		return -2;
	}
	while(1) {
		// check max failed count
		if(++fail_count > WIFI_SOCK_CLOSE_MAX_FAIL_COUNT) {
			ret = -10;
			break;
		}
		sprintf(pu8CmdMsgClose, WifiCmdCloseSock, sockfd);
		uputs(USART_WIFI, pu8CmdMsgClose);
		vTaskDelay(10 / portTICK_RATE_MS); // at least delay 10ms
		// TODO: 
	}
	// sprintf(pu8CmdMsg, "AT+CIPCLOSE=%d\r\n", sockfd);
	// uputs(USART_WIFI, pu8CmdMsg);
	// sock_data[sockfd].valid = BC_FALSE;
	return BC_OK;
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

sint32_t SocketInit(void)
{
	uint32_t i;

	for(i = 0; i < BC_MAX_SOCKET_NUM; i++) {
		memset(&sock_data[i], 0, sizeof(BC_SocketData));
		sock_data[i].queue_handle = xQueueCreate(SOCK_DATA_QUE_NUM, sizeof(BC_SocketData));
		if(NULL == sock_data[i].queue_handle) {
			return BC_ERR;
		}
	}
	// BC_MAX_SOCKET_NUM == 5;
	sock_data[0].buf = sock_buf_0;
	sock_data[1].buf = sock_buf_1;
	sock_data[2].buf = sock_buf_2;
	sock_data[3].buf = sock_buf_3;
	sock_data[4].buf = sock_buf_4;

	// set sock_data[0] to be the server socket
	memset(&sock_serv, 0, sizeof(BC_SocketData));
	sock_serv.queue_handle = xQueueCreate(BC_MAX_SOCKET_NUM, sizeof(BC_SocketData));
	if(NULL == sock_serv.queue_handle) {
		return BC_ERR;
	}

	return BC_OK;
}

// sint32_t SockQueueInit(void)
// {
// 	sint32_t result = BC_OK;
// 	uint32_t i = 0;
// 
// 	for(i = 0; i < BC_MAX_SOCKET_NUM; i++) {
// 		sock_queue[i] = xQueueCreate(1, sizeof(BC_SocketData));
// 		if(NULL == sock_queue[i]) {
// 			return BC_ERR;
// 		}
// 		sock_queue[i].queue_handle = sock_queue[i];
// 	}
// 
// 	return result;
// }

sint32_t CheckServAddr(uint8_t * addr)
{
	if(!addr) {
		return -1;
	}
	// assume min addr len: 1.1.1.1
	// strlen("1.1.1.1") == 7
	if(GetWifiServerInfo() != WIFI_SERVER_OPEN) {
		return -3;
	}
	if(strlen(addr) < 7) {
		return -2;
	}
	return BC_OK;
}


