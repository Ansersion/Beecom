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
#include <bc_msg.h>

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

uint8_t sock_buf_serv[BC_MAX_SOCKET_BUF_SIZE];

static sint8_t pu8CmdMsg[128];
// static sint8_t pu8CmdMsgClose[64];
// static sint8_t pu8CmdMsgOpen[128];
// static sint8_t pu8CmdMsgConnect[128];

static const sint8_t * WifiCmdReset = "AT+RST\r\n";
static const sint8_t * WifiCmdSetMode = "AT+CWMODE=%d\r\n";
static const sint8_t * WifiCmdSetNet = "AT+CWJAP=\"%s\",\"%s\"\r\n";
static const sint8_t * WifiCmdSetMux = "AT+CIPMUX=%d\r\n";
static const sint8_t * WifiCmdSetServ = "AT+CIPSERVER=%d,%d\r\n";
static const sint8_t * WifiCmdStatus = "AT+CIPSTATUS\r\n";
static const sint8_t * WifiCmdCloseSock = "AT+CIPCLOSE=%d\r\n";
static const sint8_t * WifiCmdSend = "AT+CIPSEND=%d,%d\r\n";

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
		if(++fail_count > WIFI_CIF_ST_MAX_FAIL_COUNT) {
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

sint32_t BC_WifiCloseSock(sint32_t sockfd, uint32_t * timeout)
{
	sint32_t ret = BC_OK;
	uint32_t fail_count = 0;
	sint32_t already_sent = 0;
	printf("Close sock\r\n");


	while(1) {
		// check max failed count
		if(++fail_count > WIFI_CIP_CLS_MAX_FAIL_COUNT) {
			ret = -10;
			break;
		}
		// send command
		if(!already_sent) {
			already_sent = 1;
			sprintf(pu8CmdMsg, WifiCmdCloseSock, sockfd);
			printf("Close sock: end uputs1\r\n");
			printf("WifiRecvFlag1:%x\r\n", WifiRecvFlag);
			uputs(USART_WIFI, pu8CmdMsg);
			printf("WifiRecvFlag2:%x\r\n", WifiRecvFlag);
		}
		printf("WifiRecvFlag3:%x\r\n", WifiRecvFlag);
		printf("Close sock: end uputs2\r\n");
		// delay a little time(at least) 
		// delay 'timeout' microseconds if any
		// printf("Close sock: end uputs\r\n");
		vTaskDelay(1000 / portTICK_RATE_MS); // at least delay 10ms
		if(timeout) {
			vTaskDelay(*timeout / portTICK_RATE_MS);
		}
		if(BCMutexLock(&WifiRecvFlagMutex) == BC_OK) {
			// check if setting OK
			printf("lock ok\r\n");
			if(WifiRecvFlag & WIFI_MSG_FLAG_GENERAL_OK) {
				WifiRecvFlag &= ~WIFI_MSG_FLAG_GENERAL_OK;
				BCMutexUnlock(&WifiRecvFlagMutex);
				ret = BC_OK;
				printf("Close sock ok\r\n");
				break;
			// check if setting error
			} else if(WifiRecvFlag & WIFI_MSG_FLAG_GENERAL_ERR) {
				WifiRecvFlag &= ~WIFI_MSG_FLAG_GENERAL_ERR;
				BCMutexUnlock(&WifiRecvFlagMutex);
				printf("Close sock err\r\n");
				ret = -4;
				break;
			}
			BCMutexUnlock(&WifiRecvFlagMutex);
		}
		else
		{
			printf("lock err\r\n");
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

sint32_t BC_WifiSend(sint32_t sockfd, uint8_t * msg, uint32_t size, uint32_t * timeout)
{
	sint32_t ret = BC_OK;
	uint32_t fail_count = 0;

	if(!msg)
	{
		return -2;
	}
	if(!ASSERT_SOCK_VALID(sockfd))
	{
		return -3;
	}

	printf("send2:%p\r\n", msg);
	printf("send:%s, %d\r\n", msg, size);

	while(1) {
		// check max failed count
		if(++fail_count > WIFI_CIP_SEND_MAX_FAIL_COUNT) {
			ret = -10;
			break;
		}
		// send command
		printf("start send:%d, %d\r\n", sockfd, size);
		sprintf(pu8CmdMsg, WifiCmdSend, sockfd, size);
		printf("Send-WifiRecvFlag1:%x\r\n", WifiRecvFlag);
		uputs(USART_WIFI, pu8CmdMsg);
		// delay a little time(at least) 
		vTaskDelay(1000 / portTICK_RATE_MS); // at least delay 10ms
		printf("Send-WifiRecvFlag2:%x\r\n", WifiRecvFlag);
		uputn(USART_WIFI, msg, size);
		vTaskDelay(1000 / portTICK_RATE_MS); // at least delay 10ms
		printf("Send-WifiRecvFlag3:%x\r\n", WifiRecvFlag);
		// printf("sending end\r\n");
		// sprintf(pu8CmdMsg, WifiCmdSend, sockfd, size);
		// delay 'timeout' microseconds if any
		if(timeout) {
			vTaskDelay(*timeout / portTICK_RATE_MS);
		}
		if(BCMutexLock(&WifiRecvFlagMutex) == BC_OK) {
			// check if setting OK
			if(WifiRecvFlag & WIFI_MSG_FLAG_GENERAL_OK) {
				WifiRecvFlag &= ~WIFI_MSG_FLAG_GENERAL_OK;
				BCMutexUnlock(&WifiRecvFlagMutex);
				printf("sending ok\r\n");
				ret = BC_OK;
				break;
			// check if setting error
			} else if(WifiRecvFlag & WIFI_MSG_FLAG_GENERAL_ERR) {
				WifiRecvFlag &= ~WIFI_MSG_FLAG_GENERAL_ERR;
				printf("sending err\r\n");
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
	const BC_SocketData * SockServ = NULL;
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
	SockServ = GetSockData(SOCK_SERV_FD);
	if(!SockServ) {
		return -11;
	}
	// if(BC_TRUE != sock_data[sockfd].valid) {
	// 	return -4;
	// }
	// wait for the semphore

	// k_cliaddr = cliaddr;
	// k_addrlen = addrlen;

	while(1) {
		if(pdFALSE == xQueueReceive(SockServ->queue_handle, &sock_data_tmp, 5000/portTICK_RATE_MS)) {
			BC_Printf(BC_MOD_WIFI, "addr7: %s", INADDR_ANY);
			// printf("addr7: %s\r\n", INADDR_ANY);
			continue;
		}
		if(!(sock_data[sock_data_tmp.wifi_id].wifi_recv_flag & WIFI_MSG_FLAG_GOT_CONNECT)) {
			printf("!WIFI_MSG_FLAG_GOT_CONNECT\r\n");
			continue;
		}

		printf("wifi_id: %d\t%x\t%d\r\n", sock_data_tmp.wifi_id, sock_data[sock_data_tmp.wifi_id].wifi_recv_flag, sock_data_tmp.valid);

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
					sock_data_tmp.wifi_id = BC_MAX_SOCKET_NUM+1;
					break;
				}
				printf("try again(addr2: %s)\r\n", INADDR_ANY);
			} else {
				break;
			}
		}
		printf("backlog-s: %d\r\n", SockServ->backlog);
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
	// sint32_t ret = BC_OK;
	uint32_t fail_count = 0;
	BC_QueueElement qe;
	stWifiMsgUnit WifiMsgUnit;
	BC_SocketData sock_data_tmp;
	const BC_SocketData * SockServ = NULL;

	if(!ASSERT_SOCK_VALID(sockfd)) {
		return -1;
	}
	if(!sock_data[sockfd].valid) {
		return -2;
	}
	SockServ = GetSockData(SOCK_SERV_FD);
	if(!SockServ) {
		return -11;
	}
	BC_MsgInit(&qe, BC_MOD_DEFAULT, BC_MOD_WIFI);
	WifiMsgUnit.WifiClbkCmd = WIFI_CLBK_CMD_CLOSE_SOCK;
	WifiMsgUnit.ClbkPara.ClsSockPara.sockfd = sockfd;
	BC_MsgSetMsg(&qe, (uint8_t *)&WifiMsgUnit, sizeof(WifiMsgUnit));
	while(1) {
		while(BC_Enqueue(BC_ModInQueue[BC_MOD_DEFAULT], &qe, 3*TIMEOUT_COMMON) == BC_FALSE) {
		}
		vTaskResume(DataHubHandle);

		printf("close enqueue == true\r\n");
		// if(pdFALSE == xQueueReceive(sock_data[sockfd].queue_handle, &sock_data_tmp, 5000/portTICK_RATE_MS)) {
		while(pdFALSE == xQueueReceive(sock_data[sockfd].queue_handle, &sock_data_tmp, 1500/portTICK_RATE_MS)) {
			fail_count++;
			if(fail_count > 3) {
				// sock_data_tmp.wifi_id = BC_MAX_SOCKET_NUM+1;
				// break;
				printf("close fail\r\n");
				sock_data[sockfd].valid = BC_FALSE;
				return BC_ERR;
			}
			printf("close:serv.wifi_id_1=%d\r\n", SockServ->wifi_id);
		// } else {
		// 	break;
		}
	vTaskDelay(500/portTICK_RATE_MS);
	printf("close:serv.wifi_id_2=%d\r\n", SockServ->wifi_id);
	break;
	}
	// sprintf(pu8CmdMsg, "AT+CIPCLOSE=%d\r\n", sockfd);
	// uputs(USART_WIFI, pu8CmdMsg);
	sock_data[sockfd].valid = BC_FALSE;
	return BC_OK;
}

sint32_t BC_Recv(sint32_t sockfd, void * buff, uint32_t nbytes, sint32_t flags)
{
	BC_SocketData sock_data_tmp;
	const BC_SocketData * SockServ = NULL;

	if(!ASSERT_SOCK_VALID(sockfd)) {
		return -1;
	}
	if(!buff) {
		return -2;
	}
	if(!sock_data[sockfd].valid) {
		return -3;
	}
	SockServ = GetSockData(SOCK_SERV_FD);
	if(!SockServ) {
		return -11;
	}
	sock_data_tmp.wifi_id = -1;
	while(BC_FALSE == xQueueReceive(sock_data[sockfd].queue_handle, &sock_data_tmp, 1000/portTICK_RATE_MS)) {
		// TODO:
		printf("BC_Recv: xQueueReceive == pdFALSE:%d\r\n", sockfd);
		printf("BC_Recv: wifi_id:%d\r\n", SockServ->wifi_id);
		printf("BC_Recv: ipd_size:%d\r\n", SockServ->ipd_size);
		// Judge if remote close
	}
	printf("BC_Recv: xQueueReceive == pdTRUE\r\n");
	printf("BC_Recv: wifi_id(serv):%d\r\n", SockServ->wifi_id);
	printf("BC_Recv: ipd_size(serv):%d\r\n", SockServ->ipd_size);
	printf("BC_Recv: wifi_id:%d\r\n", sock_data_tmp.wifi_id);
	printf("BC_Recv: ipd_size:%d\r\n", sock_data_tmp.ipd_size);
	sock_data[sock_data_tmp.wifi_id].buf[sock_data_tmp.ipd_size] = '\0';
	printf("BC_Recv: str:%s\r\n", sock_data[sock_data_tmp.wifi_id].buf);
	SockServ->buf[SockServ->ipd_size] = '\0';
	printf("BC_Recv: str(serv):%s\r\n", SockServ->buf);
	// if(sock_data[sockfd].wifi_recv_flag | WIFI_MSG_FLAG_GOT_CONNECT | WIFI_MSG_FLAG_GOT_CLOSED) {
	// if(sock_data[sockfd].wifi_recv_flag | WIFI_MSG_FLAG_GOT_CLOSED) {
	// 	return -4;
	// }
	if(!sock_data[sockfd].valid) {
		return -5;
	}
	if(sock_data_tmp.ipd_size > nbytes) {
		return -6;
	}
	memcpy(buff, sock_data[sock_data_tmp.wifi_id].buf, sock_data_tmp.ipd_size+1);
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

sint32_t BC_Send(sint32_t sockfd, void * buff, uint32_t nbytes, sint32_t flags)
{
	static sint32_t i = 0;
	static uint8_t * buf_tmp = NULL;
	BC_QueueElement qe;
	stWifiMsgUnit WifiMsgUnit;
	BC_SocketData sock_data_tmp;
	uint32_t fail_count = 0;

	if(!ASSERT_SOCK_VALID(sockfd)) {
		return -1;
	}
	if(!buff) {
		return -2;
	}
	if(!sock_data[sockfd].valid) {
		return -3;
	}

	BC_MsgInit(&qe, BC_MOD_DEFAULT, BC_MOD_WIFI);
	WifiMsgUnit.WifiClbkCmd = WIFI_CLBK_CMD_SEND;
	WifiMsgUnit.ClbkPara.SendPara.sockfd = sockfd;
	WifiMsgUnit.ClbkPara.SendPara.msg = (uint8_t *)buff;
	printf("send_msg_addr:%p\r\n", WifiMsgUnit.ClbkPara.SendPara.msg);
	WifiMsgUnit.ClbkPara.SendPara.size = nbytes;
	BC_MsgSetMsg(&qe, (uint8_t *)&WifiMsgUnit, sizeof(WifiMsgUnit));
	while(1) {
		while(BC_Enqueue(BC_ModInQueue[BC_MOD_DEFAULT], &qe, TIMEOUT_COMMON) == BC_FALSE) {
		}
		vTaskResume(DataHubHandle);

		if(pdFALSE == xQueueReceive(sock_data[sockfd].queue_handle, &sock_data_tmp, 5000/portTICK_RATE_MS)) {
		// if(pdFALSE == xQueueReceive(SockServ->queue_handle, &sock_data_tmp, 1000/portTICK_RATE_MS)) {
			fail_count++;
			printf("send wifi id1: %d\r\n", sock_data_tmp.wifi_id);
			if(fail_count > 3) {
				// sock_data_tmp.wifi_id = BC_MAX_SOCKET_NUM+1;
				// break;
				printf("send fail\r\n");
				sock_data[sockfd].valid = BC_FALSE;
				return -4;
			}
		} else {
			// vTaskDelay(100);
			printf("send wifi id2: %d\r\n", sock_data_tmp.wifi_id);
			// TODO: Check Send OK
			printf("send ok\r\n");
			break;
		}
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
	sock_serv.buf = sock_buf_serv;

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
	// if(GetWifiServerInfo() != WIFI_SERVER_OPEN) {
	// 	return -3;
	// }
	if(strlen(addr) < 7) {
		return -2;
	}
	return BC_OK;
}


