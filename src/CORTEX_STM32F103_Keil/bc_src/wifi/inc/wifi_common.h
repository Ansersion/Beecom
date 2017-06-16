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

#ifndef WIFI_COMMON_H
#define WIFI_COMMON_H

// FreeRTOS headers
#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

// Beecom headers
#include <bc_type.h>

#define BC_MAX_SOCKET_NUM 			5
#define BC_MAX_SOCKET_BUF_SIZE 		256
#define BC_MAX_WIFI_FAIL_COUNT 		5

/************************
  Only "1" now is supported
*************************/
#define SOCK_DATA_QUE_NUM 			1

#define AF_INET 	0

#define SOCK_STREAM 	0
#define SOCK_DGRAM 		1

#define LISTEN_QUEUE 	8

#define ASSERT_SOCK_VALID(s) ((s) >= 0 && (s) < BC_MAX_SOCKET_NUM)

#define WIFI_RESET_MAX_FAIL_COUNT 				3
#define WIFI_SET_MODE_MAX_FAIL_COUNT 			3
#define WIFI_SET_NET 							3
#define WIFI_SET_MUX_MAX_FAIL_COUNT 			3
#define WIFI_SET_SERV_MAX_FAIL_COUNT 			3
#define WIFI_GET_STATUS_MAX_FAIL_COUNT 			3
#define WIFI_SOCK_CLOSE_MAX_FAIL_COUNT 			3
#define WIFI_CIF_SR_MAX_FAIL_COUNT 				3
#define WIFI_CIF_ST_MAX_FAIL_COUNT 				3

extern uint8_t INADDR_ANY[16];

typedef struct in_addr {
	uint8_t s_addr[16];
}in_addr;

typedef struct BC_Sockaddr {
	sint16_t sin_family;                      /* Address family */
	uint16_t sin_port;       /* Port number */
	in_addr sin_addr;              /* Internet address */
	uint8_t sin_zero[8];         /* Same size as struct sockaddr */
}BC_Sockaddr;

typedef struct BC_SocketData {
	// uint32_t msg_flag;
	BC_Sockaddr addr;
	bool_t valid;
	// bool_t is_server;
	// bool_t accepted;
	uint32_t wifi_recv_flag;
	uint8_t * buf;
	sint32_t backlog;
	QueueHandle_t queue_handle;
	sint32_t wifi_id;
	sint32_t ipd_size;
}BC_SocketData;

typedef enum WIFI_MODE {
	WIFI_MODE_STA = 1,
	WIFI_MODE_AP, 
	WIFI_MODE_AP_STA, 
	WIFI_MODE_INVALID,
}WIFI_MODE;

typedef enum WIFI_MUX {
	WIFI_MUX_CLOSE = 0,
	WIFI_MUX_OPEN,
	WIFI_MUX_INVALID,
}WIFI_MUX;

typedef enum WIFI_SERVER {
	WIFI_SERVER_CLOSE=0,
	WIFI_SERVER_OPEN,
	WIFI_SERVER_INVALID,
}WIFI_SERVER;

WIFI_MODE 	GetWifiModeInfo(void);
WIFI_SERVER 	GetWifiServerInfo(void);
sint32_t BC_WifiReset(uint32_t * timeout);
sint32_t BC_WifiSetMode(WIFI_MODE mode, uint32_t * timeout);
sint32_t BC_WifiSetNet(uint8_t * ssid, uint8_t * pwd, uint32_t * timeout);
sint32_t BC_WifiSetMux(WIFI_MUX mux_mode, uint32_t * timeout);
sint32_t BC_WifiSetServ(WIFI_SERVER server_mode, uint16_t port, uint32_t * timeout);
sint32_t BC_WifiQuerySr(uint32_t * timeout);
sint32_t BC_WifiQuerySt(uint32_t * timeout);

// wifi APIs
// They are similar to linux socket
// just for using the linux net code procedure
// Author: Ansersion
// Date: 2017-03-14
sint32_t BC_Socket(sint32_t family, sint32_t type, sint32_t protocol);
sint32_t BC_Bind(sint32_t sockfd, const BC_Sockaddr * myaddr, uint32_t addrlen);
sint32_t BC_Listen(sint32_t sockfd, sint32_t backlog);
sint32_t BC_Accept(sint32_t sockfd, BC_Sockaddr * cliaddr, uint32_t * addrlen);
sint32_t BC_Connect(sint32_t sockfd, const BC_Sockaddr * servaddr, uint32_t addrlen);
sint32_t BC_Close(sint32_t sockfd);
sint32_t BC_Recv(sint32_t sockfd, void * buff, uint32_t nbytes, sint32_t flags);
sint32_t BC_Send(sint32_t sockfd, const void * buff, uint32_t nbytes, sint32_t flags);

sint32_t BC_Atoi(char n);

sint32_t SocketInit(void);
// /****************************
//   Must run after "SocketInit"
// *****************************/
// sint32_t SockQueueInit(void);
// extern BC_SocketData sock_serv;
sint32_t CheckServAddr(uint8_t * addr);

#endif

