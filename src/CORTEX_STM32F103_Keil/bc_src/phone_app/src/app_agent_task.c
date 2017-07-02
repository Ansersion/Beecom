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

// STM32 headers
#include <stm32f10x_gpio.h>

// Beecom headers
#include <bc_type.h>

#include <terminal.h>
#include <mutex.h>
#include <panic.h>
#include <bc_queue.h>
#include <wifi_common.h>

#include <app_agent_task.h>
#include <app_agent_common.h>

extern TaskHandle_t DataHubHandle;
extern BC_SocketData sock_serv;

#define LED_GREEN_TURN() (GPIOD->ODR ^= 1<<2) // green

#define BC_MOD_MYSELF BC_MOD_PHONE_APP

uint8_t AppPanicMsg[64];
uint8_t SrcBuf[128];
uint8_t DstBuf[128];

void TaskAppAgent(void *pvParameters)
{
	static sint32_t ret = BC_OK;
	BC_Sockaddr server_addr;
	BC_Sockaddr client_addr;
	uint32_t addr_len = sizeof(BC_Sockaddr);
	sint32_t server_socket = 0;
	sint32_t client_socket;
	sint32_t trans_len = 0;

	ret = TaskAppAgentInit();

	if(ret != BC_OK) {
		sprintf(AppPanicMsg, "AppAgent Init: ErrCode=%d", ret);

		/* Never return */
		BC_Panic(AppPanicMsg);
	}

	memset(&server_addr, 0, sizeof(BC_Sockaddr));
	server_addr.sin_family = AF_INET;
	// diff: string not long
	memcpy(server_addr.sin_addr.s_addr, INADDR_ANY, sizeof(INADDR_ANY));
	// // diff: host order not net order
	server_addr.sin_port = BC_CENTER_SERV_PORT;

	// server_socket = BC_Socket(AF_INET, SOCK_STREAM, 0);
	/* The socket of no use*/
	server_socket = 1234; 

	ret = BC_Bind(server_socket,&server_addr,sizeof(server_addr));
	if(ret != BC_OK) {
		sprintf(AppPanicMsg, "BC_Bind: ErrCode=%d", ret);
		BC_Panic(AppPanicMsg);
	}
	ret = BC_Listen(server_socket, LISTEN_QUEUE);
	if (ret != BC_OK) {
		sprintf(AppPanicMsg, "BC_Listen: ErrCode=%d", ret);
		BC_Panic(AppPanicMsg);
	}

	while(BC_TRUE) {
		memset(&client_addr, 0, sizeof(BC_Sockaddr));
		// memset(SrcBuf, 0, TASK_BUF_SIZE);
		// memset(DstBuf, 0, TASK_BUF_SIZE);
		client_socket = BC_Accept(server_socket, &client_addr,&addr_len);
		// WifiRecvFlag &= ~WIFI_MSG_FLAG_GOT_CLI;
		if (client_socket < 0) {
			printf("Server Accept Failed:%d\r\n", client_socket);
			// break;
			continue;
		} 
		// else {
		// 	printf("cli_sockfd:%d\r\n", client_socket);
		// 	printf("cli_addr:%s\r\n", client_addr.sin_addr.s_addr);
		// 	printf("cli_port:%d\r\n", client_addr.sin_port);
		// }
		trans_len = BC_Recv(client_socket, SrcBuf, sizeof(SrcBuf), 0);
		if(trans_len <= 0) {
			printf("Server Recv Failed: %d\r\n", trans_len);
			BC_Close(client_socket);
			continue;
		}
		// printf("Recv from %s:%d\r\n", client_addr.sin_addr.s_addr, client_addr.sin_port);
		// printf("Msg: #%s#\r\n", SrcBuf);
		// printf("Echoing now...\r\n");
		sprintf(DstBuf, "Echo: %s", SrcBuf);
		trans_len = strlen(DstBuf);
		trans_len = BC_Send(client_socket, DstBuf, trans_len, 0);
		if(trans_len < 0) {
			printf("Server Send Failed:%d\r\n", trans_len);
			BC_Close(client_socket);
			continue;
		}
		// vTaskDelay(1000/portTICK_RATE_MS);
		ret = BC_Close(client_socket);
		// printf("BC_Close:%d\r\n", ret);
	}

	return;
}

sint32_t TaskAppAgentInit(void)
{
	return BC_OK;
}
