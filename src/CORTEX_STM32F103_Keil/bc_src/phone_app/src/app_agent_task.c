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
#include <app_agent_common.h>

#include <terminal.h>
#include <mutex.h>
#include <panic.h>
#include <bc_queue.h>

#include <app_agent_task.h>
#include <app_agent_common.h>

extern TaskHandle_t DataHubHandle;

#define LED_GREEN_TURN() (GPIOD->ODR ^= 1<<2) // green

#define BC_MOD_MYSELF BC_MOD_PHONE_APP

static uint8_t UsartWifiBuf[USART_WIFI_BUF_SIZE];
static QueueHandle_t UsartMsgQueue;
static uint32_t GotMsgFlag = BC_FALSE;
static BC_Mutex mutex;

void TaskAppAgent(void *pvParameters)
{
	static BC_QueueElement qe;
	static uint8_t * msg = UsartWifiBuf;

	if(TaskAppAgentInit() != BC_OK) {
		BC_Panic("AppAgent Init");
	}

	while(1) {
		while(BC_FALSE == xQueueReceive(UsartMsgQueue, &GotMsgFlag, TIMEOUT_COMMON)) {
			// Indicate led
			LED_GREEN_TURN();
		}
		// BC_MsgInit(&qe, BC_MOD_MYSELF, BC_MOD_DEFAULT);
		// // BC_MsgSetMsg(&qe, UsartTermBuf, strlen(UsartTermBuf));
		// BC_MsgSetMsg(&qe, msg, strlen((const char *)msg));
		ProcWifiMsg(&qe, msg);
		while(BC_Enqueue(BC_ModInQueue[BC_MOD_MYSELF], &qe, TIMEOUT_COMMON) == BC_FALSE) {
		}
		vTaskResume(DataHubHandle);
		if(BC_Dequeue(BC_ModOutQueue[BC_MOD_MYSELF], &qe, 0) == BC_TRUE) {
			// process 
		}
	}
}

sint32_t TaskAppAgentInit(void)
{
	UsartMsgQueue = NULL;
	memset(UsartWifiBuf, 0, sizeof(UsartWifiBuf));
	GotMsgFlag = BC_FALSE;
	UsartMsgQueue = xQueueCreate(1, sizeof(GotMsgFlag));

	if(BCMutexInit(&mutex) != BC_OK) {
		return BC_ERR;
	}
	if(!UsartMsgQueue) {
		return BC_ERR;
	}
	return BC_OK;
}

sint32_t ProcWifiMsg(BC_QueueElement * qe, uint8_t * wifi_msg)
{
	return BC_OK;
}

// volatile void IrqUsartWifi( void )
// {
// }
// 
