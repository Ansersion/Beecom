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

#include <wifi_task.h>
#include <wifi_irq.h>
#include <wifi_common.h>
#include <wifi_clbk.h>

extern TaskHandle_t DataHubHandle;

#define LED_GREEN_TURN() (GPIOD->ODR ^= 1<<2) // green

#define BC_MOD_MYSELF BC_MOD_WIFI

// static uint8_t UsartWifiBuf[USART_WIFI_BUF_SIZE];
// static QueueHandle_t UsartMsgQueue;
// static uint32_t GotMsgFlag = BC_FALSE;
// static BC_Mutex mutex;
uint8_t WifiPanicMsg[64];

void TaskWifiAgent(void *pvParameters)
{
	static BC_QueueElement qe;
	static uint8_t * wifi_msg = UsartWifiBuf;
	static sint32_t ret = BC_OK;

	ret = TaskWifiAgentInit();
	if(ret != BC_OK) {
		sprintf(WifiPanicMsg, "WifiAgent Init: ErrCode=%d", ret);
		/* Never return */
		BC_Panic(WifiPanicMsg);
	}

	while(1) {
		while(BC_Dequeue(BC_ModOutQueue[BC_MOD_MYSELF], &qe, TIMEOUT_COMMON) == BC_FALSE) {
			// Indicate led
			LED_GREEN_TURN();
		}
		// BC_MsgInit(&qe, BC_MOD_MYSELF, BC_MOD_DEFAULT);
		// // BC_MsgSetMsg(&qe, UsartTermBuf, strlen(UsartTermBuf));
		// BC_MsgSetMsg(&qe, msg, strlen((const char *)msg));
		ret = ProcWifiMsg(&qe, wifi_msg);
		printf("wifi task: ret=%d\r\n%s\r\n", ret, wifi_msg);
		if(BC_MOD_IRQ != qe.u8SrcID) {
			continue;
		}
		while(BC_Enqueue(BC_ModInQueue[BC_MOD_MYSELF], &qe, TIMEOUT_COMMON) == BC_FALSE) {
		}
		vTaskResume(DataHubHandle);
		// if(BC_Dequeue(BC_ModOutQueue[BC_MOD_MYSELF], &qe, 0) == BC_TRUE) {
		// 	// process 
		// }
	}
}

sint32_t TaskWifiAgentInit(void)
{
	// UsartMsgQueue = NULL;
	memset(UsartWifiBuf, 0, USART_WIFI_BUF_SIZE);
	// GotMsgFlag = BC_FALSE;
	// UsartMsgQueue = xQueueCreate(1, sizeof(GotMsgFlag));

	if(BCMutexInit(&WifiRecvFlagMutex) != BC_OK) {
		return -1;
	}
	if(SocketInit() != BC_OK) {
		return -2;
	}
	if(CheckWifiMsgUnit() != BC_OK) {
		return -3;
	}
	return BC_OK;
}

sint32_t ProcWifiMsg(BC_QueueElement * qe, uint8_t * wifi_msg)
{
	stWifiMsgUnit * msg_unit = NULL;
	sint32_t ret = BC_OK;

	if(!qe) {
		return -1;
	}
	if(!wifi_msg) {
		return -2;
	}

	if(BC_MOD_IRQ != qe->u8SrcID) {
		// msg from datahub
		if(qe->u16MsgLen != sizeof(stWifiMsgUnit)) {
			return -11;
		}
		msg_unit = (stWifiMsgUnit *)(qe->pText);
		switch(msg_unit->WifiClbkCmd) {
			case WIFI_CLBK_CMD_RESET:
				ret = BC_WifiReset(NULL);
				break;
			case WIFI_CLBK_CMD_SET_MODE:
				ret = BC_WifiSetMode(msg_unit->ClbkPara.SetModePara.Mode, NULL);
				break;
			case WIFI_CLBK_CMD_SET_NET:
				ret = BC_WifiSetNet(msg_unit->ClbkPara.SetNetPara.Ssid, msg_unit->ClbkPara.SetNetPara.Pwd, NULL);
				break;
			case WIFI_CLBK_CMD_SET_MUX:
				ret = BC_WifiSetMux(msg_unit->ClbkPara.SetMuxPara.Mux, NULL);
				break;
			case WIFI_CLBK_CMD_SET_SERV:
				ret = BC_WifiSetServ(msg_unit->ClbkPara.ServPara.ServMode, msg_unit->ClbkPara.ServPara.Port, NULL);
				break;
			default:
				break;
		}
	} else {
		// msg from irq
	}
	return ret;
}

sint32_t BC_Atoi(char n)
{
		return n - '0';
}

// volatile void IrqUsartWifi( void )
// {
// }
// 
