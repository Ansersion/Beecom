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
#include <queue.h>
#include <task.h>

// STM32 headers
#include <stm32f10x_gpio.h>

// Beecom headers
#include <bc_msg.h>
#include <terminal.h>
#include <bc_queue.h>
#include <panic.h>
#include <utils.h>
#include <wifi_clbk.h>

extern TaskHandle_t DataHubHandle;

#define LED_RED_TURN() (GPIOA->ODR ^= 1<<8) // red
#define LED_GREEN_TURN() (GPIOD->ODR ^= 1<<2) // green

#define BC_MOD_MYSELF BC_MOD_TERMINAL

static uint8_t UsartTermBuf[USART_TERMINAL_BUF_SIZE];
static uint8_t EndFlag[] = "\r\n";
static uint32_t FlagSize = sizeof(EndFlag) - 1;
// static BC_QueueElement QueEle;
// static BC_QueueElement QueEleForIrq;
static uint32_t GotMsgFlag = BC_FALSE;
static QueueHandle_t UsartMsgQueue;

sint32_t TaskTerminalInit(void)
{
	UsartMsgQueue = NULL;
	memset(UsartTermBuf, 0, sizeof(UsartTermBuf));
	GotMsgFlag = BC_FALSE;
	UsartMsgQueue = xQueueCreate(1, sizeof(uint32_t));
	if(!UsartMsgQueue) {
		return BC_ERR;
	}
	return BC_OK;
}

volatile void IrqUsartTerminal(void)
// volatile void vUARTInterruptHandler(void)
{
	static uint32_t Index = 0;
	static uint16_t RxData=0;
	BaseType_t xHigherPriorityTaskWoken;

	if(USART_GetITStatus(USART_TERMINAL, USART_IT_RXNE) == RESET) {
		return;
	}
	RxData = USART_RECEIVE(USART_TERMINAL, RxData);
	UsartTermBuf[Index++] = (uint8_t)RxData;

	if(Index >= sizeof(UsartTermBuf) - 1) {
		UsartTermBuf[USART_TERMINAL_BUF_SIZE-1] = '\0';
		Index = 0;
		return;
	}
	// if(BC_OK == CheckEndFlag(UsartTermBuf, Index, EndFlag, FlagSize)) {
	if(BC_OK == CheckDataFlag(UsartTermBuf, Index, EndFlag, FlagSize, BC_TRUE)) {
		xQueueSendFromISR(UsartMsgQueue, &GotMsgFlag, &xHigherPriorityTaskWoken);
		UsartTermBuf[Index] = '\0';
		Index = 0;
	}
}

void TaskTerminal(void * pvParameters)
{
	static BC_QueueElement qe;
	static uint8_t * msg = UsartTermBuf;

	stWifiMsgUnit testWifiMsgUnit;

	if(TaskTerminalInit() != BC_OK) {
		BC_Panic("Ternimal Init");
	}

	while(1) {
		while(BC_FALSE == xQueueReceive(UsartMsgQueue, &GotMsgFlag, TIMEOUT_COMMON)) {
			// Indicate led
			LED_RED_TURN();
		}
		// process
		BC_MsgInit(&qe, BC_MOD_MYSELF, BC_MOD_WIFI);
		switch(msg[0]) {
			case 'R':
				testWifiMsgUnit.WifiClbkCmd = WIFI_CLBK_CMD_RESET;
				break;
			case 'M':
				testWifiMsgUnit.WifiClbkCmd = WIFI_CLBK_CMD_SET_MODE;
				testWifiMsgUnit.ClbkPara.SetModePara.Mode = WIFI_MODE_STA;
				break;
			case 'U':
				testWifiMsgUnit.WifiClbkCmd = WIFI_CLBK_CMD_SET_MUX;
				testWifiMsgUnit.ClbkPara.SetMuxPara.Mux = WIFI_MUX_OPEN;
				break;
			case 'I':
				testWifiMsgUnit.WifiClbkCmd = WIFI_CLBK_CMD_QRY_SR;
				break;
			case 'S':
				testWifiMsgUnit.WifiClbkCmd = WIFI_CLBK_CMD_SET_SERV;
				testWifiMsgUnit.ClbkPara.ServPara.ServMode = WIFI_SERVER_OPEN;
				testWifiMsgUnit.ClbkPara.ServPara.Port = BC_CENTER_SERV_PORT;
				break;
			default:
				continue;
				break;
		}
		BC_MsgSetMsg(&qe, (uint8_t *)&testWifiMsgUnit, sizeof(testWifiMsgUnit));
		while(BC_Enqueue(BC_ModInQueue[BC_MOD_MYSELF], &qe, TIMEOUT_COMMON) == BC_FALSE) {
		}
		vTaskResume(DataHubHandle);
		if(BC_Dequeue(BC_ModOutQueue[BC_MOD_MYSELF], &qe, 0) == BC_TRUE) {
			// process 
		}
	}
}

// sint32_t CheckEndFlag(uint8_t * msg, uint32_t msg_size, uint8_t * flag, uint32_t flag_size)
// {
// 
// 	static sint32_t i = 0;
// 	if(msg_size < flag_size) {
// 		return BC_ERR;
// 	}
// 	if(!msg) {
// 		return BC_ERR;
// 	}
// 	if(!flag) {
// 		return BC_ERR;
// 	}
// 	for(i = 0; i < flag_size; i++) {
// 		if(msg[msg_size - flag_size + i] != flag[i]) {
// 			return BC_ERR;
// 		}
// 	}
// 	return BC_OK;
// }

sint32_t uputs(USART_TypeDef * usart, sint8_t * str)
{
	if(!usart || !str) {
		return -1;
	}
	if('\0' == *str) {
		return 0;
	}
	do {
		USART_SendData(usart,*str++);
		while(USART_GetFlagStatus(usart,USART_FLAG_TC)!=SET);
	}while('\0' != *str);

	return 0;
}

#if 1
#pragma import(__use_no_semihosting)             
struct __FILE 
{ 
	int handle; 
	/* Whatever you require here. If the only file you are using is */ 
	/* standard output using printf() for debugging, no file handling */ 
	/* is required. */ 
}; 
/* FILE is typedef�� d in stdio.h. */ 
FILE __stdout;       
void _sys_exit(int x) 
{ 
	x = x; 
} 

int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);
	USART_SEND(USART1, ch);
	return ch;
}

#endif
