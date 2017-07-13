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
#include <stdarg.h>

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
#include <terminal_clbk.h>

extern TaskHandle_t DataHubHandle;

#define PRINTF_BUF_SIZE 128
// static sint8_t BCPrinfBuf[128];

#define LED_RED_TURN() (GPIOA->ODR ^= 1<<8) // red
#define LED_GREEN_TURN() (GPIOD->ODR ^= 1<<2) // green


static const BC_ModID MOD_MYSELF = BC_MOD_TERMINAL;

static uint8_t UsartTermBuf[USART_TERMINAL_BUF_SIZE];
static uint8_t EndFlag[] = "\r\n";
static uint32_t FlagSize = sizeof(EndFlag) - 1;
// static BC_QueueElement QueEle;
// static BC_QueueElement QueEleForIrq;
static QueueHandle_t UsartMsgQueue;
static uint8_t WifiSsidBuf[64];
static uint8_t WifiPwdBuf[64];
// static bool_t g_bPrinting = BC_FALSE;

sint32_t TaskTerminalInit(void)
{
	UsartMsgQueue = NULL;
	memset(UsartTermBuf, 0, sizeof(UsartTermBuf));
	memset(WifiSsidBuf, 0, sizeof(WifiSsidBuf));
	strcpy(WifiSsidBuf, "404-hb2g");
	memset(WifiPwdBuf, 0, sizeof(WifiPwdBuf));
	strcpy(WifiPwdBuf, "68704824");
	UsartMsgQueue = xQueueCreate(1, sizeof(uint32_t));
	if(!UsartMsgQueue) {
		return BC_ERR;
	}
	return BC_OK;
}

volatile void IrqUsartTerminal(void)
{
	static uint32_t Index = 0;
	static uint16_t RxData=0;
	static BC_QueueElement qe;

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
	if(BC_OK == CheckDataFlag(UsartTermBuf, Index, EndFlag, FlagSize, BC_TRUE)) {
		BC_MsgInit(&qe, BC_MOD_IRQ, MOD_MYSELF);
		BC_MsgSetMsg(&qe, UsartTermBuf, Index);
		xQueueSendFromISR(BC_ModOutQueue[MOD_MYSELF], &qe, NULL);
		UsartTermBuf[Index] = '\0';
		Index = 0;
	}
}

void TaskTerminal(void * pvParameters)
{
	static BC_QueueElement qe;
	static uint8_t * msg = UsartTermBuf;
	static sint32_t ret = BC_ERR;

	stWifiMsgUnit testWifiMsgUnit;

	if(TaskTerminalInit() != BC_OK) {
		BC_Panic("Ternimal Init");
	}

	while(1) {
		while(BC_FALSE == BC_Dequeue(BC_ModOutQueue[MOD_MYSELF], &qe, TIMEOUT_COMMON)) {
		 	LED_RED_TURN();
		}

		ret = ProcTermMsg(&qe);
		/* Your control: START */
		/* Drop by myself*/
		if(BC_MOD_ANONYMITY == qe.u8DstID) {
			continue;
		}
		/* Your control: END*/

		while(BC_Enqueue(BC_ModInQueue[MOD_MYSELF], &qe, TIMEOUT_COMMON) == BC_FALSE) {
		}
		vTaskResume(DataHubHandle);
	}
}

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

sint32_t uputn(USART_TypeDef * usart, sint8_t * str, uint32_t size)
{
	if(!usart || !str) {
		return -1;
	}
	if(0 == size) {
		return 0;
	}
	for(;size != 0; size--) {
		USART_SendData(usart,*str++);
		while(USART_GetFlagStatus(usart,USART_FLAG_TC)!=SET);
	}

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
/* FILE is typedef¡¯ d in stdio.h. */ 
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

sint32_t _BC_Printf(const sint8_t * file_name, uint32_t line, uint32_t mod_id, const sint8_t * fmt, ...)
{
	va_list ap;
	sint32_t tmp = 0;
	uint8_t * pBCPrinfBuf = NULL;
	// uint8_t BCPrinfBuf[PRINTF_BUF_SIZE];
	stTermMsgUnit TermMsgUnit;
	BC_QueueElement qe;

	va_start(ap, fmt);

	pBCPrinfBuf = (uint8_t *)pvPortMalloc(PRINTF_BUF_SIZE);
	if(!pBCPrinfBuf) {
		va_end(ap);
		return -1;
	}
	tmp = snprintf(pBCPrinfBuf, PRINTF_BUF_SIZE, "M%-2d %s[%d]:", mod_id, file_name, line);
	if(tmp <= 0) {
		vPortFree(pBCPrinfBuf);
		pBCPrinfBuf = NULL;
		va_end(ap);
		return -2;
	}
	tmp += vsnprintf(pBCPrinfBuf+tmp, PRINTF_BUF_SIZE-tmp, fmt, ap);
	if(tmp < 0) {
		vPortFree(pBCPrinfBuf);
		pBCPrinfBuf = NULL;
		va_end(ap);
		return -3;
	}
	if(uxTaskPriorityGet(NULL) > BC_CONFIG_PRIORITY_COMMON_TASK ) {
		printf("%s\r\n", pBCPrinfBuf);
		vPortFree(pBCPrinfBuf);
		pBCPrinfBuf = NULL;
	} else {
		TermMsgUnit.TermClbkCmd = TERM_CLBK_CMD_PRINTF;
		TermMsgUnit.ClbkPara.PrintfPara.msg = pBCPrinfBuf;
		BC_MsgInit(&qe, BC_MOD_ANONYMITY, BC_MOD_TERMINAL);
		BC_MsgSetMsg(&qe, (uint8_t *)&TermMsgUnit, sizeof(TermMsgUnit));
		if(BC_FALSE == BC_Enqueue(BC_ModOutQueue[BC_MOD_TERMINAL], &qe, TIMEOUT_COMMON)) {
			vPortFree(pBCPrinfBuf);
			pBCPrinfBuf = NULL;
			return -4;
		}
	}
	va_end(ap);
	return tmp;
}

sint32_t ProcTermMsg(BC_QueueElement * p_qe)
{
	sint32_t ret = BC_OK;
	stWifiMsgUnit testWifiMsgUnit;
	stTermMsgUnit * TermMsgUnit = NULL;

	if(!p_qe) {
		return -1;
	}

	if(BC_MOD_IRQ != p_qe->u8SrcID) {
		TermMsgUnit = (stTermMsgUnit *)(p_qe->pText);
		if(TERM_CLBK_CMD_PRINTF == TermMsgUnit->TermClbkCmd)
		{
			if(TermMsgUnit->ClbkPara.PrintfPara.msg) {
				printf("%s\r\n", TermMsgUnit->ClbkPara.PrintfPara.msg);
				vPortFree(TermMsgUnit->ClbkPara.PrintfPara.msg);
				TermMsgUnit->ClbkPara.PrintfPara.msg = NULL;
			}
		}
		BC_MsgDropedInit(p_qe, MOD_MYSELF);
		// BC_MsgDropedInit(p_qe, MOD_MYSELF);
	} else {
		switch(p_qe->pText[0]) {
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
			case 'N':
				testWifiMsgUnit.WifiClbkCmd = WIFI_CLBK_CMD_SET_NET;
				testWifiMsgUnit.ClbkPara.SetNetPara.Ssid = WifiSsidBuf;
				testWifiMsgUnit.ClbkPara.SetNetPara.Pwd = WifiPwdBuf;
				break;
			default:
				BC_MsgDropedInit(p_qe, MOD_MYSELF);
				return -2;
				break;
		}
		BC_MsgInit(p_qe, MOD_MYSELF, BC_MOD_WIFI);
		BC_MsgSetMsg(p_qe, (uint8_t *)&testWifiMsgUnit, sizeof(testWifiMsgUnit));
	}
	return ret;
}

