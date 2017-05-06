#include <string.h>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

#include <stm32f10x_gpio.h>

#include <bc_msg.h>
#include <terminal.h>
#include <bc_queue.h>

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

sint32_t InitTerm(void)
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
	if(BC_OK == CheckEndFlag(UsartTermBuf, Index, EndFlag, FlagSize)) {
		xQueueSendFromISR(UsartMsgQueue, &GotMsgFlag, &xHigherPriorityTaskWoken);
		UsartTermBuf[Index] = '\0';
		Index = 0;
	}
}

void TaskTerminal(void * pvParameters)
{
	static BC_QueueElement qe;

	if(InitTerm() != BC_OK) {
		while(1) {
			vTaskDelay(TIMEOUT_COMMON);
			printf("Terminal: Init error\r\n");
		}
	}
	while(1) {
		while(BC_FALSE == xQueueReceive(UsartMsgQueue, &GotMsgFlag, TIMEOUT_COMMON)) {
			// Indicate led
			LED_GREEN_TURN();
		}
		BC_MsgInit(&qe, BC_MOD_MYSELF, BC_MOD_DATAHUB);
		BC_MsgSetMsg(&qe, UsartTermBuf, strlen(UsartTermBuf));
		while(BC_EnQueue(BC_ModInQueue[BC_MOD_MYSELF], &qe, TIMEOUT_COMMON) == BC_FALSE) {
		}
		vTaskResume(DataHubHandle);
		if(BC_OutQueue(BC_ModOutQueue[BC_MOD_MYSELF], &qe, 0) == BC_TRUE) {
			// process 
		}
	}
}

sint32_t CheckEndFlag(uint8_t * msg, uint32_t msg_size, uint8_t * flag, uint32_t flag_size)
{

	static sint32_t i = 0;
	if(msg_size < flag_size) {
		return BC_ERR;
	}
	if(!msg) {
		return BC_ERR;
	}
	if(!flag) {
		return BC_ERR;
	}
	for(i = 0; i < flag_size; i++) {
		if(msg[msg_size - flag_size + i] != flag[i]) {
			return BC_ERR;
		}
	}
	return BC_OK;
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
