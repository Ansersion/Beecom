#ifndef BC_INIT_H
#define BC_INIT_H

#include "stm32f10x_nvic.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"

#include "beecomint.h"
#include "FreeRTOS.h"
#include "queue.h"

/* for serial terminal */
#define USART_TERMINAL_BAUD_RATE 	9600

/* for wifi module*/
#define USART_WIFI_BAUD_RATE 	115200

/* system essential initialization */
sint32_t BC_Init(void);
void LedInit(void);
void UsartTermInit(uint32_t bound);
void UsartWifiInit(uint32_t bound);
void BufInit(void);
// void SocketInit(void);
sint32_t BC_QueueInit(void);
sint32_t BC_MutexInit(void);

#endif

