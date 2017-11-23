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

