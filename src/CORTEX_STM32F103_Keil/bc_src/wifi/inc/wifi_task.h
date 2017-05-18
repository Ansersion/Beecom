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

#ifndef WIFI_TASK_H
#define WIFI_TASK_H

// STM32 headers
#include <stm32f10x_usart.h>
#include <stm32f10x_it.h>

// Beecom headers
#include <wifi_common.h>
#include <bc_type.h>
#include <wifi_irq.h>

/* UART interrupt handler. */
volatile void IrqUsartWifi( void );

void TaskWifiAgent(void *para);
sint32_t TaskWifiAgentInit(void);
sint32_t ProcWifiMsg(BC_QueueElement * qe, uint8_t * wifi_msg);

#endif

