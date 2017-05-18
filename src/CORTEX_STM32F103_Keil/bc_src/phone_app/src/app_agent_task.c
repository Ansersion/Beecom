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

#include <app_agent_task.h>
#include <app_agent_common.h>

extern TaskHandle_t DataHubHandle;

#define LED_GREEN_TURN() (GPIOD->ODR ^= 1<<2) // green

#define BC_MOD_MYSELF BC_MOD_PHONE_APP

void TaskAppAgent(void *pvParameters)
{
	// block for 2000ms
	const TickType_t delay_ms = 2000 / portTICK_PERIOD_MS;

	while(BC_TRUE) {
		vTaskDelay(delay_ms);
		// printf("TaskAppAgent\r\n");
	}
}
// volatile void IrqUsartWifi( void )
// {
// }
// 
