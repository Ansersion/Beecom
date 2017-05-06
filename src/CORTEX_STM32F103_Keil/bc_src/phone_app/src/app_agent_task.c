#include "app_agent_common.h"

#include "terminal.h"

#include "app_agent_task.h"
#include "app_agent_common.h"

void TaskAppAgent(void *pvParameters)
{
	// block for 2000ms
	const TickType_t delay_ms = 2000 / portTICK_PERIOD_MS;

	while(BC_TRUE) {
		vTaskDelay(delay_ms);
		// printf("TaskAppAgent\r\n");
	}
}

