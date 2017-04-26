#include "terminal.h"
#include "datahub_task.h"

void TaskDataHub(void *pvParameters)
{
	// block for 2000ms
	const TickType_t delay_ms = 2000 / portTICK_PERIOD_MS;

	while(BC_TRUE) {
		vTaskDelay(delay_ms);
		printf("DataHub\r\n");
	}
}

