#include "terminal.h"
#include "bt_ui_task.h"

void TaskBluetoothUI(void *pvParameters)
{
	// block for 2000ms
	const TickType_t delay_ms = 2000 / portTICK_PERIOD_MS;

	while(BC_TRUE) {
		vTaskDelay(delay_ms);
		// printf("TaskBluetoothUI\r\n");
	}
}

