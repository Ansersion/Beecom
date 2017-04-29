#include <bc_type.h>
#include <bc_queue.h>

QueueHandle_t BC_ModInQueue[] = {
	NULL, 	// FOR MOD_DEFAULT,
	NULL, 	// FOR MOD_DATAHUB,
	NULL, 	// FOR MOD_PHONE_APP,
	NULL, 	// FOR MOD_ZIGBEE,
	NULL, 	// FOR MOD_BLUETOOTH,
	NULL, 	// FOR MOD_NET_SERVER,
	NULL, 	// FOR MOD_TERMINAL, 
	NULL, 	// FOR MOD_INVALID,
};
const uint32_t BC_ModInQueueSize = sizeof(BC_ModInQueue) / sizeof(QueueHandle_t);

QueueHandle_t BC_ModOutQueue[] = {
	NULL, 	// FOR MOD_DEFAULT,
	NULL, 	// FOR MOD_DATAHUB,
	NULL, 	// FOR MOD_PHONE_APP,
	NULL, 	// FOR MOD_ZIGBEE,
	NULL, 	// FOR MOD_BLUETOOTH,
	NULL, 	// FOR MOD_NET_SERVER,
	NULL, 	// FOR MOD_TERMINAL, 
	NULL, 	// FOR MOD_INVALID,
};
const uint32_t BC_ModOutQueueSize = sizeof(BC_ModOutQueue) / sizeof(QueueHandle_t);

