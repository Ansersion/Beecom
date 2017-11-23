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

#include <bc_type.h>
#include <bc_queue.h>

QueueHandle_t BC_ModInQueue[] = {
	NULL, 	// FOR MOD_DEFAULT,
	NULL, 	// FOR MOD_DATAHUB,
	NULL, 	// FOR MOD_WIFI,
	NULL, 	// FOR MOD_PHONE_APP,
	NULL, 	// FOR MOD_ZIGBEE,
	NULL, 	// FOR MOD_BLUETOOTH,
	NULL, 	// FOR MOD_NET_SERVER,
	NULL, 	// FOR MOD_TERMINAL, 
	NULL, 	// FOR MOD_IRQ, 
	NULL, 	// FOR MOD_INVALID,
};
const uint32_t BC_ModInQueueSize = sizeof(BC_ModInQueue) / sizeof(QueueHandle_t);

QueueHandle_t BC_ModOutQueue[] = {
	NULL, 	// FOR MOD_DEFAULT,
	NULL, 	// FOR MOD_DATAHUB,
	NULL, 	// FOR MOD_WIFI,
	NULL, 	// FOR MOD_PHONE_APP,
	NULL, 	// FOR MOD_ZIGBEE,
	NULL, 	// FOR MOD_BLUETOOTH,
	NULL, 	// FOR MOD_NET_SERVER,
	NULL, 	// FOR MOD_TERMINAL, 
	NULL, 	// FOR MOD_IRQ, 
	NULL, 	// FOR MOD_INVALID,
};
const uint32_t BC_ModOutQueueSize = sizeof(BC_ModOutQueue) / sizeof(QueueHandle_t);

