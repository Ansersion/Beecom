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

#include <string.h>

#include <terminal.h>
#include <datahub_task.h>
#include <datahub_common.h>
#include <bc_queue.h>
#include <panic.h>


// typedef struct BC_MsgDirMap {
// 	uint8_t 		u8SrcID;
// 	uint8_t 		u8DstID1;
// 	uint8_t 		u8DstID2;
// 	uint8_t 		u8DstID3;
// 	BC_MsgDirMap * 	pExtDst;
// }BC_MsgDirMap;
// 
// The order is according to 'enum BC_ModID'
BC_MsgDirMap ModMsgMap[] = {
	{BC_MOD_DEFAULT, 	NULL, 				NULL, 					NULL, 					NULL},
	// {BC_MOD_DATAHUB, 	0, 					0, 		0, 		NULL},
	{BC_MOD_PHONE_APP, 	BC_MOD_ZIGBEE, 		BC_MOD_INVALID, 		BC_MOD_INVALID, 		NULL},
	{BC_MOD_ZIGBEE, 	BC_MOD_PHONE_APP, 	BC_MOD_INVALID, 		BC_MOD_INVALID, 		NULL},
	{BC_MOD_BLUETOOTH, 	BC_MOD_PHONE_APP, 	BC_MOD_INVALID, 		BC_MOD_INVALID, 		NULL},
	{BC_MOD_NET_SERVER, BC_MOD_PHONE_APP, 	BC_MOD_INVALID, 		BC_MOD_INVALID, 		NULL},
	{BC_MOD_TERMINAL, 	BC_MOD_INVALID, 	BC_MOD_INVALID, 		BC_MOD_INVALID, 		NULL}, 
	// {BC_MOD_INVALID, 	0, 					0, 		0, 		NULL},
};
uint32_t ModMsgMapSize = sizeof(ModMsgMap) / sizeof(BC_MsgDirMap);

void TaskDataHub(void *pvParameters)
{
	// block for 2000ms
	// static const TickType_t delay_ms = 2000 / portTICK_PERIOD_MS;
	static uint32_t i = 0;
	// static uint32_t mod_id;
	static uint8_t src_id = BC_MOD_INVALID;
	static BC_QueueElement queue_element;

	static DataHubInit_Type init_type;

	if(TaskDataHubInit(&init_type) != 0) {
		BC_Panic("Datahub init");
	}

	while(BC_TRUE) {
		// printf("DataHub\r\n");
		memset(&queue_element, 0, sizeof(queue_element));
		printf("datahub: start\r\n");
		for(i = 0; i < ModMsgMapSize; i++) {
			src_id = ModMsgMap[i].u8SrcID;
			if(!BC_ASSERT_MOD_ID_VALID(src_id)) {
				printf("DEBUG: DataHub\r\n");
				continue;
			}
			if(BC_Dequeue(BC_ModInQueue[src_id], &queue_element, 0) == BC_TRUE) {
				printf("datahub: get queue: %d->%d\r\n", queue_element.u8SrcID, queue_element.u8DstID);
				printf("bc_transmit ret=%d\r\n", BC_Transmit2Mod(&queue_element, &ModMsgMap[i]));
			}
		}
		ProcQueueElm();
		vTaskSuspend(NULL);

	}
}

static sint32_t TaskDataHubInit(DataHubInit_Type * init_type)
{
	// Nothing need to do
	if(!init_type) {
		return -1;
	}
	return BC_OK;
}

static sint32_t BC_Transmit2Mod(BC_QueueElement * que_elm, BC_MsgDirMap * msg_dir_map)
{
	if(!que_elm) {
		return -1;
	}
	if(!BC_ASSERT_MOD_ID_VALID(que_elm->u8DstID)) {
		return -2;
	}
	// if(que_elm->u8DstID == que_elm->u8SrcID) {
	// 	printf("To self: %d\r\n", que_elm->u8DstID);
	// 	return 0;
	// }
	if(que_elm->u8DstID != BC_MOD_DEFAULT) {
		// if the queue is full
		// the data will be dropped
		BC_Enqueue(BC_ModOutQueue[que_elm->u8DstID], que_elm, 0);
		return 0;
		
	} else {
		if(!msg_dir_map) {
			return -3;
		}
		if(BC_ASSERT_MOD_ID_VALID(msg_dir_map->u8DstID1) && msg_dir_map->u8DstID1 != BC_MOD_DEFAULT) {
			BC_Enqueue(BC_ModOutQueue[msg_dir_map->u8DstID1], que_elm, 0);
		} else {
			return -4;
		}	
		if(BC_ASSERT_MOD_ID_VALID(msg_dir_map->u8DstID2) && msg_dir_map->u8DstID2 != BC_MOD_DEFAULT) {
			BC_Enqueue(BC_ModOutQueue[msg_dir_map->u8DstID2], que_elm, 0);
		} else {
			return 0;
		}
		if(BC_ASSERT_MOD_ID_VALID(msg_dir_map->u8DstID3) && msg_dir_map->u8DstID3 != BC_MOD_DEFAULT) {
			BC_Enqueue(BC_ModOutQueue[msg_dir_map->u8DstID3], que_elm, 0);
		} else {
			return 0;
		}
		return BC_Transmit2Mod(que_elm, msg_dir_map->pExtDst);
	}
}

static sint32_t ProcQueueElm(void)
{
	static BC_QueueElement queue_element;
	if(BC_Dequeue(BC_ModOutQueue[BC_MOD_DATAHUB], &queue_element, 0) == BC_TRUE) {
		printf("Datahub: %d->OutQueue For Datahub\r\n", queue_element.u8SrcID);
	}
	return 0;
	// no need to resumd task 'datahub'(because it's myself)
	// vTaskResume();
}

