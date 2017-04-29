#include "bc_type.h"

#include "terminal.h"
#include "datahub_task.h"


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
	{BC_MOD_DATAHUB, 	0, 					0, 		0, 		NULL},
	{BC_MOD_PHONE_APP, 	BC_MOD_ZIGBEE, 		0, 		0, 		NULL},
	{BC_MOD_ZIGBEE, 	BC_MOD_PHONE_APP, 	0, 		0, 		NULL},
	{BC_MOD_BLUETOOTH, 	BC_MOD_PHONE_APP, 	0, 		0, 		NULL},
	{BC_MOD_NET_SERVER, BC_MOD_PHONE_APP, 	0, 		0, 		NULL},
	{BC_MOD_TERMINAL, 	0, 					0, 		0, 		NULL}, 
	{BC_MOD_INVALID, 	0, 					0, 		0, 		NULL},
};
uint32_t ModMsgMapSize = sizeof(ModMsgMap) / sizeof(BC_MsgDirMap);

void TaskDataHub(void *pvParameters)
{
	// block for 2000ms
	const TickType_t delay_ms = 2000 / portTICK_PERIOD_MS;
	uint32_t i = 0;
	uint8_t u8SrcID = BC_MOD_INVALID;

	for(i = 0; i < ModMsgMapSize; i++) {
		u8SrcID = ModMsgMap[i].u8SrcID;
	}

	while(BC_TRUE) {
		vTaskDelay(delay_ms);
		printf("DataHub\r\n");
	}
}

