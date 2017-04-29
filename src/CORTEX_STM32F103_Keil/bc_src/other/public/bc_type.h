#ifndef BC_TYPE_H
#define BC_TYPE_H

#include "beecomint.h"
#include "bc.h"

enum BC_ModID {
	BC_MOD_DATAHUB=0,
	BC_MOD_PHONE_APP,
	BC_MOD_ZIGBEE,
	BC_MOD_BLUETOOTH,
	BC_MOD_NET_SERVER,
	BC_MOD_TERMINAL, 
	BC_MOD_INVALID,
};

typedef struct BC_QueueElement {
	uint8_t 	u8SrcID;
	uint8_t 	u8DstID;
	uint16_t 	u16MsgLen;
	uint8_t 	pText[BC_CONFIG_QUEUE_ELEMENT_BUF_SIZE];
}BC_QueueElement;

typedef struct BC_MsgDirMap {
	uint8_t 				u8SrcID;
	uint8_t 				u8DstID1;
	uint8_t 				u8DstID2;
	uint8_t 				u8DstID3;
	struct BC_MsgDirMap * 	pExtDst;
}BC_MsgDirMap;

#endif

