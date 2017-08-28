#ifndef BC_TYPE_H
#define BC_TYPE_H

#include <beecomint.h>
#include <bc.h>

typedef enum BC_ModID {
	BC_MOD_DEFAULT=0,
	BC_MOD_DATAHUB,
	BC_MOD_WIFI,
	BC_MOD_PHONE_APP,
	BC_MOD_ZIGBEE,
	BC_MOD_BLUETOOTH,
	BC_MOD_NET_SERVER,
	BC_MOD_TERMINAL, 
	/* indicate the queue msg from IRQ */
	BC_MOD_IRQ,
	/* BC_MOD_ANONYMITY:
	   1. as source ID: means not to response 
	   2. as destionation ID: means the message would be droped*/
	BC_MOD_ANONYMITY,
	/* invalid mod, to define the limit */
	BC_MOD_INVALID,
}BC_ModID;

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

#define BC_ASSERT_MOD_ID_VALID(ID) ((ID) < BC_MOD_INVALID)

/************************************
  Push a queue element into the queue
*************************************/
#define BC_Enqueue 	 	xQueueSend

/************************************
  Extract a queue element from the queue
*************************************/
#define BC_Dequeue 	xQueueReceive

/************************************
  Push a queue element into the queue
  in IRQ context
*************************************/
#define BC_EnqueueISR 	xQueueSendFromISR

/************************************
  Extract a queue element from the queue
  in IRQ context
*************************************/
#define BC_DequeueISR 	xQueueReceiveFromISR

#endif

