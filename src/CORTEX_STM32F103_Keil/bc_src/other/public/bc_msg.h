#ifndef BC_MSG_H
#define BC_MSG_H

#include <bc.h>
#include <beecomint.h>
#include <bc_type.h>

#define INVALID_QUE_ELEMNT 		0x00000010
#define INVALID_SRC_MOD_ID 		0x00000011
#define INVALID_DST_MOD_ID 		0x00000012
#define INVALID_MSG 			0x00000013
#define INVALID_MSG_SIZE 		0x00000014

sint32_t BC_MsgInit(BC_QueueElement * qe, uint8_t src_mod, uint8_t dst_mod);
sint32_t BC_MsgSetSrcMod(BC_QueueElement * qe, uint8_t src_mod);
sint32_t BC_MsgSetDstMod(BC_QueueElement * qe, uint8_t dst_mod);
sint32_t BC_MsgSetMsg(BC_QueueElement * qe, uint8_t * msg, uint16_t msg_size);

sint32_t BC_MsgDropedInit(BC_QueueElement * qe, uint8_t src_mod);

#endif

