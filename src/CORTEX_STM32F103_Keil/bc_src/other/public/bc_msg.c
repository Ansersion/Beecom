#include <string.h>
#include <bc_msg.h>

sint32_t BC_MsgInit(BC_QueueElement * qe, uint8_t src_mod, uint8_t dst_mod)
{
	if(!qe) {
		return INVALID_QUE_ELEMNT;
	}
	if(!BC_ASSERT_MOD_ID_VALID(src_mod)) {
		return INVALID_SRC_MOD_ID;
	}
	if(!BC_ASSERT_MOD_ID_VALID(dst_mod)) {
		return INVALID_DST_MOD_ID;
	}
	qe->u16MsgLen = 0;
	qe->u8SrcID = src_mod;
	qe->u8DstID = dst_mod;
	return BC_OK;
}

sint32_t BC_MsgSetSrcMod(BC_QueueElement * qe, uint8_t src_mod)
{
	if(!qe) {
		return INVALID_QUE_ELEMNT;
	}
	if(!BC_ASSERT_MOD_ID_VALID(src_mod)) {
		return INVALID_SRC_MOD_ID;
	}
	qe->u8SrcID = src_mod;
	return BC_OK;
}

sint32_t BC_MsgSetDstMod(BC_QueueElement * qe, uint8_t dst_mod)
{
	if(!qe) {
		return INVALID_QUE_ELEMNT;
	}
	if(!BC_ASSERT_MOD_ID_VALID(dst_mod)) {
		return INVALID_SRC_MOD_ID;
	}
	qe->u8DstID = dst_mod;
	return BC_OK;
}

sint32_t BC_MsgSetMsg(BC_QueueElement * qe, uint8_t * msg, uint16_t msg_size)
{
	if(!qe) {
		return INVALID_QUE_ELEMNT;
	}
	if(!msg) {
		return INVALID_MSG;
	}
	if(msg_size > BC_CONFIG_QUEUE_ELEMENT_BUF_SIZE) {
		return INVALID_MSG_SIZE;
	}
	qe->u16MsgLen = msg_size;
	memcpy(qe->pText, msg, msg_size);
	return BC_OK;
}
