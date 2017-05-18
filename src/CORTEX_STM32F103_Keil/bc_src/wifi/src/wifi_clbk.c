#include <wifi_clbk.h>
#include <bc_queue.h>

// use for initialization
sint32_t CheckWifiMsgUnit(void)
{
	if(sizeof(stWifiMsgUnit) > BC_CONFIG_QUEUE_ELEMENT_BUF_SIZE) {
		return BC_ERR;
	} else {
		return BC_OK;
	}
}


