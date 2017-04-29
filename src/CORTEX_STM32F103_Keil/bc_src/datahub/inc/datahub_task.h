#ifndef DATAHUB_TASK_H
#define DATAHUB_TASK_H

#include <datahub_common.h>
#include <bc_type.h>

void TaskDataHub(void *pvParameters);
sint32_t ProcQueueElm(void);
sint32_t BC_Transmit2Mod(BC_QueueElement * que_elm, BC_MsgDirMap * msg_dir_map);

#endif

