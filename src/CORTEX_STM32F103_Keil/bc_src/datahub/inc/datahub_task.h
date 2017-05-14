//   Copyright 2017-04 Ansersion
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

#ifndef DATAHUB_TASK_H
#define DATAHUB_TASK_H

#include <datahub_common.h>
#include <bc_type.h>

void TaskDataHub(void *pvParameters);
static sint32_t TaskDataHubInit(DataHubInit_Type * init_type);
static sint32_t BC_Transmit2Mod(BC_QueueElement * que_elm, BC_MsgDirMap * msg_dir_map);

static sint32_t ProcQueueElm(void);

#endif

