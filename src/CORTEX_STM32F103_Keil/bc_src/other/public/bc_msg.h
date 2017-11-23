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

sint32_t CheckMsgUnit(uint32_t type_size);


#endif

