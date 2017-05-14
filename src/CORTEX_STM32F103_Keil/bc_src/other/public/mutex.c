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

#include <mutex.h>
#include <bc_type.h>

sint32_t BCMutexInit(BC_Mutex * mutex)
{
	if(!mutex) {
		return -1;
	}
	// mutex->qh = NULL;
	mutex->qh = xQueueCreate(1, sizeof(uint32_t));
	if(!mutex->qh) {
		return -2;
	}
	mutex->u32Qh = 0;
	if(BC_Enqueue(mutex->qh, &(mutex->u32Qh), 0) != BC_TRUE) {
		return -3;
	}
	return BC_OK;
}

sint32_t BCMutexLock(BC_Mutex * mutex)
{
	if(!mutex) {
		return -1;
	}
	if(!(mutex->qh)) {
		return -2;
	}
	if(BC_Dequeue(mutex->qh, &(mutex->u32Qh), 0) != BC_TRUE) {
		return -3;
	}
	return BC_OK;
}

sint32_t BCMutexUnlock(BC_Mutex * mutex)
{
	if(!mutex) {
		return -1;
	}
	if(!(mutex->qh)) {
		return -2;
	}
	if(BC_Enqueue(mutex->qh, &(mutex->u32Qh), 0) != BC_TRUE) {
		return -3;
	}
	return BC_OK;
}
