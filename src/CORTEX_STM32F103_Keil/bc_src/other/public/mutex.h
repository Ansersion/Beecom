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

#ifndef MUTEX_H
#define MUTEX_H

#include <beecomint.h>

#include <FreeRTOS.h>
#include <queue.h>

typedef struct BC_Mutex {
	QueueHandle_t qh;
	// for no use
	uint32_t u32Qh; 
}BC_Mutex;

sint32_t BCMutexInit(BC_Mutex * mutex);
sint32_t BCMutexLock(BC_Mutex * mutex);
sint32_t BCMutexUnlock(BC_Mutex * mutex);
sint32_t BCMutexLockISR(BC_Mutex * mutex);
sint32_t BCMutexUnlockISR(BC_Mutex * mutex);


#endif

