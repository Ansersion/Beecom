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

