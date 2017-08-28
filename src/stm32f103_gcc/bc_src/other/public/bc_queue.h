#ifndef BC_QUEUE_H
#define BC_QUEUE_H

#include "beecomint.h"
#include "FreeRTOS.h"
#include "queue.h"

// TODO:
// use api instead of constant array size
extern QueueHandle_t BC_ModInQueue[];
extern const uint32_t BC_ModInQueueSize;
extern QueueHandle_t BC_ModOutQueue[];
extern const uint32_t BC_ModOutQueueSize;

#endif

