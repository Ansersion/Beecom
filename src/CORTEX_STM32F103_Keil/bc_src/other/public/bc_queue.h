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

