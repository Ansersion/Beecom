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

#ifndef BC_H
#define BC_H

#include <FreeRTOS.h>
#include <task.h>

/*-------------------------------------------------------------------*/

/*************************************************
 The lowest priority, all tasks' priority must be 
 higher than this priority.
***************************************************/
#define BC_CONFIG_BASE_PRIORITY tskIDLE_PRIORITY

/*************************************************
 The priority of tasks except "TaskDataHub"
***************************************************/
#define BC_CONFIG_PRIORITY_COMMON_TASK 	(BC_CONFIG_BASE_PRIORITY + 2)

/*************************************************
 The priority of tasks except "TaskDataHub"
***************************************************/
#define BC_CONFIG_PRIORITY_DEBUG_TASK 	(BC_CONFIG_PRIORITY_COMMON_TASK + 1)

/*************************************************
 The priority of "TaskDataHub" is higher than
 the comman tasks for high speed of transmitting
 inner messages.
***************************************************/
#define BC_CONFIG_PRIORITY_DATA_HUB 	(BC_CONFIG_PRIORITY_DEBUG_TASK + 1)

/*-------------------------------------------------------------------*/

/*************************************************
 Task stack size(in unit of word) for all tasks
***************************************************/
#define BC_CONFIG_TASK_STACK_SIZE 	((unsigned short)256)

/*************************************************
 Queue element buffer size
***************************************************/
#define BC_CONFIG_QUEUE_ELEMENT_BUF_SIZE 	(64)

/*-------------------------------------------------------------------*/
/*************************************************
 Common timeout of 1 second 
***************************************************/
#define TIMEOUT_COMMON 	1000/portTICK_RATE_MS

/*************************************************
 Beecom wifi tcp server port.
***************************************************/
#define BC_CENTER_SERV_PORT 	8025

#endif

