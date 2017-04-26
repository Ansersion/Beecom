#ifndef BC_H
#define BC_H

#include "task.h"

/*-------------------------------------------------------------------*/

/*************************************************
 The lowest priority, all tasks' priority must be 
 higher than this priority.
***************************************************/
#define BC_CONFIG_BASE_PRIORITY tskIDLE_PRIORITY

/*************************************************
 The priority of tasks except "TaskDataHub"
***************************************************/
#define BC_CONFIG_PRIORITY_COMMON_TASK 	(BC_CONFIG_BASE_PRIORITY + 1)

/*************************************************
 The priority of "TaskDataHub" is higher than
 the comman tasks for high speed of transmitting
 inner messages.
***************************************************/
#define BC_CONFIG_PRIORITY_DATA_HUB 	(BC_CONFIG_BASE_PRIORITY + 2)

/*-------------------------------------------------------------------*/

/*************************************************
 The priority of "TaskDataHub" is higher than
 the comman tasks for high speed of transmitting
 inner messages.
***************************************************/
#define BC_CONFIG_TASK_STACK_SIZE 	((unsigned short)256)

/*-------------------------------------------------------------------*/

#endif

