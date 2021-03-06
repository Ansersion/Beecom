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

#ifndef TERMINAL_H
#define TERMINAL_H

// STD headers
#include <stdio.h>

// STM32 headers
#include <stm32f10x_usart.h>

// Beecom headers
#include <irq.h>
#include <beecomint.h>
#include <bc.h>
#include <bc_type.h>


#define USART_TERMINAL 	USART1
#define USART_TERMINAL_BUF_SIZE 	128


#ifdef __MODULE__
#define __BC_FILE__ __MODULE__
#else
#define __BC_FILE__ __FILE__
#endif

#ifdef BC_DEBUG
#define BC_Printf(mod_id, fmt, ...) (_BC_Printf(__BC_FILE__, __LINE__, mod_id, fmt, ##__VA_ARGS__))
#else
#define BC_Printf(mod_id, fmt, ...)
#endif

void _sys_exit(int x);
int fputc(int ch, FILE *f);

volatile void IrqUsartTerminal(void);
// volatile void vUARTInterruptHandler(void);
void TaskTerminal(void * pvParameters);
sint32_t TaskTerminalInit(void);
// sint32_t CheckEndFlag(uint8_t * msg, uint32_t msg_size, uint8_t * flag, uint32_t flag_size);
sint32_t uputs(USART_TypeDef * usart, sint8_t * str);
sint32_t uputn(USART_TypeDef * usart, sint8_t * str, uint32_t size);

sint32_t _BC_Printf(const sint8_t * file_name, uint32_t line, uint32_t mod_id, const sint8_t * fmt, ...);

sint32_t ProcTermMsg(BC_QueueElement * qe);


#endif
