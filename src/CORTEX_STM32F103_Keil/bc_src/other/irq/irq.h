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

#ifndef IRQ_H
#define IRQ_H

#define IrqUsartWifi 		xUSART2_IRQHandler

#define IrqUsartTerminal 	vUARTInterruptHandler

#define USART_SEND(usart_type, ch)      ((usart_type)->DR = (ch) & (uint16_t)0x01FF)
#define USART_RECEIVE(usart_type, ch)   ((ch) = (usart_type)->DR & (uint16_t)0x01FF)

#define HFSR 	((uint32_t *)0xE000ED2C)
#define MFSR ((char *)0xE000ED28)
#define BFSR ((char *)0xE000ED29)
#define UFSR ((char *)0xE000ED2A)
#define BFAR ((uint32_t *)0xE000ED38)

volatile void HardFault_Handler(void);
volatile void MemManage_Handler(void);
volatile void BusFault_Handler(void);
volatile void UsageFault_Handler(void);


#endif

