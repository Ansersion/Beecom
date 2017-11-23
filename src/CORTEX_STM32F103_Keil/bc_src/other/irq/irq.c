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

#include <string.h>

#include <terminal.h>
#include <bc_type.h>

uint8_t irq_fault_msg[128];

volatile void HardFault_Handler(void)
{
	sprintf(irq_fault_msg, "HardFault:%x\r\n", *HFSR);
	sprintf(irq_fault_msg+strlen(irq_fault_msg), "MemManage:%x\r\n", *MFSR);
	sprintf(irq_fault_msg+strlen(irq_fault_msg), "BusFault:%x\r\n", *BFSR);
	sprintf(irq_fault_msg+strlen(irq_fault_msg), "BFAR:%x\r\n", *BFAR);
	sprintf(irq_fault_msg+strlen(irq_fault_msg), "UsageFault:%x\r\n", *UFSR);
	uputs(USART1, irq_fault_msg);
	for(;;);
}

volatile void MemManage_Handler(void)
{

	sprintf(irq_fault_msg, "MemManage:%x\r\n", *MFSR);
	uputs(USART1, irq_fault_msg);
	for(;;);
}

volatile void BusFault_Handler(void)
{

	sprintf(irq_fault_msg, "BusFault:%x\r\n", *BFSR);
	sprintf(irq_fault_msg+strlen(irq_fault_msg), "BFAR:%x\r\n", *BFAR);
	uputs(USART1, irq_fault_msg);
	for(;;);
}

volatile void UsageFault_Handler(void)
{

	sprintf(irq_fault_msg, "UsageFault:%x\r\n", *UFSR);
	uputs(USART1, irq_fault_msg);
	for(;;);
}
