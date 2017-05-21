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
