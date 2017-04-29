#include <stdio.h>
#include <panic.h>


void BC_Panic(uint8_t * msg)
{
	uint32_t i = 0;
	while(1) {
		for(i = 0; i < 100000; i++) {
		}
		printf("PANIC: %s\r\n", msg);
	}
}




