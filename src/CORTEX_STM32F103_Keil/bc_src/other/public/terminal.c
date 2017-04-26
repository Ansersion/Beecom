#include "terminal.h"

#if 1
#pragma import(__use_no_semihosting)             
struct __FILE 
{ 
	int handle; 
	/* Whatever you require here. If the only file you are using is */ 
	/* standard output using printf() for debugging, no file handling */ 
	/* is required. */ 
}; 
/* FILE is typedef¡¯ d in stdio.h. */ 
FILE __stdout;       
void _sys_exit(int x) 
{ 
	x = x; 
} 

int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);
	USART_SEND(USART1, ch);
	return ch;
}

volatile void IrqUsartTerminal(void)
{
}

#endif
