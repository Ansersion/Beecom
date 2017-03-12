#include "printf_to_serial.h"

#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
	/* Whatever you require here. If the only file you are using is */ 
	/* standard output using printf() for debugging, no file handling */ 
	/* is required. */ 
}; 
/* FILE is typedef’ d in stdio.h. */ 
FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
_sys_exit(int x) 
{ 
	x = x; 
} 
/*
* USART1 sends data 
*/
// #define USART1_SEND(ch) (USART1->DR = (ch) & (unsigned short)0x01FF)

/*
* USART1 receive data
*/
// #define USART1_RECEIVE(ch) ((ch) = USART1->DR & (unsigned short)0x01FF)
	
// #define USART_SEND(usart_type, ch) 		((usart_type)->DR = (ch) & (unsigned short)0x01FF)
// #define USART_RECEIVE(usart_type, ch)	((ch) = (usart_type)->DR & (unsigned short)0x01FF)
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
	// while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
	// USART1->DR = (u8) ch;      
	// return ch;
	
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
	// USART1->DR = (u8) ch;      
	// USART1->DR = (u8)ch;
	USART_SEND(USART1, ch);
	return ch;
}
#endif
