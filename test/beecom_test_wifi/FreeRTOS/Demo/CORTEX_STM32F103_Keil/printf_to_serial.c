#include "printf_to_serial.h"

#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
	/* Whatever you require here. If the only file you are using is */ 
	/* standard output using printf() for debugging, no file handling */ 
	/* is required. */ 
}; 
/* FILE is typedef�� d in stdio.h. */ 
FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
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
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{      
	// while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
	// USART1->DR = (u8) ch;      
	// return ch;
	
	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
	// USART1->DR = (u8) ch;      
	// USART1->DR = (u8)ch;
	USART_SEND(USART1, ch);
	return ch;
}
#endif
