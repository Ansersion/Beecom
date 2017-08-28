/*MEMORY*/
/*{*/
	/* ROM 是可读的(r)和可执行的(x) */ 
	/*rom (rx) : ORIGIN = 0, LENGTH = 0x40000*/
	/* RAM 是可读的(r),可写的(w),可执行的(x) */ 
	/* ram (rwx) : ORIGIN = 0x20000000, LENGTH = 0x10000 */
/*(}*/
/* SECTIONS 命令 : 定义各输入段到输出段的映射 */
SECTIONS
{
	/* . = 0x0; */
	. = 0x20000000;
	/* 从 0x00000000 开始 */
	. = ALIGN(4);
	.text : {
		startup.o(.text) /* .text of wan-kernel.o must be smaller than 0x00000200 bytes*/
		/* . = ALIGN(4); */
		/* . = 0x00000200; */
		/* irq_a.o(.text) */
		*(.text) /* 程序代码 */
	}
	. = ALIGN(4);
	.rodata : {
		*(.rodata)
	}
	. = ALIGN(4);
	/* . = 0x20000000; */
	/* 从 0x20000000 开始 */
	.data : {
		*(.data)
		/* 数据存储器 */
	}
	. = ALIGN(4);
	.bss : {
		*(.bss)
		/* 预留的数据存储器,必须初始化为零 */
	}
}

