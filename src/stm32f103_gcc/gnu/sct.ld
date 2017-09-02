/* MEMORY */
/* { */
/*         FLASH_ON_CHIP   (rx)    : ORIGIN = 0x08000000, LENGTH = 256K */
/*         SRAM_ON_CHIP    (rwx)   : ORIGIN = 0x20000000, LENGTH = 48K */
/* } */
SECTIONS
{
    /*flash start address: 0x08000000*/
    . = 0x08000000;

    /* code */
    /* __code_start_flash__ = .; */

    .vector : {
        STM32F10x_Startup.o(.vector);
        STM32F10x_Startup.o(.init);
    }

	.text : {
		*(.text);
	}
    . = ALIGN(4);
    __code_end_flash__ = .;

    /* rodata */
    __rodata_start_flash__ = .;
	.rodata : {
		*(.rodata);
	}
    . = ALIGN(4);
    __rodata_end_flash__ = .;

    /* data */
    __data_start_flash__ = .;

    /* sram start address: 0x20000000 */
    . = 0x20000000;

    /* data */
    __data_start__ = .;
	.data : AT(__data_start_flash__){
		*(.data);
	}
    __data_end__ = .;
    __data_size__ = __data_end__ - __data_start__;

    /* bss */
    __bss_start__ = .;
    .bss :
    {
        *(.bss);
        *(COMMON);
    }
    . = ALIGN(4);
    __bss_end__ = .;
    __bss_size__ = __bss_end__ - __bss_start__;

    /* SP */
    __stack_end__ = 0x20000000 + __data_size__ + __bss_size__;

    end = __stack_end__;
}

