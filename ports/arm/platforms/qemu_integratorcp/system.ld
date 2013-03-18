
ENTRY(__interrupt_vector_table)


MEMORY
{
    flash (rx) : ORIGIN = 0x00000000, LENGTH = 0x00020000
    sram (rwx) : ORIGIN = 0x00020000, LENGTH = 0x00020000
}


EXTERN(__interrupt_vector_table);


C_STACK_SIZE = 512;
IRQ_STACK_SIZE = 256;
FIQ_STACK_SIZE = 256;
SVC_STACK_SIZE = 512;
ABT_STACK_SIZE = 256;
UND_STACK_SIZE = 256;

SECTIONS
{

    
    .text :
    {
        *(.vectors)
        /* Startup assembly */
        *(.startup)
        *(.init)

        /* Rest of the code (C) */
        *(.text)
        *(.rodata)
        *(.rodata*)

        _end_text = .;
    } >flash

    .data :
    {
        _start_data = .;
        *(.data)
        _end_data = .;
    } >sram

    .bss :
    {
        _start_bss = .;
        __bss_start__ = . ;
        *(.bss)
    } >sram

    . = ALIGN(4);
    _end_bss = .;
    __bss_end__ = . ;

    . = ALIGN(256);
    
    .stack : {
    __stack_start__ = . ;
    . += IRQ_STACK_SIZE;
    . = ALIGN (4);
    __irq_stack_top__ = . ;
    . += FIQ_STACK_SIZE;
    . = ALIGN (4);
    __fiq_stack_top__ = . ;
    . += SVC_STACK_SIZE;
    . = ALIGN (4);
    __svc_stack_top__ = . ;
    . += ABT_STACK_SIZE;
    . = ALIGN (4);
    __abt_stack_top__ = . ;
    . += UND_STACK_SIZE;
    . = ALIGN (4);
    __und_stack_top__ = . ;
    . += C_STACK_SIZE;
    . = ALIGN (4);
    __c_stack_top__ = . ;
    __stack_end__ = .;
    } >sram 

}
__end__ = .;
_end = .;
PROVIDE(end = .);

heap_top = ORIGIN(sram) + LENGTH(sram) - 4;
