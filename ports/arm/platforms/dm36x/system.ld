
ENTRY(__interrupt_vector_table)


MEMORY
{
    sram (rwx) : ORIGIN = 0x80000000, LENGTH = 0x08000000
}


EXTERN(__interrupt_vector_table);


C_STACK_SIZE = 4096;
IRQ_STACK_SIZE = 4096;
FIQ_STACK_SIZE = 2048;
SVC_STACK_SIZE = 4096;
ABT_STACK_SIZE = 2048;
UND_STACK_SIZE = 2048;

SECTIONS
{

    
    .text :
    {
        _start_vectors = .;
        *(.vectors)
        _end_vectors = .;
        /* Startup assembly */
        *(.startup)
        *(.init)

        /* Rest of the code (C) */
        *(.text)
        *(.rodata)
        *(.rodata*)

        _end_text = .;

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
