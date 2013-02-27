

MEMORY
{
    FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 0x00040000
    SRAM (rwx) : ORIGIN = 0x20000000, LENGTH = 0x00010000
}

SECTIONS
{
    _vRamTop = 0x20000000 + 0x00010000;
    .text :
    {
        KEEP(*(.vectors))
		*(.startup)
        *(.text*)
        *(.rodata*)
    } > FLASH

    /*
     * for exception handling/unwind - some Newlib functions (in common with
     * C++ and STDC++) use this.
     */

    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > FLASH

    __exidx_start = .;
    .ARM.exidx :
    {
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    } > FLASH
    __exidx_end = .;

    _etext = .;

    .data : AT (__exidx_end)
    {
        _data = .;
        *(vtable)
        *(.data*)
        _edata = .;
    } > SRAM

    /* zero initialized data */

    .bss :
    {
		__bss_start__ = . ;
        _bss = .;
        *(.bss*)
        *(COMMON)
		__bss_end__ = . ;
        _ebss = .;
    } > SRAM

    /* Where we put the heap with cr_clib */

    .cr_heap :
    {
        end = .;
        _pvHeapStart = .;
    } > SRAM

    _vStackTop = _vRamTop - 16;
	.stack  _vStackTop :
	{
		__c_stack_top__ = . ;
	}
}
