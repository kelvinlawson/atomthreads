/******************************************************************************
 *
 * hello_codered.ld - Code Red linker configuration file for hello.
 *
 * Copyright (c) 2006-2012 Texas Instruments Incorporated.  All rights reserved.
 * Software License Agreement
 * 
 * Texas Instruments (TI) is supplying this software for use solely and
 * exclusively on TI's microcontroller products. The software is owned by
 * TI and/or its suppliers, and is protected under applicable copyright
 * laws. You may not combine this software with "viral" open-source
 * software in order to form a larger program.
 * 
 * THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
 * NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
 * NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
 * CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
 * DAMAGES, FOR ANY REASON WHATSOEVER.
 * 
 * This is part of revision 9107 of the EK-LM3S6965 Firmware Package.
 *
 *****************************************************************************/

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
        *(.vectors)
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

    /*
     * Note: (ref: M0000066)
     * Moving the stack down by 16 is to work around a GDB bug.
     * This space can be reclaimed for Production Builds.
     */

    _vStackTop = _vRamTop - 16;
	.stack  _vStackTop :
	{
		__c_stack_top__ = . ;
	}
}
