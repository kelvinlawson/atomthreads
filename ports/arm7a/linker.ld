/*
 * Copyright (c) 2011, Anup Patel. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. No personal names or organizations' names associated with the
 *    Atomthreads project may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE ATOMTHREADS PROJECT AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE PROJECT OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

OUTPUT_FORMAT("elf32-littlearm", "elf32-littlearm", "elf32-littlearm")
OUTPUT_ARCH("arm")
ENTRY(_start_vect)

SECTIONS 
{
	. = 0x100000;

	.text :
 	{
		*(.expvect)
		*(.text)
		. = ALIGN(4);
		_etext = .;
	}

	.data :
	{
		*(.data)
		. = ALIGN(4);
		_edata = .;
	}

	.bss :
	{
		*(.bss)
		. = ALIGN(4);
		_ebss = .;
	}

	.rodata :
	{
		*(.rodata .rodata.*)
		. = ALIGN(4);
		_erodata = .;
	}

	.cmdbtl :
	{
		PROVIDE(_cmdtbl_start = .);
		*(.cmdtbl)
		. = ALIGN(4);
		PROVIDE(_cmdtbl_end = .);
	}

	.modtbl :
	{
		PROVIDE(_modtbl_start = .);
		*(.modtbl)
		. = ALIGN(4);
		PROVIDE(_modtbl_end = .);
	}

 	.svc_stack :
        {
                PROVIDE(_svc_stack_start = .);
                . = . + 4096;
                . = ALIGN(4);
                PROVIDE(_svc_stack_end = .);
        }

 	.mon_stack :
        {
                PROVIDE(_mon_stack_start = .);
                . = . + 4096;
                . = ALIGN(4);
                PROVIDE(_mon_stack_end = .);
        }

 	.abt_stack :
        {
                PROVIDE(_abt_stack_start = .);
                . = . + 4096;
                . = ALIGN(4);
                PROVIDE(_abt_stack_end = .);
        }

 	.und_stack :
        {
                PROVIDE(_und_stack_start = .);
                . = . + 4096;
                . = ALIGN(4);
                PROVIDE(_und_stack_end = .);
        }

 	.irq_stack :
        {
                PROVIDE(_irq_stack_start = .);
                . = . + 4096;
                . = ALIGN(4);
                PROVIDE(_irq_stack_end = .);
        }

 	.fiq_stack :
        {
                PROVIDE(_fiq_stack_start = .);
                . = . + 4096;
                . = ALIGN(4);
                PROVIDE(_fiq_stack_end = .);
        }

 	.usr_stack :
        {
                PROVIDE(_usr_stack_start = .);
                . = . + 4096;
                . = ALIGN(4);
                PROVIDE(_usr_stack_end = .);
        }
}
