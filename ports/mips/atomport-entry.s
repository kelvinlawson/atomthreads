/*
 * Copyright (c) 2011, Himanshu Chauhan. All rights reserved.
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

#include "atomport-asm-macros.h"

.extern _stack_start
.section .start.text,"ax",@progbits

EXCEPTION_VECTOR(_tlbmiss, 0x00, _handle_tlbmiss)
EXCEPTION_VECTOR(_cache_error, 0x100, _handle_cache_error)
EXCEPTION_VECTOR(_general_exception, 0x180, _handle_general_exception)
/* FIXME: We don't need this when in EIC mode. */
EXCEPTION_VECTOR(_interrupts, 0x200, _handle_interrupt)

LEAF(_start)
	mtc0	zero, CP0_CONTEXT
	nop
	nop
	nop

	/* globally disable interrupts until we are prepared. */
	disable_global_interrupts

	/* clear CPU timer counters. We don't want surprises. */
	mtc0	zero, CP0_COMPARE
	mtc0	zero, CP0_COUNT

        li      a0, 0xC0000000 /* FIXME: Remove these two hard codings */
	li      a1, 0x14000000
	bal     create_tlb_entry
	move    zero, a2

	la	sp, _stack_start	/* setup the stack (bss segment) */
	la	t0, main
	j	t0			/* Call the C- code now */
	nop

1:	b 	1b 			/* we should not come here whatsoever */
END(_start)

LEAF(_handle_tlbmiss)
#if 0
	disable_global_interrupts
	move k0, sp
	SAVE_INT_CONTEXT(_int_stack)
	move a0, sp
	bal vmm_cpu_handle_pagefault
	nop
	enable_global_interrupts
	eret
#else
	b _handle_tlbmiss
	nop
#endif
END(_handle_tlbmiss)

.extern handle_mips_systick
.extern _int_stack
LEAF(_handle_interrupt)
	disable_global_interrupts
	mfc0 k0, CP0_CAUSE
	lui k1, 0x4000
	and k0, k1, k0
	beq k0, zero, 1f
	nop

	move k0, sp
	/* Calculate interrupt context base */
	addi sp, sp, -(NUM_CTX_REGS * WORD_SIZE)
	SAVE_INT_CONTEXT(sp)
	bal handle_mips_systick
	nop
	RESTORE_INT_CONTEXT(sp)
1:
	enable_global_interrupts
	eret
END(_handle_interrupt)

LEAF(_handle_cache_error)
	b _handle_cache_error
	nop
END(_handle_cache_error)

LEAF(_handle_general_exception)
	b _handle_general_exception
	nop
END(_handle_general_exception)

/**
 * a0 -> Contains virtual address.
 * a1 -> Contains physical address.
 * a2 -> TLB index: If -1 select automatically.
 */
.globl create_tlb_entry
LEAF(create_tlb_entry)
	mtc0 a2, CP0_INDEX /* load the tlb index to be programmed. */
	srl a0, a0, 12 /* get the VPN */
	sll a0, a0, 12
	nop
	mtc0 a0, CP0_ENTRYHI /* load VPN in entry hi */
	addi t0, a1, 0x1000 /* next PFN for entry lo1 in T0 */
	srl a1, a1, 12 /* get the PFN */
	sll a1, a1, 6 /* get the PFN */
	srl t0, t0, 12
	sll t0, t0, 6
	ori a1, a1, 0x7 /* mark the page writable, global and valid */
	mtc0 a1, CP0_ENTRYLO0
	ori t0, t0, 0x7 /* mark the next physical page writable, global and valid */
	nop
	nop
	mtc0 t0, CP0_ENTRYLO1
	nop
	nop
	nop
	tlbwi
	ehb
	j ra
	nop
END(create_tlb_entry)
