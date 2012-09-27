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
#ifndef __ARM_ASM_MACRO_H__
#define __ARM_ASM_MACRO_H__

#include <arm_defines.h>

#ifdef __ASSEMBLY__

.macro SET_CURRENT_FLAGS flags, treg
	mrs	\treg, cpsr
	orr	\treg, \treg, #(\flags)
	msr 	cpsr, \treg
.endm

.macro SET_CURRENT_MODE mode
	cps	#(\mode)
.endm

.macro SET_CURRENT_STACK new_stack
	ldr	sp, \new_stack
.endm

.macro START_EXCEPTION_HANDLER irqname, lroffset
	.align 5
\irqname:
	sub	lr, lr, #\lroffset
.endm

/* Save User Registers */
.macro PUSH_USER_REGS
	str     lr, [sp, #-4]!;         /* Push the return address */
	sub     sp, sp, #(4*15);        /* Adjust the stack pointer */
	stmia   sp, {r0-r12};           /* Push user mode registers */
	add     r0, sp, #(4*13);        /* Adjust the stack pointer */
	stmia   r0, {r13-r14}^;         /* Push user mode registers */
	mov     r0, r0;                 /* NOP for previous inst */
	mrs     r0, spsr_all;           /* Put the SPSR on the stack */
	str     r0, [sp, #-4]!
.endm

/* If came from priviledged mode then push banked registers */
.macro PUSH_BANKED_REGS skip_lable
	mov	r4, r0
	and	r0, r0, #CPSR_MODE_MASK
	cmp	r0, #CPSR_MODE_USER
	beq	\skip_lable
	add	r1, sp, #(4*14)
	mrs	r5, cpsr
	orr	r4, r4, #(CPSR_IRQ_DISABLED | CPSR_FIQ_DISABLED)
	msr	cpsr, r4
	str	sp, [r1, #0]
	str	lr, [r1, #4]
	msr	cpsr, r5
	\skip_lable:
.endm

/* Call C function to handle exception */
.macro CALL_EXCEPTION_CFUNC cfunc
	mov	r0, sp
	bl	\cfunc
.endm

/* If going back to priviledged mode then pull banked registers */
.macro PULL_BANKED_REGS skip_lable
	ldr     r0, [sp, #0]
	mov	r4, r0
	and	r0, r0, #CPSR_MODE_MASK
	cmp	r0, #CPSR_MODE_USER
	beq	\skip_lable
	add	r1, sp, #(4*14)
	mrs	r5, cpsr
	orr	r4, r4, #(CPSR_IRQ_DISABLED | CPSR_FIQ_DISABLED)
	msr	cpsr, r4
	ldr	sp, [r1, #0]
	ldr	lr, [r1, #4]
	msr	cpsr, r5
	\skip_lable:
.endm

/* Restore User Registers */
.macro PULL_USER_REGS
	ldr     r0, [sp], #0x0004;      /* Get SPSR from stack */
	msr     spsr_all, r0;
	ldmia   sp, {r0-r14}^;          /* Restore registers (user) */
	mov     r0, r0;                 /* NOP for previous isnt */
	add     sp, sp, #(4*15);        /* Adjust the stack pointer */
	ldr     lr, [sp], #0x0004       /* Pull return address */
.endm

.macro END_EXCEPTION_HANDLER
	movs	pc, lr
.endm

#endif

#endif
