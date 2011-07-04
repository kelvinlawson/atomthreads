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

#include <atomport-private.h>

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

/* Save User Registers in FIQ */
.macro PUSH_FIQUSER_REGS
	str     lr, [sp, #-4]!;         /* Push the return address */
	sub     sp, sp, #(4*15);        /* Adjust the stack pointer */
	stmia   sp, {r0-r7};            /* Push user mode registers */
	add     r0, sp, #(4*8);         /* Adjust the stack pointer */
	stmia   r0, {r8-r14}^;          /* Push user mode registers */
	mov     r0, r0;                 /* NOP for previous inst */
	mrs     r0, spsr_all;           /* Put the SPSR on the stack */
	str     r0, [sp, #-4]!
.endm

/* Call C function to handle exception */
.macro CALL_EXCEPTION_CFUNC cfunc
	mov	r0, sp
	bl	\cfunc
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

	.section .expvect, "ax", %progbits
	.globl _start_vect
_start_vect:	
	ldr	pc, __reset
	ldr	pc, __undefined_instruction
	ldr	pc, __software_interrupt
	ldr	pc, __prefetch_abort
	ldr	pc, __data_abort
	ldr	pc, __not_used
	ldr	pc, __irq
	ldr	pc, __fiq
__reset:
	.word _reset
__undefined_instruction:
	.word _undefined_instruction
__software_interrupt:
	.word _software_interrupt
__prefetch_abort:
	.word _prefetch_abort
__data_abort:
	.word _data_abort
__not_used:
	.word _not_used
__irq:
	.word _irq
__fiq:
	.word _fiq
	.global _end_vect
_end_vect:

__svc_stack_end:
	.word _svc_stack_end
__und_stack_end:
	.word _und_stack_end
__abt_stack_end:
	.word _abt_stack_end
__irq_stack_end:
	.word _irq_stack_end
__fiq_stack_end:
	.word _fiq_stack_end
__usr_stack_end:
	.word _usr_stack_end

	.globl _reset
_reset:
	/* Clear a register for temporary usage */
	mov	r8, #0
	/* Disable IRQ & FIQ */
	cpsid if
	/* Set Supervisor Mode Stack */
	SET_CURRENT_MODE CPSR_MODE_SUPERVISOR
	SET_CURRENT_STACK __svc_stack_end
	/* Set Undefined Mode Stack */
	SET_CURRENT_MODE CPSR_MODE_UNDEFINED
	SET_CURRENT_STACK __und_stack_end
	/* Set Abort Mode Stack */
	SET_CURRENT_MODE CPSR_MODE_ABORT
	SET_CURRENT_STACK __abt_stack_end
	/* Set IRQ Mode Stack */
	SET_CURRENT_MODE CPSR_MODE_IRQ
	SET_CURRENT_STACK __irq_stack_end
	/* Set FIQ Mode Stack */
	SET_CURRENT_MODE CPSR_MODE_FIQ
	SET_CURRENT_STACK __fiq_stack_end
	/* Set System Mode Stack */
	SET_CURRENT_MODE CPSR_MODE_SYSTEM
	SET_CURRENT_STACK __usr_stack_end
	/* Set to Supervisor Mode */
	SET_CURRENT_MODE CPSR_MODE_SUPERVISOR
	/* Call main function */
	bl	main
	/* We should never reach here */
	b	.
	

START_EXCEPTION_HANDLER _undefined_instruction, 4
	PUSH_USER_REGS
	CALL_EXCEPTION_CFUNC do_undefined_instruction
	PULL_USER_REGS
END_EXCEPTION_HANDLER

START_EXCEPTION_HANDLER _software_interrupt, 4
	PUSH_USER_REGS
	CALL_EXCEPTION_CFUNC do_software_interrupt
	PULL_USER_REGS
END_EXCEPTION_HANDLER

START_EXCEPTION_HANDLER _prefetch_abort, 4
	PUSH_USER_REGS
	CALL_EXCEPTION_CFUNC do_prefetch_abort
	PULL_USER_REGS
END_EXCEPTION_HANDLER

START_EXCEPTION_HANDLER _data_abort, 8
	PUSH_USER_REGS
	CALL_EXCEPTION_CFUNC do_data_abort
	PULL_USER_REGS
END_EXCEPTION_HANDLER

START_EXCEPTION_HANDLER _not_used, 4
	PUSH_USER_REGS
	CALL_EXCEPTION_CFUNC do_not_used
	PULL_USER_REGS
END_EXCEPTION_HANDLER

START_EXCEPTION_HANDLER _irq, 4
	PUSH_USER_REGS
	CALL_EXCEPTION_CFUNC do_irq
	PULL_USER_REGS
END_EXCEPTION_HANDLER

START_EXCEPTION_HANDLER _fiq, 4
	PUSH_FIQUSER_REGS
	CALL_EXCEPTION_CFUNC do_fiq
	PULL_USER_REGS
END_EXCEPTION_HANDLER

