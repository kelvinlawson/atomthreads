/*
 * Copyright (c) 2011, Anup Patel for Atomthreads Project.
 * All rights reserved.
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

#include <arm_asm_macro.h>

	.section .text

/**
 * int archSetJump(pt_regs_t *regs)
 */
	.globl archSetJump
archSetJump:
	add	r0, r0, #(4 * 16)
	str	lr, [r0]
	sub	r0, r0, #(4 * 14)
	stm	r0, {r1-r14}
	mov	r0, r0 /* NOP */
	str	r2, [r1]
	mov	r2, #0
	sub	r0, r0, #4
	str	r2, [r0]
	mrs	r2, cpsr_all
	sub	r0, r0, #4
	str	r2, [r0]
	ldr	r2, [r1]
	mov	r0, #1
	bx	lr

/**
 * void archLongJump(pt_regs_t *regs)
 */
	.globl archLongJump
archLongJump:
	mrs	r1, cpsr_all
	SET_CURRENT_MODE CPSR_MODE_IRQ
	mov	sp, r0
	SET_CURRENT_MODE CPSR_MODE_FIQ
	mov	sp, r0
	msr	cpsr_all, r1
	ldr     r1, [r0], #4 /* Get CPSR from stack */
	orr	r2, r1, #(CPSR_IRQ_DISABLED | CPSR_FIQ_DISABLED)
	msr	cpsr_all, r2
	msr	spsr_all, r1
	ldm	r0, {r0-r15}^
	mov	r0, r0 /* NOP */

