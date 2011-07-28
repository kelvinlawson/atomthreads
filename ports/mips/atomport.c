/*
 * Copyright (c) 2011, Himanshu Chauhan for Atomthreads Project.
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

#include "atom.h"
#include "atomport-private.h"
#include "atomport.h"
#include "atomport-asm-macros.h"
#include "string.h"


/* Used for managing nesting of atomport.h critical sections */
uint32_t at_preempt_count = 0;

/**
 * This function initialises each thread's stack during creation, before the
 * thread is first run. New threads are scheduled in using the same
 * context-switch function used for threads which were previously scheduled
 * out, therefore this function should set up a stack context which looks
 * much like a thread which has been scheduled out and had its context saved.
 * We fill part of the stack with those registers which are involved in the
 * context switch, including appropriate stack or register contents to cause
 * the thread to branch to its entry point function when it is scheduled in.
 *
 * Interrupts should also be enabled whenever a thread is restored, hence
 * ports may wish to explicitly include the interrupt-enable register here
 * which will be restored when the thread is scheduled in. Other methods
 * can be used to enable interrupts, however, without explicitly storing
 * it in the thread's context.
 */
void archThreadContextInit (ATOM_TCB *tcb_ptr, void *stack_top,
                            void (*entry_point)(UINT32),
                            UINT32 entry_param)
{
#define STORE_VAL(base, reg, val) \
	*((uint32_t *)(base + ((reg ## _IDX) * WORD_SIZE))) = (uint32_t)val

	/* Make space for context saving */
	uint32_t stack_start = (uint32_t)(stack_top - (WORD_SIZE * NUM_CTX_REGS));

	tcb_ptr->sp_save_ptr = (void *)stack_start;

	STORE_VAL(stack_start, sp, stack_start);
	STORE_VAL(stack_start, s8, stack_start);
	STORE_VAL(stack_start, s1, 0);
	STORE_VAL(stack_start, s2, 0);
	STORE_VAL(stack_start, s3, 0);
	STORE_VAL(stack_start, s4, 0);
	STORE_VAL(stack_start, s5, 0);
	STORE_VAL(stack_start, s6, 0);
	STORE_VAL(stack_start, s7, 0);
	STORE_VAL(stack_start, cp0_epc, 0);
	STORE_VAL(stack_start, ra, entry_point);
	STORE_VAL(stack_start, a0, entry_param);
}

