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

#include "atom.h"
#include "atomport.h"
#include "atomport-private.h"
#include "string.h"
#include "arm_defines.h"

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
	int i;
	pt_regs_t *regs = (pt_regs_t *)((uint32_t)stack_top - sizeof(pt_regs_t));

	tcb_ptr->sp_save_ptr = stack_top;
	regs->cpsr = CPSR_COND_ZERO_MASK |
		    CPSR_ASYNC_ABORT_DISABLED | CPSR_MODE_SUPERVISOR;
	regs->gpr[0] = entry_param;
	for (i = 1; i < 13; i++) {
		regs->gpr[i] = 0x0;
	}
	regs->sp = (uint32_t)stack_top - sizeof(pt_regs_t) - 1024;
	regs->lr = (uint32_t)entry_point;
	regs->pc = (uint32_t)entry_point;
}


/**
 * archFirstThreadRestore(ATOM_TCB *new_tcb)
 *
 * This function is responsible for restoring and starting the first
 * thread the OS runs. It expects to find the thread context exactly
 * as it would be if a context save had previously taken place on it.
 * The only real difference between this and the archContextSwitch()
 * routine is that there is no previous thread for which context must
 * be saved.
 *
 * The final action this function must do is to restore interrupts.
 */
void archFirstThreadRestore(ATOM_TCB *new_tcb)
{
	pt_regs_t *regs = (pt_regs_t *)((uint32_t)new_tcb->sp_save_ptr 
							- sizeof(pt_regs_t));
	archLongJump(regs);
}

/**
 * Function that performs the contextSwitch. Whether its a voluntary release
 * of CPU by thread or a pre-emption, under both conditions this function is
 * called. The signature is as follows:
 *
 * archContextSwitch(ATOM_TCB *old_tcb, ATOM_TCB *new_tcb)
 */
void archContextSwitch(ATOM_TCB *old_tcb, ATOM_TCB *new_tcb)
{
	uint32_t tmp = 0x0, lr = 0x0;
	pt_regs_t *old_regs = (pt_regs_t *)((uint32_t)old_tcb->sp_save_ptr
							- sizeof(pt_regs_t));
	pt_regs_t *new_regs = (pt_regs_t *)((uint32_t)new_tcb->sp_save_ptr
							- sizeof(pt_regs_t));
	asm volatile (" mov %0, lr\n\t" :"=r"(lr):);
	if (archSetJump(old_regs, &tmp)) {
		old_regs->lr = lr;
		archLongJump(new_regs);
	}
}

