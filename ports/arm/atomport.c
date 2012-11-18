/*
 * Copyright (c) 2012, Natie van Rooyen. All rights reserved.
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
#include "types.h"


/* *
 *
 * Functions defined in atomport_s.S
 *
 */
typedef void * SYSCONTEXT ;

extern void             contextSwitch (SYSCONTEXT* save_context, SYSCONTEXT* new_context) ;
extern void             contextStart (SYSCONTEXT* context) ;
extern void             contextEnableInterrupts (void) ;

/**
 * \b thread_shell
 *
 * Documented in atomThreads.
 *
 */
void
thread_shell (void)
{
    ATOM_TCB *curr_tcb;

    /* Get the TCB of the thread being started */
    curr_tcb = atomCurrentContext();

    /**
     * Enable interrupts - these will not be enabled when a thread
     * is first restored.
     */
    // sei();
     contextEnableInterrupts () ;

    /* Call the thread entry point */
    if (curr_tcb && curr_tcb->entry_point)
    {
        curr_tcb->entry_point(curr_tcb->entry_param);
    }

    /* Not reached - threads should never return from the entry point */
}


/**
 * \b archThreadContextInit
 *
 * Documented in atomThreads.
 *
 */
void
archThreadContextInit (ATOM_TCB *tcb_ptr, void *stack_top, void (*entry_point)(uint32_t), uint32_t entry_param)
{
    uint32_t * stack_ptr ;

    tcb_ptr->sp_save_ptr = stack_top;
    tcb_ptr->entry_param = entry_param ;
    tcb_ptr->entry_point = entry_point ;

    stack_ptr  = (uint32_t *)stack_top;             //-- Load stack pointer

	*stack_ptr = ( uint32_t ) thread_shell ; 	
	stack_ptr--;

	*stack_ptr = ( uint32_t ) 0x00001111;	/* R11 */
	stack_ptr--;	
	*stack_ptr = ( uint32_t ) 0x00001010;	/* R10 */
	stack_ptr--;	
	*stack_ptr = ( uint32_t ) 0x00000909;	/* R9 */
	stack_ptr--;	
	*stack_ptr = ( uint32_t ) 0x00000808;	/* R8 */
	stack_ptr--;	
	*stack_ptr = ( uint32_t ) 0x00000707;	/* R7 */
	stack_ptr--;	
	*stack_ptr = ( uint32_t ) 0x00000606;	/* R6 */
	stack_ptr--;	
	*stack_ptr = ( uint32_t ) 0x00000505;	/* R5 */
	stack_ptr--;	
	*stack_ptr = ( uint32_t ) 0x00000404;	/* R4 */

#ifdef CONTEXT_THREAD_ID
	stack_ptr--;	
	*stack_ptr = context_thread_id++ ;	/* thread_id */
#endif

    tcb_ptr->sp_save_ptr = stack_ptr ;
}


/**
 * \b archFirstThreadRestore
 *
 * Documented in atomThreads.
 *
 */
void
archFirstThreadRestore(ATOM_TCB * p_sp_new)
{
    contextStart (&p_sp_new->sp_save_ptr) ;
}


/**
 * \b archContextSwitch
 *
 * Documented in atomThreads.
 *
 */
void
archContextSwitch (ATOM_TCB * p_sp_old, ATOM_TCB * p_sp_new)
{
    contextSwitch (&p_sp_old->sp_save_ptr, &p_sp_new->sp_save_ptr) ;
}


