/*
 * Copyright (c) 2013, Natie van Rooyen. All rights reserved.
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

#include <stdio.h>
#include <string.h>
#include <sys/reent.h>

#include "atom.h"
#include "atomport.h"


/** Functions defined in atomport_s.s */
extern void contextEnableInterrupts (void);


/** Forward declarations */
static void thread_shell (void);


/**
 * \b thread_shell
 *
 * Shell routine which is used to call all thread entry points.
 *
 * This routine is called whenever a new thread is starting, and
 * provides a simple wrapper around the thread entry point that
 * allows us to carry out any actions we want to do on thread's
 * first starting up, or returning after completion.
 *
 * We mainly just want to make sure interrupts are enabled when a
 * thread is run for the first time. This can be done via stack
 * restores when threads are first run, but it's handy to have this
 * wrapper anyway to run some common code if threads run to
 * completion.
 *
 * A thread shell is also handy for providing port users with a place
 * to do any other initialisation that must be done for each thread
 * (e.g. opening stdio files etc).
 *
 * @return None
 */
static void thread_shell (void)
{
    ATOM_TCB *curr_tcb;

    /* Get the TCB of the thread being started */
    curr_tcb = atomCurrentContext();

    /**
     * Open a stdout file descriptor so that the thread has its own stdout.
     * In theory threads could open stdout to different output drivers
     * if syscalls.s supported different output write functions.
     */
    stdout = fopen ("/debuguart", "w");
    setvbuf (stdout, 0, _IONBF, 0);
 
    /**
     * Enable interrupts - these will not be enabled when a thread
     * is first restored.
     */
    contextEnableInterrupts ();

    /* Call the thread entry point */
    if (curr_tcb && curr_tcb->entry_point)
    {
        curr_tcb->entry_point(curr_tcb->entry_param);
    }

    /* Clean up after thread completion */
    fclose (stdout);
    _reclaim_reent (&(curr_tcb->port_priv.reent));

    /* Thread has run to completion: remove it from the ready list */
    curr_tcb->terminated = TRUE;
    atomSched (FALSE);
}


/**
 * \b archThreadContextInit
 *
 * Architecture-specific thread context initialisation routine.
 *
 * This function must set up a thread's context ready for restoring
 * and running the thread via archFirstThreadRestore() or
 * archContextSwitch().
 *
 * The layout required to fill the correct register values is
 * described in archContextSwitch(). Note that not all registers
 * are restored by archContextSwitch() and archFirstThreadRestore()
 * as this port takes advantage of the fact that not all registers
 * must be stored by ARM gcc C subroutines. This means that we don't
 * provide start values for those registers, as they are "don't cares".
 *
 * @param[in] tcb_ptr Pointer to the TCB of the thread being created
 * @param[in] stack_top Pointer to the top of the new thread's stack
 * @param[in] entry_point Pointer to the thread entry point function
 * @param[in] entry_param Parameter to be passed to the thread entry point
 *
 * @return None
 */
void archThreadContextInit (ATOM_TCB *tcb_ptr, void *stack_top, void (*entry_point)(uint32_t), uint32_t entry_param)
{
    uint32_t *stack_ptr;

    /** Start at stack top */
    tcb_ptr->sp_save_ptr = stack_ptr = stack_top;

    /**
     * After restoring all of the context registers, the thread restore
     * routines will return to the address of the calling routine on the
     * stack. In this case (the first time a thread is run) we "return"
     * to the entry point for the thread. That is, we store the thread
     * entry point in the place that return will look for the return
     * address: the stack.
     *
     * Note that we are using the thread_shell() routine to start all
     * threads, so we actually store the address of thread_shell()
     * here. Other ports may store the real thread entry point here
     * and call it directly from the thread restore routines.
     *
     * Because we are filling the stack from top to bottom, this goes
     * on the stack first (at the top).
     */
    *stack_ptr-- = (uint32_t)thread_shell;

    /**
     * Store starting register values for R4-R11
     */
    *stack_ptr-- = (uint32_t) 0x00001111;   /* R11 */
    *stack_ptr-- = (uint32_t) 0x00001010;   /* R10 */
    *stack_ptr-- = (uint32_t) 0x00000909;   /* R9 */
    *stack_ptr-- = (uint32_t) 0x00000808;   /* R8 */
    *stack_ptr-- = (uint32_t) 0x00000707;   /* R7 */
    *stack_ptr-- = (uint32_t) 0x00000606;   /* R6 */
    *stack_ptr-- = (uint32_t) 0x00000505;   /* R5 */
    *stack_ptr   = (uint32_t) 0x00000404;   /* R4 */

    /**
     * All thread context has now been initialised. Save the current
     * stack pointer to the thread's TCB so it knows where to start
     * looking when the thread is started.
     */
    tcb_ptr->sp_save_ptr = stack_ptr;

    /**
     * At thread startup (and every time we switch back to a thread)
     * we set the global reentrancy pointer to this thread's reent struct
     */
    _REENT_INIT_PTR (&(tcb_ptr->port_priv.reent));
}

