/*
 * Copyright (c) 2010, Kelvin Lawson. All rights reserved.
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
#include "atomvm.h"
#include "windows.h"

/** Forward declarations */
static void thread_shell (void);
DWORD WINAPI cntrl_thread_proc (LPVOID lpParameter) ;

/* Global data */
HATOMVM the_atomvm ;

/* Local data */
static HANDLE cntrl_thread ;


/**
 * \b atomvmRun
 *
 * Starts the atom vm. atomvmRun creates a thread from where the atomvmCtrlRun function
 * will be called. atomvmCtrlRun never returns and this thread becomes the controll
 * thread of the vm.
 *
 */
void 
atomvmRun ()
{
	atomvmCtrlCreate (&the_atomvm) ;
	cntrl_thread = CreateThread (NULL, 0, cntrl_thread_proc, (uint32_t*)the_atomvm, CREATE_SUSPENDED, NULL) ;
	ResumeThread (cntrl_thread) ;
}

DWORD WINAPI 
cntrl_thread_proc (LPVOID lpParameter)
 {
	atomvmCtrlRun ((HATOMVM)lpParameter, 0) ;
	return 0 ;
 }


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
	//atomvmExitCritical () ;
    atomvmInterruptMask (0) ;

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
    tcb_ptr->sp_save_ptr = stack_top;
	tcb_ptr->entry_param = entry_param ;
	tcb_ptr->entry_point = entry_point ;

	atomvmContextCreate (&tcb_ptr->context, (unsigned int )stack_top, (unsigned int )thread_shell) ;
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
	atomvmContextSwitch (0, p_sp_new->context) ;
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
    atomvmContextSwitch (p_sp_old->context, p_sp_new->context) ;
}


/**
 * \b archTimerTickIrqHandler
 *
 * System timer tick interrupt handler.
 *
 */
void archTimerTickIrqHandler ()
{
    atomIntEnter();

    /* Call the OS system tick handler */
    atomTimerTick();

	/* Call the interrupt exit routine */
    atomIntExit(TRUE);
}
