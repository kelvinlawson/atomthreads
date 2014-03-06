/*
 * Copyright (c) 2014, Juan Angel Hernandez Hdez. for Atomthreads Project.
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

ATOM_TCB    *asm_shell_new_tcb_ptr;
ATOM_TCB    *asm_shell_old_tcb;
ATOM_TCB    *asm_shell_new_tcb;

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
     * Enable interrupts - these will not be enabled when a thread
     * is first restored.
     */
    asm("FSET I");

    /* Call the thread entry point */
    if (curr_tcb && curr_tcb->entry_point)
    {
        curr_tcb->entry_point(curr_tcb->entry_param);
    }

    /* Thread has run to completion: remove it from the ready list */
    curr_tcb->suspended = TRUE;
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
 * described in archContextSwitch(). 
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
    uint16_t *stack_ptr;
    
    /** Start at stack top */
    stack_ptr = (uint16_t*)stack_top;		

    /**
     * After restoring all of the context registers, the thread restore
     * routines will perform a RTS or REIT which expect to find the
     * address of the calling routine on the stack. In this case (the
     * first time a thread is run) we "return" to the entry point for
     * the thread. That is, we store the thread entry point in the
     * place that RTS or REIT will look for the return address: the
     * stack.
     *
     * Note that we are using the thread_shell() routine to start all
     * threads, so we actually store the address of thread_shell()
     * here. Other ports may store the real thread entry point here
     * and call it directly from the thread restore routines.
     *
     * Because we are filling the stack from top to bottom, this goes
     * on the stack first (at the top).
     */
    
    /* | Ret ADD High  PC (H)  | */
    *stack_ptr-- = ((uint32_t)thread_shell >> 16L);
    /* | Ret ADD Low   PC (L)  | */
    *stack_ptr-- = (uint16_t)((uint32_t)thread_shell & 0x0000FFFFL);
     /* | FB                   | */
    *stack_ptr-- = (uint16_t)0xFBFB;
    *stack_ptr-- = (uint16_t)0xFBFB;    
     /* | FLG                  | */
    *stack_ptr-- = (uint16_t)0x00C0; //User stack pointer activated and interrupts enabled
    /* | SB                    | */
    *stack_ptr-- = (uint16_t)0x3B3B; 
    *stack_ptr-- = (uint16_t)0x3B3B;
    /* | A1                    | */
    *stack_ptr-- = (uint16_t)0xA1A1;   
    *stack_ptr-- = (uint16_t)0xA1A1;   
    /* | A0                    | */
    *stack_ptr-- = (uint16_t)0xA0A0;   
    *stack_ptr-- = (uint16_t)0xA0A0;   
    /* | R3                    | */
    *stack_ptr-- = (uint16_t)0x3333;   
    /* | R2                    | */
    *stack_ptr-- = (uint16_t)0x2222;     
    /* | R1                    | */
    *stack_ptr-- = (uint16_t)0x1111; 
    /* | R0                    | */
    *stack_ptr = (uint16_t)0x0000;          


    /**
     * All thread context has now been initialised. Save the current
     * stack pointer to the thread's TCB so it knows where to start
     * looking when the thread is started.
     */
    tcb_ptr->sp_save_ptr = stack_ptr;

}

/**
 * \b archFirstThreadRestore
 *
 * Architecture-specific function to restore and start the first thread.
 * This is called by atomOSStart() when the OS is starting.
 *
 * This function will be largely similar to the latter half of
 * archContextSwitch(). Its job is to restore the context for the
 * first thread, and finally enable interrupts (although we actually
 * enable interrupts in thread_shell() for new threads in this port
 * rather than doing it explicitly here).
 *
 * It expects to see the context saved in the same way as if the
 * thread has been previously scheduled out, and had its context
 * saved. That is, archThreadContextInit() will have been called
 * first (via atomThreadCreate()) to create a "fake" context save
 * area, containing the relevant register-save values for a thread
 * restore.
 *
 * Note that you can create more than one thread before starting
 * the OS - only one thread is restored using this function, so
 * all other threads are actually restored by archContextSwitch().
 * This is another reminder that the initial context set up by
 * archThreadContextInit() must look the same whether restored by
 * archFirstThreadRestore() or archContextSwitch().
 *
 * @param[in] new_tcb_ptr Pointer to the thread being scheduled in
 *
 * @return None
 *
 * void archFirstThreadRestore (ATOM_TCB *new_tcb_ptr)
 */
void archFirstThreadRestore(ATOM_TCB *new_tcb_ptr)
{
    asm_shell_new_tcb_ptr = new_tcb_ptr;
    
    asm("MOV.L  _asm_shell_new_tcb_ptr,A0");
    asm("LDC    [A0],SP");                  
    asm("POPM   R0,R1,R2,R3,A0,A1,SB"); 
    asm("POPC   FLG");
    asm("POPC   FB");
    asm("RTS"); 
}

/*
 * \b archContextSwitch
 *
 * Architecture-specific context switch routine.
 *
 * Note that interrupts are always locked out when this routine is
 * called. For cooperative switches, the scheduler will have entered
 * a critical region. For preemptions (called from an ISR), the
 * ISR will have disabled interrupts on entry.
 *
 * @param[in] old_tcb_ptr Pointer to the thread being scheduled out
 * @param[in] new_tcb_ptr Pointer to the thread being scheduled in
 *
 * @return None
 *
 * void archContextSwitch (ATOM_TCB *old_tcb_ptr, ATOM_TCB *new_tcb_ptr)
 */
void archContextSwitch(ATOM_TCB *old_tcb, ATOM_TCB *new_tcb)
{
    asm_shell_old_tcb = old_tcb;
    asm_shell_new_tcb = new_tcb;
    
    /* Save context */
    asm("PUSHC FLG");
    asm("PUSHM R0,R1,R2,R3,A0,A1,SB");
    asm("MOV.L  _asm_shell_old_tcb,A0"); 
    asm("STC    SP,[A0]");              
    
    /* Restore Context */
    asm("MOV.L  _asm_shell_new_tcb,A0");
    asm("LDC    [A0],SP");              
    asm("POPM   R0,R1,R2,R3,A0,A1,SB");
    asm("POPC   FLG");
    asm("POPC   FB");    
    asm("RTS");  
}

/**
 *  \b init_timerb2
 *
 *  Initiate 1MS timer. Not used in simulator mode.
 *
 *  @return None
 */
void init_timerb2(void)
{	
#ifndef HEW_SIMULATOR
    TB2MR = 0x40;       // M32C_OSC_FREQUENCY/8	      
    TB2 = COUNT_10MS;
    TB2IC = INTERRUPT_LVL_1;
#endif    
}

void init_pin_P0_0(void)
{
    PD0 = 0xFF; //All outputs.
    P0  = 0x00; //Outputs low.
}

void toggle_pin_P0_0(void)
{
    P0 ^= 1 << 0;
}