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


/** Forward declarations */
static void thread_shell (void);


/**
 * \b thread_shell
 *
 * Shell routine which is used to call all thread entry points.
 *
 * This routine is called whenever a new thread is starting, and is
 * responsible for taking the entry point parameter off the TCB
 * and passing this into the thread entry point, as well as enabling
 * interrupts. This is an optional function for a port, as it is
 * also possible to do this with a regular context restore if the
 * appropriate registers are saved for context switches (parameter
 * and interrupt enable). Alternatively a flag could be used to
 * notify archFirstThreadRestore() and archContextSwitch()
 * that they should this time restore the contents of the parameter
 * registers and enable interrupts. After restoring a thread first
 * time the context restore routines need not perform those
 * operations again. This is discussed in more detail below.
 *
 * When starting new threads, ports must pass in the entry point
 * function parameter, and enable interrupts. This can be handled by
 * the first context restore for new threads if they are saved as
 * part of the thread's context. However for the AVR port we have
 * chosen to save only the minimum registers required for context
 * switches. This reduces the cycle time required for all context
 * switches during operation. This means, however, that we don't save
 * the parameter registers R25/R24 or the SREG. These would otherwise
 * be used when restoring a thread for the first time to pass the
 * parameter to the entry point, and enabling interrupts in the SREG.
 *
 * A few of the possible ways round this are to:
 *
 * a) Save R25/R24 and SREG with the normal context - thus increasing
 *    the processor cycles required on each context switch.
 * b) Use a thread shell routine which is used to start all threads.
 *    This routine can then pass the parameter and enable interrupts,
 *    without incurring any overhead on all context switches.
 * c) Store a flag in a new thread's TCB to notify the normal context
 *    switch routine that it must (the first time a thread is restored
 *    only) pass the parameter, and enable interrupts.
 *
 * We have chosen to implement (b) in this case, as it does not affect
 * normal context switch times just for the benefit of the first restore,
 * and it does not incur extra complication to the thread restore
 * routines through handling special cases. A thread shell is also handy
 * for providing port users with a place to do any initialisation that
 * must be done for each thread (e.g. opening stdio files etc).
 *
 * Other ports are free to implement whatever scheme they wish. In
 * particular if you save all necessary registers on a context switch
 * then you need not worry about any special requirements for
 * starting threads for the first time.
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
    sei();

    /* Call the thread entry point */
    if (curr_tcb && curr_tcb->entry_point)
    {
        curr_tcb->entry_point(curr_tcb->entry_param);
    }

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
 * must be stored by gcc-avr C subroutines. This means that we don't
 * provide start values for those registers, as they are "don't cares".
 *
 * Because we don't actually save the parameter registers (R25-R24)
 * for this particular architecture, we use a separate thread shell.
 * The thread shell is always used as the thread entry point stored
 * in the thread context, and it does the actual calling of the
 * proper thread entry point, passing the thread entry parameter.
 * This allows us to pass the entry parameter without actually
 * storing it on the stack (the thread shell routine takes the
 * entry point and parameter from the thread's TCB). On other ports
 * you may instead choose to store the entry point and parameter
 * in the thread context and use no thread shell routine.
 *
 * Similarly we use the thread shell in this case to enable interrupts.
 * When a thread is restored and started for the first time, it must
 * also enable interrupts. This might be done by setting up the
 * appropriate value in the SREG register for enabled interrupts, which
 * would then be restored when the thread is first started. But to
 * reduce register-saves we do not save SREG on the AVR port, and
 * instead we use the thread shell routine to enable interrupts the
 * first time a thread is started.
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
    uint8_t *stack_ptr;

    /** Start at stack top */
    stack_ptr = (uint8_t *)stack_top;

    /**
     * After restoring all of the context registers, the thread restore
     * routines will perform a RET or RETI which expect to find the
     * address of the calling routine on the stack. In this case (the
     * first time a thread is run) we "return" to the entry point for
     * the thread. That is, we store the thread entry point in the
     * place that RET and RETI will look for the return address: the
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
    *stack_ptr-- = (uint8_t)((uint16_t)thread_shell & 0xFF);
    *stack_ptr-- = (uint8_t)(((uint16_t)thread_shell >> 8) & 0xFF);

    /**
     * Devices with 3 byte program counters (e.g. Atmega25x, Xmega)
     * must have 3 bytes stacked for the entry point. In GCC
     * function pointers are still 16-bits, however, so we cannot
     * actually pass entry points at > 64K in instruction space
     * (128KB in real terms) and just set the top byte to zero here.
     * This means that the thread_shell() function must be located
     * in the bottom 128KB. You may need to modify linker scripts to
     * force this.
     */
#ifdef __AVR_3_BYTE_PC__
    *stack_ptr-- = 0;
#endif

    /**
     * For the AVR port the parameter registers (R25-R24) are not
     * saved and restored by the context switch routines. This means
     * that they cannot be used to pass a parameter to the entry
     * point the first time a thread is restored. Rather than incur
     * the overhead of saving them (just for the benefit of starting
     * threads) we can either use a flag here to notify the context
     * restore routines (archThreadFirstRestore() and
     * archContextSwitch()) that they should restore R25-R24 this
     * one time, or we can use a thread shell routine which replaces
     * that actual thread entry point. In this case we use a thread
     * shell which is responsible for passing the parameters to the
     * actual thread entry point, without adding extra processing
     * to the context switch routines.
     *
     * Other ports may wish to store entry_param in the appropriate
     * parameter registers when creating a thread's context,
     * particularly if that port saves those registers anyway.
     *
     * Similarly, although interrupts must be enabled when starting
     * new threads, we also defer this to the thread shell because
     * we don't save the SREG contents for normal context switches.
     * Other ports may choose to context switch the relevant
     * interrupt enable register, so that the first context switch
     * is able to enable interrupts during its normal context
     * restore.
     */

    /**
     * Store starting register values for R2-R17, R28-R29
     */
    *stack_ptr-- = 0x00;    /* R2 */
    *stack_ptr-- = 0x00;    /* R3 */
    *stack_ptr-- = 0x00;    /* R4 */
    *stack_ptr-- = 0x00;    /* R5 */
    *stack_ptr-- = 0x00;    /* R6 */
    *stack_ptr-- = 0x00;    /* R7 */
    *stack_ptr-- = 0x00;    /* R8 */
    *stack_ptr-- = 0x00;    /* R9 */
    *stack_ptr-- = 0x00;    /* R10 */
    *stack_ptr-- = 0x00;    /* R11 */
    *stack_ptr-- = 0x00;    /* R12 */
    *stack_ptr-- = 0x00;    /* R13 */
    *stack_ptr-- = 0x00;    /* R14 */
    *stack_ptr-- = 0x00;    /* R15 */
    *stack_ptr-- = 0x00;    /* R16 */
    *stack_ptr-- = 0x00;    /* R17 */
    *stack_ptr-- = 0x00;    /* R28 */
    *stack_ptr-- = 0x00;    /* R29 */

    /**
     * On devices with large program space we also context switch RAMPZ, EIND.
     */
#ifdef __AVR_3_BYTE_PC__
    *stack_ptr-- = 0x00;    /* RAMPZ */
    *stack_ptr-- = 0x00;    /* EIND */
#endif

    /**
     * All thread context has now been initialised. Save the current
     * stack pointer to the thread's TCB so it knows where to start
     * looking when the thread is started.
     */
    tcb_ptr->sp_save_ptr = stack_ptr;

}

