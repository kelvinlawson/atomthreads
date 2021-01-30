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
#include "atomport-private.h"
#include "config.h"


/** Forward declarations */
static NO_REG_SAVE void thread_shell (void);


/**
 * \b thread_shell
 *
 * Shell routine which is used to call all thread entry points.
 *
 * This routine is called whenever a new thread is starting, and is
 * responsible for taking the entry point parameter off the TCB
 * and passing this into the thread entry point, as well as enabling
 * interrupts.
 *
 * This is an optional function for a port, because interrupts could
 * be enabled by the first-thread and normal context restore routines,
 * but that would require special handling in the normal context
 * switch routine (archContextSwitch()) that is only needed the first
 * time a thread is started. A much neater method is to direct all
 * threads through this shell routine first, so that interrupts will
 * always be enabled at thread startup, and no special first-time-run
 * handling is required in the context restore routines (i.e. we
 * don't affect normal context switch times just for the benefit of
 * the first time a thread is restored by adding extra complication
 * to the thread restore routines).
 *
 * Other ports are free to implement whatever scheme they wish. In
 * particular if you save all necessary registers (including the
 * interrupt enable register) on a context switch then you need not
 * worry about any special requirements for starting threads for the
 * first time because you can preinitialise the stack context with
 * a suitable register value that will enable interrupts.
 *
 * If the compiler supports it, stack space can be saved by preventing
 * the function from saving registers on entry. This is because we
 * are called directly by the context-switch assembler, and know that
 * threads cannot return from here. The NO_REG_SAVE macro is used to
 * denote such functions in a compiler-agnostic way, though not all
 * compilers support it.
 *
 * @return None
 */
static NO_REG_SAVE void thread_shell (void)
{
    ATOM_TCB *curr_tcb;

    /* Get the TCB of the thread being started */
    curr_tcb = atomCurrentContext();

    /**
     * Enable interrupts - these will not be enabled when a thread
     * is first restored.
     */
    ENABLE_INTERRUPTS();

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
 * Architecture-specific thread context initialisation routine.
 *
 * This function must set up a thread's context ready for restoring
 * and running the thread via archFirstThreadRestore() or
 * archContextSwitch().
 *
 * (COSMIC & SDCC) On this port we take advantage of the fact
 * that when the context switch routine is called these compilers will
 * automatically stack all registers which should not be clobbered.
 * This means that the context switch need only save and restore the
 * stack pointer, which is stored in the thread's TCB. Because of
 * this, it is not necessary to prefill a new thread's stack with any
 * register values here. The only entry we need to make in the stack
 * is the thread's entry point - this is not exactly restored when
 * the thread is context switched in, but rather is popped off the
 * stack by the context switch routine's RET call. That is used to
 * direct the program counter to our thread's entry point - we are
 * faking a return to a caller which never actually existed.
 *
 * (IAR) The IAR compiler works around the lack of CPU registers on
 * STM8 by allocating some space in low SRAM which is used for
 * "virtual" registers. The compiler uses these like normal CPU
 * registers, and hence their values must be preserved when
 * context-switching between threads. Some of these (?b8 to ?b15)
 * are expected to be preserved by called functions, and hence we
 * actually need to save/restore those registers (unlike the rest
 * of the virtual registers and the standard CPU registers). We
 * therefore must prefill the stack with values for ?b8 to ?b15
 * here.
 *
 * We could pre-initialise the stack so that the RET call goes
 * directly to the thread entry point, with the thread entry
 * parameter filled in. On this architecture, however, we use an
 * outer thread shell routine which is used to call all threads.
 * The thread entry point and parameter are stored in the thread's
 * TCB which the thread shell uses to make the actual call to the
 * entry point. We don't therefore need to store the actual thread
 * entry and parameter within the stack.
 *
 * Note that interrupts must be enabled the first time a thread is
 * run. On some architectures this might be done by setting an
 * initial value for the interrupt-enable register within the stack
 * area. In this port, however, we use the thread shell to enable
 * interrupts at the start of any thread.
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
     * The thread restore routines will perform a RET which expects to
     * find the address of the calling routine on the stack. In this case
     * (the first time a thread is run) we "return" to the entry point for
     * the thread. That is, we store the thread entry point in the
     * place that RET will look for the return address: the stack.
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
     * Because we are using a threa./STM8S105K6d shell which is responsible for
     * calling the real entry point, it also passes the parameters
     * to entry point and we need not stack the entry parameter here.
     *
     * Other ports may wish to store entry_param in the appropriate
     * parameter registers when creating a thread's context,
     * particularly if that port saves those registers anyway.
     */

    /**
     * (IAR) Set up initial values for ?b8 to ?b15.
     */
#if defined(__IAR_SYSTEMS_ICC__)
    *stack_ptr-- = 0;    // ?b8
    *stack_ptr-- = 0;    // ?b9
    *stack_ptr-- = 0;    // ?b10
    *stack_ptr-- = 0;    // ?b11
    *stack_ptr-- = 0;    // ?b12
    *stack_ptr-- = 0;    // ?b13
    *stack_ptr-- = 0;    // ?b14
    *stack_ptr-- = 0;    // ?b15
#endif

    /**
     * (COSMIC & SDCC) We do not initialise any registers via the
     * initial stack context at all.
     */

    /**
     * All thread context has now been initialised. All that is left
     * is to save the current stack pointer to the thread's TCB so
     * that it knows where to start looking when the thread is started.
     */
    tcb_ptr->sp_save_ptr = stack_ptr;

}


/**
 * \b archInitSystemTickTimer
 *
 * Initialise the system tick timer for a 1ms overflow.
   Uses TIM4, as this is "cheap" and available in most STM8's
 *
 * @return None
 */
void archInitSystemTickTimer ( void )
{
    // stop the timer
    sfr_TIM4.CR1.CEN = 0;

    // clear counter
    sfr_TIM4.CNTR.byte = 0x00;

    // auto-reload value buffered
    sfr_TIM4.CR1.ARPE = 1;

    // clear pending events
    sfr_TIM4.EGR.byte  = 0x00;

    // fMaster = 16MHz
    #if (FSYS_FREQ == 16000000L)

        // set clock to 16Mhz/2^6 = 250kHz -> 4us period
        sfr_TIM4.PSCR.PSC = 6;

        // set autoreload value for 1ms (=250*4us)
        sfr_TIM4.ARR.byte  = 250;

    // fMaster = 20MHz
    #elif (FSYS_FREQ == 20000000L)

        // set clock to 20Mhz/2^7 = 156,25kHz -> 6.4us period
        sfr_TIM4.PSCR.PSC = 7;

        // set autoreload value for 1ms (=156.25*6.4us)
        // account for fractional ARR in ISR
        sfr_TIM4.ARR.byte  = 156;

    // fMaster = 24MHz
    #elif (FSYS_FREQ == 24000000L)

        // set clock to 24Mhz/2^7 = 187.5kHz -> 5.33us period
        sfr_TIM4.PSCR.PSC = 7;

        // set autoreload value for 1ms (=187.5*5.33us)
        // account for fractional ARR in ISR
        sfr_TIM4.ARR.byte  = 187;

    // FSYS_FREQ not yet supported -> add manually
    #else
        #error FSYS_FREQ not yet supported -> add manually
    #endif

    // enable timer 4 interrupt
    sfr_TIM4.IER.UIE = 1;

    // start the timer
    sfr_TIM4.CR1.CEN = 1;

}


/**
 *
 * System tick ISR.
 *
 * This is responsible for regularly calling the OS system tick handler.
 * The system tick handler checks if any timer callbacks are necessary,
 * and runs the scheduler.
 *
 * The CPU automatically saves all registers before calling out to an
 * interrupt handler like this.
 *
 * The system may decide to schedule in a new thread during the call to
 * atomTimerTick(), in which case the program counter will be redirected
 * to the new thread's running location during atomIntExit(). This ISR
 * function will not actually complete until the thread we interrupted is
 * scheduled back in, at which point the end of this function will be
 * reached (after atomIntExit()) and the IRET call by the compiler will
 * return us to the interrupted thread as if we hadn't run any other
 * thread in the meantime. In other words the interrupted thread can be
 * scheduled out by atomIntExit() and several threads could run before we
 * actually reach the end of this function. When this function does
 * finally complete, the return address (the PC of the thread which was
 * interrupted) will be on the interrupted thread's stack because it was
 * saved on there by the CPU when the interrupt triggered.
 *
 * As with all interrupts, the ISR should call atomIntEnter() and
 * atomIntExit() on entry and exit. This serves two purposes:
 *
 * a) To notify the OS that it is running in interrupt context
 * b) To defer the scheduler until after the ISR is completed
 *
 * We defer all scheduling decisions until after the ISR has completed
 * in case the interrupt handler makes more than one thread ready.
 *
 * @return None
 */
#if defined(__CSMC__)
    @svlreg ISR_HANDLER(TIM4_SystemTickISR, _TIM4_OVR_UIF_VECTOR_)
#else
    ISR_HANDLER(TIM4_SystemTickISR, _TIM4_OVR_UIF_VECTOR_)
#endif
{
    static uint8_t  count_period = 0;       // for thread timing
#if (FSYS_FREQ != 16000000L)
    static uint8_t  count_fractional = 0;   // only required for fractional reload value
#endif
    
    // clear timer 4 interrupt flag
    sfr_TIM4.SR.UIF = 0;

// 16MHz -> reload=250 -> no correction required
#if (FSYS_FREQ == 16000000L)
    // dummy

// 20MHz -> reload=156.25 -> every 4th cycle
#elif (FSYS_FREQ == 20000000L)
    if ((++count_fractional) == 4)
    {
        count_fractional  = 0;
        sfr_TIM4.ARR.byte = 157;
    }
    else
        sfr_TIM4.ARR.byte = 156;

// 20MHz -> reload=187.5 -> every 2nd cycle
#elif (FSYS_FREQ == 24000000L)
    if ((++count_fractional) == 2)
    {
        count_fractional  = 0;
        sfr_TIM4.ARR.byte = 188;
    }
    else
        sfr_TIM4.ARR.byte = 187;

// FSYS_FREQ not yet supported -> add manually
#else
    #error FSYS_FREQ not yet supported -> add manually
#endif


    // atomthread period has passed -> handle threads
    if ((++count_period) == PERIOD_THREADS)
    {
        // reset counter
        count_period = 0;

        /* Call the interrupt entry routine */
        atomIntEnter();

        /* Call the OS system tick handler */
        atomTimerTick();

        /* Call the interrupt exit routine */
        atomIntExit(TRUE);
    }

}
