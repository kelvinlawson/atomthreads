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

#include <stdio.h>

#include "atom.h"
#include "atomport-private.h"
#include "atomtests.h"
#include "atomtimer.h"
#include "uart.h"
#include <avr/pgmspace.h>


/* Constants */

/*
 * Idle thread stack size
 *
 * This needs to be large enough to handle any interrupt handlers
 * and callbacks called by interrupt handlers (e.g. user-created
 * timer callbacks) as well as the saving of all context when
 * switching away from this thread.
 *
 * In this case, the idle stack is allocated on the BSS via the
 * idle_thread_stack[] byte array.
 */
#define IDLE_STACK_SIZE_BYTES       128


/*
 * Main thread stack size
 *
 * Note that this is not a required OS kernel thread - you will replace
 * this with your own application thread.
 *
 * In this case the Main thread is responsible for calling out to the
 * test routines. Once a test routine has finished, the test status is
 * printed out on the UART and the thread remains running in a loop
 * flashing a LED.
 *
 * The Main thread stack generally needs to be larger than the idle
 * thread stack, as not only does it need to store interrupt handler
 * stack saves and context switch saves, but the application main thread
 * will generally be carrying out more nested function calls and require
 * stack for application code local variables etc.
 *
 * With all OS tests implemented to date on the AVR, the Main thread
 * stack has not exceeded 201 bytes. To allow all tests to run we set
 * a minimum main thread stack size of 204 bytes. This may increase in
 * future as the codebase changes but for the time being is enough to
 * cope with all of the automated tests.
 */
#define MAIN_STACK_SIZE_BYTES       204


/*
 * Startup code stack
 *
 * Some stack space is required at initial startup for running the main()
 * routine. This stack space is only temporarily required at first bootup
 * and is no longer required as soon as the OS is started. By default
 * GCC sets this to the top of RAM (RAMEND) and it grows down from there.
 * Because we only need this temporarily, though, it would be wasteful to
 * set aside a region at the top of RAM which is not used during runtime.
 *
 * What we do here is to reuse part of the idle thread's stack during
 * initial startup. As soon as we enter the main() routine we move the
 * stack pointer to half-way down the idle thread's stack. This is used
 * temporarily while calls are made to atomOSInit(), atomThreadCreate()
 * and atomOSStart(). Once the OS is started this stack area is no
 * longer required, and can be used for its original purpose (for the
 * idle thread's stack).
 *
 * This does mean, however, that we cannot monitor the stack usage of the
 * idle thread. Stack usage is monitored by prefilling the stack with a
 * known value, and we are obliterating some of that prefilled area by
 * using it as our startup stack, so we cannot use the stack-checking API
 * to get a true picture of idle thread stack usage. If you wish to 
 * monitor idle thread stack usage for your applications then you are
 * free to use a different region for the startup stack (e.g. set aside
 * an area permanently, or place it somewhere you know you can reuse
 * later in the application). For the time being, this method gives us a
 * simple way of reducing the memory consumption without having to add
 * any special AVR-specific considerations to the automated test
 * applications.
 *
 * This optimisation was required to allow some of the larger automated
 * test modules to run on devices with 1KB of RAM. You should avoid doing
 * this if you can afford to set aside 64 bytes or so, or if you are
 * writing your own applications in which you have further control over
 * where data is located.
 */


/* Local data */

/* Application threads' TCBs */
static ATOM_TCB main_tcb;

/* Main thread's stack area */
static uint8_t main_thread_stack[MAIN_STACK_SIZE_BYTES];

/* Idle thread's stack area */
static uint8_t idle_thread_stack[IDLE_STACK_SIZE_BYTES];

/* STDIO stream */
static FILE uart_stdout = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);


/* Forward declarations */
static void main_thread_func (uint32_t data);


/**
 * \b main
 *
 * Program entry point.
 *
 * Sets up the AVR hardware resources (system tick timer interrupt) necessary
 * for the OS to be started. Creates an application thread and starts the OS.
 */

int main ( void )
{
    int8_t status;

    /**
     * Reuse part of the idle thread's stack for the stack required
     * during this startup function.
     */
    SP = (int)&idle_thread_stack[(IDLE_STACK_SIZE_BYTES/2) - 1];

    /**
     * Note: to protect OS structures and data during initialisation,
     * interrupts must remain disabled until the first thread
     * has been restored. They are reenabled at the very end of
     * the first thread restore, at which point it is safe for a
     * reschedule to take place.
     */

    /**
     * Initialise the OS before creating our threads.
     *
     * Note that we cannot enable stack-checking on the idle thread on
     * this platform because we are already using part of the idle
     * thread's stack now as our startup stack. Prefilling for stack
     * checking would overwrite our current stack.
     *
     * If you are not reusing the idle thread's stack during startup then
     * you are free to enable stack-checking here.
     */
    status = atomOSInit(&idle_thread_stack[0], IDLE_STACK_SIZE_BYTES, FALSE);
    if (status == ATOM_OK)
    {
        /* Enable the system tick timer */
        avrInitSystemTickTimer();

        /* Create an application thread */
        status = atomThreadCreate(&main_tcb,
                     TEST_THREAD_PRIO, main_thread_func, 0,
                     &main_thread_stack[0],
                     MAIN_STACK_SIZE_BYTES,
                     TRUE);
        if (status == ATOM_OK)
        {
            /**
             * First application thread successfully created. It is
             * now possible to start the OS. Execution will not return
             * from atomOSStart(), which will restore the context of
             * our application thread and start executing it.
             *
             * Note that interrupts are still disabled at this point.
             * They will be enabled as we restore and execute our first
             * thread in archFirstThreadRestore().
             */
            atomOSStart();
        }
    }

    while (1)
        ;

    /* There was an error starting the OS if we reach here */
    return (0);
}


/**
 * \b main_thread_func
 *
 * Entry point for main application thread.
 *
 * This is the first thread that will be executed when the OS is started.
 *
 * @param[in] data Unused (optional thread entry parameter)
 *
 * @return None
 */
static void main_thread_func (uint32_t data)
{
    uint32_t test_status;
    int sleep_ticks;

    /* Enable all LEDs (STK500-specific) */
    DDRB = 0xFF;
    PORTB = 0xFF;

    /* Initialise UART (9600bps) */
    if (uart_init(9600) != 0)
    {
        /* Error initialising UART */
    }

    /**
     * Redirect stdout via the UART. Note that the UART write routine
     * is protected via a semaphore, so the OS must be started before
     * use of the UART.
     */
    stdout = &uart_stdout;

    /* Put a message out on the UART */
    printf_P (PSTR("Go\n"));

    /* Start test. All tests use the same start API. */
    test_status = test_start();

    /* Check main thread stack usage (if enabled) */
#ifdef ATOM_STACK_CHECKING
    if (test_status == 0)
    {
        uint32_t used_bytes, free_bytes;

        /* Check idle thread stack usage */
        if (atomThreadStackCheck (&main_tcb, &used_bytes, &free_bytes) == ATOM_OK)
        {
            /* Check the thread did not use up to the end of stack */
            if (free_bytes == 0)
            {
                printf_P (PSTR("Main stack overflow\n"));
                test_status++;
            }

            /* Log the stack usage */
#ifdef TESTS_LOG_STACK_USAGE
            printf_P (PSTR("MainUse:%d\n"), used_bytes);
#endif
        }

    }
#endif

    /* Log final status */
    if (test_status == 0)
    {
        printf_P (PSTR("Pass\n"));
    }
    else
    {
        printf_P (PSTR("Fail(%d)\n"), test_status);
    }

    /* Flash LED once per second if passed, very quickly if failed */
    sleep_ticks = (test_status == 0) ? SYSTEM_TICKS_PER_SEC : (SYSTEM_TICKS_PER_SEC/8);

    /* Test finished, flash slowly for pass, fast for fail */
    while (1)
    {
        /* Toggle a LED (STK500-specific) */
        PORTB ^= (1 << 7);

        /* Sleep then toggle LED again */
        atomTimerDelay (sleep_ticks);
    }

}
