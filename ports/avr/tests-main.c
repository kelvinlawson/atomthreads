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

#include <avr/pgmspace.h>

#include "atom.h"
#include "atomport.h"
#include "atomtests.h"
#include "atomtimer.h"

#include "uart.h"


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
 * Startup code stack size
 *
 * This defines the size of stack allowed for the main() startup
 * code before the OS is actually started. This needs to be large
 * enough to manage the atomOSInit(), atomOSStart() and
 * atomThreadCreate() calls which occur before the OS is started.
 *
 * In this case we use the default startup stack location used by
 * avr-gcc of the top of RAM (defined as RAMEND). After the OS
 * is started this allocation is no longer required,therefore
 * you could alternatively use some location which you know that
 * your application will not use until the OS is started. Note
 * that you cannot use the idle thread or main thread stack here
 * because the stack contexts of these threads are initialised
 * during OS creation.
 *
 * Instead of reusing some application area, here we set aside
 * 64 bytes of RAM for this purpose, because we call out to
 * several different test applications, and do not know of any
 * particular application locations which will be free to use.
 */
#define STARTUP_STACK_SIZE_BYTES    64


/*
 * Main thread stack size
 *
 * Here we utilise the space starting at 64 bytes below the startup
 * stack for the Main application thread. Note that this is not a
 * required OS kernel thread - you will replace this with your own
 * application thread.
 *
 * In this case the Main thread is responsible for calling out to the
 * test routines. Once a test routine has finished, the thread remains
 * running and loops printing out an error message (if the test failed)
 * or flashes a LED once per second (if the test passed).
 *
 * The Main thread stack generally needs to be larger than the idle
 * thread stack, as not only does it need to store interrupt handler
 * stack saves and context switch saves, but the application main thread
 * will generally be carrying out more nested function calls and require
 * stack for application code local variables etc.
 *
 * With all OS tests implemented to date on the AVR, the Main thread
 * stack has not exceeded 147 bytes. Care must be taken to ensure that
 * the data section, BSS section, and 64 byte startup section leave
 * enough free space for the main thread. You can use the avr-size
 * command to view the size of the BSS and data sections in your
 * application ELF files. For example if you require a 196 byte main
 * thread stack, then the data, BSS and startup stack combined must
 * not exceed RAMSIZE-196 bytes.
 */


/* Local data */

/* Application threads' TCBs */
static ATOM_TCB main_tcb;

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
     * Note: to protect OS structures and data during initialisation,
     * interrupts must remain disabled until the first thread
     * has been restored. They are reenabled at the very end of
     * the first thread restore, at which point it is safe for a
     * reschedule to take place.
     */

    /* Initialise the OS before creating our threads */
    status = atomOSInit(&idle_thread_stack[IDLE_STACK_SIZE_BYTES - 1]);
    if (status == ATOM_OK)
    {
        /* Enable the system tick timer */
        avrInitSystemTickTimer();

        /* Create an application thread */
        status = atomThreadCreate(&main_tcb,
                     TEST_THREAD_PRIO, main_thread_func, 0,
                     (POINTER)(RAMEND-STARTUP_STACK_SIZE_BYTES));
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
    printf_P(PSTR("Go\n"));

    /* Start test. All tests use the same start API. */
    test_status = test_start();

    /* Test finished, sleep forever */
    while (1)
    {
        /* Log test status */
        if (test_status == 0)
        {
            /* Toggle a LED (STK500-specific) */
            PORTB ^= (1 << 7);
        }
        else
        {
            printf_P (PSTR("Fail%d\n"), atomTimeGet());
        }

        /* Sleep for one second and log status again */
        atomTimerDelay(SYSTEM_TICKS_PER_SEC);
    }
}