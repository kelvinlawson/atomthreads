/*
 * Copyright (c) 2015, Stefan Petersen. All rights reserved.
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
#include "atomtests.h"
#include "atomevent.h"


/* Number of test threads */
#define NUM_TEST_THREADS      1


/* Test OS objects */
static ATOM_TCB tcb[NUM_TEST_THREADS];
static uint8_t test_thread_stack[NUM_TEST_THREADS][TEST_THREAD_STACK_SIZE];


/* Test result tracking */
static volatile int g_result;

/* Forward declarations */
static void test1_thread_func (uint32_t param);


/**
 * \b test_start
 *
 * Start test.
 *
 * @retval Number of failures
 */
uint32_t test_start (void)
{
    uint8_t status;
    int failures;

    /* Default to zero failures */
    failures = 0;

    /* Run test and update "failures" count */
    if (atomThreadCreate(&tcb[0], TEST_THREAD_PRIO - 1, test1_thread_func, 0,
                         &test_thread_stack[0][0],
                         TEST_THREAD_STACK_SIZE, TRUE) != ATOM_OK)
    {
        /* Fail */
        ATOMLOG (_STR("Error creating test thread 1\n"));
        failures++;
    }
    else
    {
        /* Give the other thread some time to start */
        atomTimerDelay(20);
        /* Send some events to see if we can do this repeatedly */
        status = atomEventSignal(&tcb[0], 0x02);
        if (status != ATOM_OK) {
            failures++;
        }

        if (g_result != 1)
        {
            failures++;
        }
    }

    /* If threads are created, check for thread stack overflow */

    /* Quit */
    return failures;
}


/**
 * \b test1_thread_func
 *
 * Entry point for test thread 1.
 *
 * @param[in] param Unused (optional thread entry parameter)
 *
 * @return None
 */
static void test1_thread_func (uint32_t param)
{
    ATOM_EVENTS events;
    int failures;

    /* Compiler warnings */
    param = param;

    /* Default to zero failures */
    failures = 0;
    g_result = 0;

    /* Just check without locking and no signal */
    if (atomEventWait(1, &events, -1) != ATOM_WOULDBLOCK) {
        failures++;
    }
    /* Check that the event register is cleared after each atomEventWait() */
    if (tcb[0].events != 0) {
        failures++;
    }

    /* Just check with timeout and no signal */
    if (atomEventWait(1, &events, 10) != ATOM_TIMEOUT) {
        failures++;
    }
    /* Check that the event register is cleared after each atomEventWait() */
    if (tcb[0].events != 0) {
        failures++;
    }

    /* Wait timed for event to actually be signalled */
    if (atomEventWait(0x02, &events, 15) != ATOM_OK) {
        failures++;
    }
    if (events != 0x02) {
        failures++;
    }
    /* Check that the event register is cleared after each atomEventWait() */
    if (tcb[0].events != 0) {
        failures++;
    }

    if (failures == 0) {
        g_result = 1;
    } else {
        g_result = 0;
    }

    /* Wait forever */
    while (1)
    {
        atomTimerDelay (SYSTEM_TICKS_PER_SEC);
    }
}
