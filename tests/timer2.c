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
#include "atomtimer.h"
#include "atomtests.h"


/* Test period (in seconds) */
#define TEST_PERIOD_SECS    10

/* Number of test threads */
#define NUM_TEST_THREADS      3


/* Test OS objects */
static ATOM_TCB tcb[NUM_TEST_THREADS];
static uint8_t test_thread_stack[NUM_TEST_THREADS][TEST_THREAD_STACK_SIZE];


/* Per-thread failure counts */
static volatile int g_failure_cnt[3];


/* Forward declarations */
static void test_thread_func (uint32_t param);


/**
 * \b test_start
 *
 * Start timer test.
 *
 * Tests that atomTimerDelay() delays for the correct time
 * period when used by three threads simultanously.
 *
 * @retval Number of failures
 */
uint32_t test_start (void)
{
    int failures;

    /* Default to zero failures */
    failures = 0;
    g_failure_cnt[0] = g_failure_cnt[1] = g_failure_cnt[2] = 0;

    /* Create Thread 1 */
    if (atomThreadCreate(&tcb[0], TEST_THREAD_PRIO, test_thread_func, 1,
          &test_thread_stack[0][0],
          TEST_THREAD_STACK_SIZE, TRUE) != ATOM_OK)
    {
        /* Fail */
        ATOMLOG (_STR("Thread1\n"));
        failures++;
    }

    /* Create Thread 2 */
    if (atomThreadCreate(&tcb[1], TEST_THREAD_PRIO, test_thread_func, 2,
          &test_thread_stack[1][0],
          TEST_THREAD_STACK_SIZE, TRUE) != ATOM_OK)
    {
        /* Fail */
        ATOMLOG (_STR("Thread2\n"));
        failures++;
    }

    /* Create Thread 3 */
    if (atomThreadCreate(&tcb[2], TEST_THREAD_PRIO, test_thread_func, 3,
          &test_thread_stack[2][0],
          TEST_THREAD_STACK_SIZE, TRUE) != ATOM_OK)
    {
        /* Fail */
        ATOMLOG (_STR("Thread3\n"));
        failures++;
    }

    /* Sleep for 10 seconds allowing the three threads to run tests */
    if (atomTimerDelay(TEST_PERIOD_SECS * SYSTEM_TICKS_PER_SEC) != ATOM_OK)
    {
        ATOMLOG (_STR("Period\n"));
        failures++;
    }

    /* Add the per-thread failure count to the main count */
    failures += g_failure_cnt[0] + g_failure_cnt[1] + g_failure_cnt[2];

    /* Check thread stack usage (if enabled) */
#ifdef ATOM_STACK_CHECKING
    {
        uint32_t used_bytes, free_bytes;
        int thread;

        /* Check all threads */
        for (thread = 0; thread < NUM_TEST_THREADS; thread++)
        {
            /* Check thread stack usage */
            if (atomThreadStackCheck (&tcb[thread], &used_bytes, &free_bytes) != ATOM_OK)
            {
                ATOMLOG (_STR("StackCheck\n"));
                failures++;
            }
            else
            {
                /* Check the thread did not use up to the end of stack */
                if (free_bytes == 0)
                {
                    ATOMLOG (_STR("StackOverflow %d\n"), thread);
                    failures++;
                }

                /* Log the stack usage */
#ifdef TESTS_LOG_STACK_USAGE
                ATOMLOG (_STR("StackUse:%d\n"), (int)used_bytes);
#endif
            }
        }
    }
#endif

    /* Quit */
    return failures;

}


/**
 * \b test_thread_func
 *
 * Entry point for test thread. The same thread entry point is used for all
 * three test threads, with the thread number/ID (1-3) passed as the entry
 * point parameter.
 *
 * @param[in] param Thread number (1,2,3)
 *
 * @return None
 */
static void test_thread_func (uint32_t param)
{
    uint8_t thread_id;
    uint32_t start_time, end_time;

    /* Thread ID is passed through the function parameter */
    thread_id = (uint8_t)param;

    /*
     * Sleep for 1 tick to ensure that the thread starts near
     * a timer tick boundary. This ensures that the system
     * tick does not advance between the atomTimeGet() call
     * and the actual atomTimerDelay() call being made.
     */
    atomTimerDelay (1);

    /* Loop running the test forever */
    while (1)
    {
        /* Record the start time */
        start_time = atomTimeGet();

        /* Sleep for n ticks, where n is the thread ID */
        if (atomTimerDelay(thread_id) != ATOM_OK)
        {
            g_failure_cnt[thread_id-1]++;
        }
        else
        {
            /* Record the time we woke up */
            end_time = atomTimeGet();

            /* Check that time has advanced by exactly n ticks */
            if ((end_time - start_time) != thread_id)
            {
                g_failure_cnt[thread_id-1]++;
            }
        }
    }
}
