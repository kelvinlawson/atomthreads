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
#include "atomtests.h"


/* Number of test threads */
#define NUM_TEST_THREADS      4


/* Test OS objects */
static ATOM_TCB tcb[NUM_TEST_THREADS];
static uint8_t test_thread_stack[NUM_TEST_THREADS][TEST_THREAD_STACK_SIZE];


/* Test global data (one per thread) */
static uint32_t volatile last_time;
static int volatile last_thread_id;
static volatile int switch_cnt;
static uint32_t volatile failure_cnt[4];
static int volatile test_started;


/* Forward declarations */
static void test_thread_func (uint32_t param);


/**
 * \b test_start
 *
 * Start kernel test.
 *
 * This tests the round-robin timeslicing of same priority threads.
 *
 * Four threads are created (over and above the main test thread).
 * The main test thread sleeps for the duration of the entire test.
 * While the test is ongoing, all four threads are running
 * continuously at the same priority. They each check that whenever
 * the system tick (atomTimeGet()) changes, it has moved on 1 tick
 * since the last time the tick was checked, and that the previous
 * thread is the one created before itself. In the case of the first
 * thread created, the previous thread should be the last thread
 * created. This proves that on every tick the four threads get a
 * schedule timeslice equally, and in the same order throughout the
 * duration of the test.
 *
 * @retval Number of failures
 */
uint32_t test_start (void)
{
    int failures;

    /* Default to zero failures */
    failures = 0;

    /* Initialise global data */
    last_time = 0;
    last_thread_id = -1;
    failure_cnt[0] = failure_cnt[1] = failure_cnt[2] = failure_cnt[3] = 0;

    /* Set test as not started until all threads are ready to go */
    test_started = FALSE;

    /* 
     * Create all four threads at the same priority as each other.
     * They are given a lower priority than this thread, however,
     * to ensure that once this thread wakes up to stop the test it
     * can do so without confusing the scheduling tests by having
     * a spell in which this thread was run.
     */
    if (atomThreadCreate (&tcb[0], TEST_THREAD_PRIO + 1, test_thread_func, 0,
            &test_thread_stack[0][0],
            TEST_THREAD_STACK_SIZE, TRUE) != ATOM_OK)
    {
        ATOMLOG (_STR("Bad thread create\n"));
        failures++;
    }
    else if (atomThreadCreate (&tcb[1], TEST_THREAD_PRIO + 1, test_thread_func, 1,
            &test_thread_stack[1][0],
            TEST_THREAD_STACK_SIZE, TRUE) != ATOM_OK)
    {
        ATOMLOG (_STR("Bad thread create\n"));
        failures++;
    }
    else if (atomThreadCreate (&tcb[2], TEST_THREAD_PRIO + 1, test_thread_func, 2,
            &test_thread_stack[2][0],
            TEST_THREAD_STACK_SIZE, TRUE) != ATOM_OK)
    {
        ATOMLOG (_STR("Bad thread create\n"));
        failures++;
    }
    else if (atomThreadCreate (&tcb[3], TEST_THREAD_PRIO + 1, test_thread_func, 3,
            &test_thread_stack[3][0],
            TEST_THREAD_STACK_SIZE, TRUE) != ATOM_OK)
    {
        ATOMLOG (_STR("Bad thread create\n"));
        failures++;
    }

    /* Start the test */
    test_started = TRUE;

    /* Sleep for 5 seconds during test */
    atomTimerDelay (5 * SYSTEM_TICKS_PER_SEC);

    /* Stop the test */
    test_started = FALSE;

    /* Sleep for tests to complete */
    atomTimerDelay (SYSTEM_TICKS_PER_SEC/4);

    /* Count any failures from test threads */
    failures += failure_cnt[0] + failure_cnt[1] + failure_cnt[2] + failure_cnt[3];

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
 * Entry point for test thread.
 *
 * @param[in] param Thread ID (0 to 3)
 *
 * @return None
 */
static void test_thread_func (uint32_t param)
{
    int thread_id, expected_thread;
    int time_error, thread_error;
    uint32_t new_time;
    CRITICAL_STORE;

    /* Pull out thread ID */
    thread_id = (int)param;

    /* Run forever */
    while (1)
    {
        /* Check if test is currently in operation */
        if (test_started)
        {
            /*
             * If the system time has ticked over, check that the currently
             * running thread is not the one that was running last tick.
             */

            /* Default to no error this time */
            time_error = thread_error = FALSE;

            /* Do the whole operation with interrupts locked out */
            CRITICAL_START();

            /* Check if time has ticked over */
            new_time = atomTimeGet();

            /* Only perform the check if this is not the first thread to run */
            if ((last_time != 0) && (last_thread_id != -1))
            {
                /* Check if the time has ticked over */
                if (new_time != last_time)
                {
                    /* Check time only ticked over by 1 */
                    if ((new_time - last_time) != 1)
                    {
                        time_error = 1;
                    }

                    /*
                     * We are expecting the previous thread to be our thread_id
                     * minus one.
                     */
                    expected_thread = thread_id - 1;
                    if (expected_thread == -1)
                    {
                        expected_thread = 3;
                    }

                    /* Check that the last thread was the expected one */
                    if (last_thread_id != expected_thread)
                    {
                        thread_error = TRUE;
                    }

                    /* Increment the switch count */
                    switch_cnt++;
                }
            }

            /* Store the currently-running thread as the last-running */
            last_thread_id = thread_id;
            last_time = new_time;

            /* Finished with the interrupt lockout */
            CRITICAL_END();

            /* If we got an error above, increment the total failure count */
            if (test_started && (thread_error || time_error))
            {
                failure_cnt[thread_id]++;
                ATOMLOG(_STR("T%d\n"), thread_id);
            }
        }
    }
}
