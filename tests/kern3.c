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
#define NUM_TEST_THREADS      2


/* Test OS objects */
static ATOM_TCB tcb[NUM_TEST_THREADS];
static uint8_t test_thread_stack[NUM_TEST_THREADS][TEST_THREAD_STACK_SIZE];


/* Test global data (one per thread) */
static volatile int running_flag[2];
static volatile int sleep_request[2];


/* Forward declarations */
static void test_thread_func (uint32_t param);


/**
 * \b test_start
 *
 * Start kernel test.
 *
 * This tests the scheduling of threads at different priorities, and
 * preemption of lower priority threads by higher priority threads.
 *
 * Much of this functionality is already tested implicitly by the
 * semaphore, mutex tests etc but we repeat it here within the kernel
 * tests for completeness.
 *
 * Two threads are created at different priorities, with each thread
 * setting a running flag whenever it runs. We check that when the
 * higher priority thread is ready to run, only the higher priority
 * thread's running flag is set (even though the lower priority
 * thread should also be setting it at this time). This checks that
 * the scheduler is correctly prioritising thread execution.
 *
 * The test also exercises preemption, by disabling setting of the
 * running flag in the higher priority thread for a period. During
 * this time the higher priority thread repeatedly sleeps for one
 * system tick then wakes up to check the sleep-request flag again.
 * Every time the higher priority thread wakes up, it has preempted
 * the lower priority thread (which is always running). By ensuring
 * that the higher priority thread is able to start running again
 * after one of these periods (through checking the running flag)
 * we prove that the preemption has worked.
 *
 * @retval Number of failures
 */
uint32_t test_start (void)
{
    int failures;
    int i;

    /* Default to zero failures */
    failures = 0;

    /* Initialise global data */
    running_flag[0] = running_flag[1] = FALSE;
    sleep_request[0] = sleep_request[1] = FALSE;

    /* Create low priority thread */
    if (atomThreadCreate (&tcb[0], 253, test_thread_func, 0,
            &test_thread_stack[0][0],
            TEST_THREAD_STACK_SIZE, TRUE) != ATOM_OK)
    {
        ATOMLOG (_STR("Bad thread create\n"));
        failures++;
    }

    /* Create high priority thread */
    else if (atomThreadCreate (&tcb[1], 252, test_thread_func, 1,
            &test_thread_stack[1][0],
            TEST_THREAD_STACK_SIZE, TRUE) != ATOM_OK)
    {
        ATOMLOG (_STR("Bad thread create\n"));
        failures++;
    }

    /* Repeat test a few times */
    for (i = 0; i < 8; i++)
    {
        /* Make the higher priority thread sleep */
        sleep_request[1] = TRUE;

        /* Sleep a little to make sure the thread sees the sleep request */
        atomTimerDelay (SYSTEM_TICKS_PER_SEC/4);

        /* Reset the running flag for both threads */
        running_flag[0] = running_flag[1] = FALSE;

        /* Sleep a little to give any running threads time to set their flag */
        atomTimerDelay (SYSTEM_TICKS_PER_SEC/4);

        /* Check only the low priority thread has run since we reset the flags */
        if ((running_flag[0] != TRUE) || (running_flag[1] != FALSE))
        {
            ATOMLOG (_STR("Lo%d %d/%d\n"), i, running_flag[0], running_flag[1]);
            failures++;
            break;
        }
        else
        {
            /*
             * We have confirmed that only the ready thread has been running.
             * Now check that if we wake up the high priority thread, the
             * low priority one stops running and only the high priority one
             * does.
             */

            /* Tell the higher priority thread to stop sleeping */
            sleep_request[1] = FALSE;

            /* Reset the running flag for both threads */
            running_flag[0] = running_flag[1] = FALSE;

            /* Sleep a little to give any running threads time to set their flag */
            atomTimerDelay (SYSTEM_TICKS_PER_SEC/4);

            /* Check only the high priority thread has run since we reset the flags */
            if ((running_flag[1] != TRUE) || (running_flag[0] != FALSE))
            {
                ATOMLOG (_STR("Hi%d/%d\n"), running_flag[0], running_flag[1]);
                failures++;
                break;
            }
            else
            {
                /*
                 * We have confirmed that the high priority thread has preempted the
                 * low priority thread, and remain running while never scheduling
                 * the lower one back in.
                 */
            }
        }
    }

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
 * @param[in] param Thread ID (0 = low prio, 1 = high prio)
 *
 * @return None
 */
static void test_thread_func (uint32_t param)
{
    int thread_id;

    /* Pull out thread ID */
    thread_id = (int)param;

    /* Run forever */
    while (1)
    {
        /* If this thread is requested to sleep, sleep until told to stop */
        if (sleep_request[thread_id])
        {
            atomTimerDelay (1);
        }
        else
        {
            /* Otherwise set running flag for this thread */
            running_flag[thread_id] = TRUE;
        }
    }
}
