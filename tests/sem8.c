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
#include "atomsem.h"


/* Number of test threads */
#define NUM_TEST_THREADS      3


/* Test OS objects */
static ATOM_SEM sem1;
static ATOM_TCB tcb[NUM_TEST_THREADS];
static uint8_t test_thread_stack[NUM_TEST_THREADS][TEST_THREAD_STACK_SIZE];


/* Test running flag */
static volatile int test_running;


/* Forward declarations */
static void test_thread_func (uint32_t param);
static void testCallback (POINTER cb_data);


/**
 * \b test_start
 *
 * Start semaphore test.
 *
 * This stress-tests atomSemGet()/atomSemPut() with a single thread
 * continually calling atomSemGet() and several contexts continually
 * calling atomSemPut(). This stresses in particular the atomSemPut()
 * API, with three threads at different priorities posting
 * simultaneously, as well as a timer callback posting it from
 * interrupt context. In all cases the same semaphore is posted.
 *
 * This tests the thread-safety and interrupt-safety of the semaphore
 * APIs, and particularly the atomSemPut() function.
 *
 * @retval Number of failures
 */
uint32_t test_start (void)
{
    int failures;
    uint32_t end_time;
    ATOM_TIMER timer_cb;

    /* Default to zero failures */
    failures = 0;

    /* Create sem with count of zero */
    if (atomSemCreate (&sem1, 0) != ATOM_OK)
    {
        ATOMLOG (_STR("Error creating test semaphore 1\n"));
        failures++;
    }
    else
    {
        /* Set the test running flag */
        test_running = TRUE;

        /*
         * Fill out a timer callback request structure. Pass the timer
         * structure itself so that the callback can requeue the request.
         */
        timer_cb.cb_func = testCallback;
        timer_cb.cb_data = &timer_cb;
        timer_cb.cb_ticks = 1;

        /*
         * Request a timer callback to run in one tick's time. The callback
         * will automatically queue another so that this happens repeatedly
         * until the test is flagged as finished.
         */
        if (atomTimerRegister (&timer_cb) != ATOM_OK)
        {
            ATOMLOG (_STR("Error registering timer\n"));
            failures++;
        }

        /* Create thread 1: Higher priority than main thread so should sleep */
        else if (atomThreadCreate(&tcb[0], TEST_THREAD_PRIO - 1, test_thread_func, 1,
              &test_thread_stack[0][0],
              TEST_THREAD_STACK_SIZE, TRUE) != ATOM_OK)
        {
            /* Fail */
            ATOMLOG (_STR("Error creating test thread 1\n"));
            failures++;
        }

        /* Create thread 2: Same priority as main thread so should not sleep */
        else if (atomThreadCreate(&tcb[1], TEST_THREAD_PRIO, test_thread_func, 0,
              &test_thread_stack[1][0],
              TEST_THREAD_STACK_SIZE, TRUE) != ATOM_OK)
        {
            /* Fail */
            ATOMLOG (_STR("Error creating test thread 2\n"));
            failures++;
        }

        /* Create thread 3: Same priority as main thread so should not sleep */
        else if (atomThreadCreate(&tcb[2], TEST_THREAD_PRIO + 1, test_thread_func, 0,
              &test_thread_stack[2][0],
              TEST_THREAD_STACK_SIZE, TRUE) != ATOM_OK)
        {
            /* Fail */
            ATOMLOG (_STR("Error creating test thread 3\n"));
            failures++;
        }

        /* The test threads have now all been created */
        else
        {
            /*
             * Continually decrement the semaphore while the test threads
             * and timer callbacks are continually incrementing it. The
             * test finishes after this runs without error for 5 seconds.
             */
            end_time = atomTimeGet() + (5 * SYSTEM_TICKS_PER_SEC);
            while (atomTimeGet() < end_time)
            {
                /* Decrement the semaphore */
                if (atomSemGet (&sem1, SYSTEM_TICKS_PER_SEC) != ATOM_OK)
                {
                    ATOMLOG (_STR("SemGet\n"));
                    failures++;
                    break;
                }
            }

            /* Test finished, stop the other threads and timer callbacks */
            test_running = FALSE;

            /*
             * Wait before finishing: a timer callback could be due
             * shortly, and we allocated the timer structure off the
             * local call stack.
             */
            atomTimerDelay(2);
        }

        /* Delete semaphores, test finished */
        if (atomSemDelete (&sem1) != ATOM_OK)
        {
            ATOMLOG (_STR("Delete failed\n"));
            failures++;
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
 * @param[in] param sleep_flag passed through here
 *
 * @return None
 */
static void test_thread_func (uint32_t param)
{
    uint8_t status;
    int sleep_flag, count, failures;

    /* Were we requested to sleep occasionally? */
    sleep_flag = (int)param;

    /* Run until the main thread sets the finish flag or we get an error */
    failures = 0;
    while ((test_running == TRUE) && (failures == 0))
    {
        /* Post the semaphore 50 times */
        count = 50;
        while (count--)
        {
            /*
             * Post the semaphore. Allow overflows as these are likely
             * to occur when so many threads are posting the same
             * semaphore continually.
             */
            status = atomSemPut (&sem1);
            if ((status != ATOM_OK) && (status != ATOM_ERR_OVF))
            {
                ATOMLOG (_STR("Put\n"));
                failures++;
                break;
            }
        }

        /*
         * If requested to do so, sleep for a tick. This only happens on threads which
         * are higher priority than the main test thread, and is necessary to allow
         * the main thread to actually run. For better stress-testing, same or lower
         * priority threads do not sleep.
         */
        if (sleep_flag)
        {
            atomTimerDelay (1);
        }
    }

    /* Loop forever */
    while (1)
    {
        atomTimerDelay (SYSTEM_TICKS_PER_SEC);
    }
}


/**
 * \b testCallback
 *
 * Post the semaphore from interrupt context. This will be occurring while
 * atomSemPut() calls for the other threads are in progress, because it is
 * called by the system tick ISR. This tests that the APIs lockout
 * interrupts where necessary.
 *
 * Automatically requeues itself for one tick in the future, so this
 * continually fires until the finish flag is set.
 *
 * @param[in] cb_data Pointer to the original ATOM_TIMER structure
 */
static void testCallback (POINTER cb_data)
{
    ATOM_TIMER *ptimer;
    uint8_t status;

    /* Pull out the original timer request */
    ptimer = (ATOM_TIMER *)cb_data;

    /* Post sem1 */
    status = atomSemPut (&sem1);
    if ((status != ATOM_OK) && (status != ATOM_ERR_OVF))
    {
        /* Error */
    }

    /* Enqueue another timer callback in one tick's time */
    if (test_running == TRUE)
    {
        /* Update the callback time and requeue */
        ptimer->cb_ticks = 1;
        if (atomTimerRegister (ptimer) != ATOM_OK)
        {
        }
    }
    else
    {
        /* Test finished, no more will be queued */
    }

}
