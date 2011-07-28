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


/* Number of test loops for stress-test */
#define NUM_TEST_LOOPS      10000


/* Number of test threads */
#define NUM_TEST_THREADS      4


/* Test OS objects */
static ATOM_SEM sem1, sem2;
static ATOM_TCB tcb[NUM_TEST_THREADS];
static uint8_t test_thread_stack[NUM_TEST_THREADS][TEST_THREAD_STACK_SIZE];


/*
 * Global failure count (can be updated by test threads but is
 * protected by an interrupt lockout).
 */
static volatile int g_failures;


/* Forward declarations */
static void test_thread_func (uint32_t param);


/**
 * \b test_start
 *
 * Start semaphore test.
 *
 * Stress-tests semaphore Get and Put operations. Four threads are created which are
 * continually Getting and Putting the same semaphore, with no time delays between
 * each Get/Put. A single semaphore is used, which is initially created with a count
 * of two, meaning that at any one time there should be threads blocking and not
 * blocking. This ensures that the stress-test covers multiple threads accessing the
 * semaphore APIs simultaneously, as well as usage of the APIs while other threads
 * are blocking on the semaphore.
 *
 * @retval Number of g_failures
 */
uint32_t test_start (void)
{
    CRITICAL_STORE;
    int finish_cnt;

    /* Default to zero g_failures */
    g_failures = 0;

    /* Create sem to stress with count two */
    if (atomSemCreate (&sem1, 2) != ATOM_OK)
    {
        ATOMLOG (_STR("Error creating test semaphore 1\n"));
        g_failures++;
    }
    /* Create sem to receive thread-finished notification */
    else if (atomSemCreate (&sem2, 0) != ATOM_OK)
    {
        ATOMLOG (_STR("Error creating test semaphore 1\n"));
        g_failures++;
    }
    else
    {
        /* Create Thread 1 */
        if (atomThreadCreate(&tcb[0], TEST_THREAD_PRIO, test_thread_func, 1,
              &test_thread_stack[0][0],
              TEST_THREAD_STACK_SIZE, TRUE) != ATOM_OK)
        {
            /* Fail */
            ATOMLOG (_STR("Error creating test thread\n"));
            CRITICAL_START ();
            g_failures++;
            CRITICAL_END ();
        }

        /* Create Thread 2 */
        if (atomThreadCreate(&tcb[1], TEST_THREAD_PRIO, test_thread_func, 2,
              &test_thread_stack[1][0],
              TEST_THREAD_STACK_SIZE, TRUE) != ATOM_OK)
        {
            /* Fail */
            ATOMLOG (_STR("Error creating test thread\n"));
            CRITICAL_START ();
            g_failures++;
            CRITICAL_END ();
        }

        /* Create Thread 3 */
        if (atomThreadCreate(&tcb[2], TEST_THREAD_PRIO, test_thread_func, 3,
              &test_thread_stack[2][0],
              TEST_THREAD_STACK_SIZE, TRUE) != ATOM_OK)
        {
            /* Fail */
            ATOMLOG (_STR("Error creating test thread\n"));
            CRITICAL_START ();
            g_failures++;
            CRITICAL_END ();
        }

        /* Create Thread 4 */
        if (atomThreadCreate(&tcb[3], TEST_THREAD_PRIO, test_thread_func, 4,
              &test_thread_stack[3][0],
              TEST_THREAD_STACK_SIZE, TRUE) != ATOM_OK)
        {
            /* Fail */
            ATOMLOG (_STR("Error creating test thread\n"));
            CRITICAL_START ();
            g_failures++;
            CRITICAL_END ();
        }

        /*
         * All four threads will now be performing Gets/Puts on sem1.
         * When they have finished they will post sem2, so we wait
         * until sem2 is posted four times.
         */
        finish_cnt = 0;
        while (1)
        {
            /*
             * Attempt to Get sem2. When we have managed to get
             * the semaphore four times, it must have been posted
             * by all four threads.
             */
            if (atomSemGet (&sem2, 0) == ATOM_OK)
            {
                /* Increment our count of finished threads */
                finish_cnt++;

                /* Check if all four threads have now posted sem2 */
                if (finish_cnt == 4)
                {
                    break;
                }
            }
        }

        /* Delete semaphores, test finished */
        if (atomSemDelete (&sem1) != ATOM_OK)
        {
            ATOMLOG (_STR("Delete failed\n"));
            CRITICAL_START ();
            g_failures++;
            CRITICAL_END ();
        }
        if (atomSemDelete (&sem2) != ATOM_OK)
        {
            ATOMLOG (_STR("Delete failed\n"));
            CRITICAL_START ();
            g_failures++;
            CRITICAL_END ();
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
                g_failures++;
            }
            else
            {
                /* Check the thread did not use up to the end of stack */
                if (free_bytes == 0)
                {
                    ATOMLOG (_STR("StackOverflow %d\n"), thread);
                    g_failures++;
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
    return g_failures;

}


/**
 * \b test_thread_func
 *
 * Entry point for test thread. The same thread entry point is used for all
 * four test threads.
 *
 * @param[in] param Unused (optional thread entry parameter)
 *
 * @return None
 */
static void test_thread_func (uint32_t param)
{
    uint32_t loop_cnt;
    uint8_t status;
    CRITICAL_STORE;

    /* Compiler warnings */
    param = param;

    /* Run a Get/Put pair many times */
    loop_cnt = NUM_TEST_LOOPS;
    while (loop_cnt--)
    {
        if ((status = atomSemGet (&sem1, 0)) != ATOM_OK)
        {
            /* Error getting semaphore, notify the status code */
            ATOMLOG (_STR("G%d\n"), status);
            CRITICAL_START ();
            g_failures++;
            CRITICAL_END ();
            break;
        }
        else if ((status = atomSemPut (&sem1)) != ATOM_OK)
        {
            /* Error putting semaphore, notify the status code */
            ATOMLOG (_STR("P%d\n"), status);
            CRITICAL_START ();
            g_failures++;
            CRITICAL_END ();
            break;
        }
    }

    /* Post sem2 to notify the main thread we're finished */
    if (atomSemPut (&sem2) != ATOM_OK)
    {
        ATOMLOG (_STR("Sem2 putfail\n"));
        CRITICAL_START ();
        g_failures++;
        CRITICAL_END ();
    }

    /* Loop forever */
    while (1)
    {
        atomTimerDelay (SYSTEM_TICKS_PER_SEC);
    }
}
