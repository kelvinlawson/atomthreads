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
#define NUM_TEST_THREADS      1


/* Test OS objects */
static ATOM_SEM sem1, sem2;
static ATOM_TCB tcb[NUM_TEST_THREADS];
static uint8_t test_thread_stack[NUM_TEST_THREADS][TEST_THREAD_STACK_SIZE];


/* Forward declarations */
static void test_thread_func (uint32_t param);


/**
 * \b test_start
 *
 * Start semaphore test.
 *
 * This tests basic synchronisation between threads using a semaphore.
 *
 * A second thread is created, which blocks on a semaphore until posted
 * by the main thread. Testing all aspects of this transaction proves
 * the basic usage of semaphores as a synchronisation mechanism
 * between threads.
 *
 * @retval Number of failures
 */
uint32_t test_start (void)
{
    int failures;

    /* Default to zero failures */
    failures = 0;

    /* Create sem with count zero for second thread to block on */
    if (atomSemCreate (&sem1, 0) != ATOM_OK)
    {
        ATOMLOG (_STR("Error creating test semaphore 1\n"));
        failures++;
    }
    /* Create sem to receive test-passed notification */
    else if (atomSemCreate (&sem2, 0) != ATOM_OK)
    {
        ATOMLOG (_STR("Error creating test semaphore 1\n"));
        failures++;
    }
    else
    {
        /* Create second thread */
        if (atomThreadCreate(&tcb[0], TEST_THREAD_PRIO, test_thread_func, 1,
              &test_thread_stack[0][0],
              TEST_THREAD_STACK_SIZE, TRUE) != ATOM_OK)
        {
            /* Fail */
            ATOMLOG (_STR("Error creating test thread\n"));
            failures++;
        }

        /*
         * The second thread has now been created and should block on
         * sem1 until we increment it to 1 (sem1 was created with a
         * count of zero). Once the second thread has stopped blocking
         * on sem1, it will post sem2 to notify us that it has finished.
         *
         * We can test the count of sem2 before and after posting sem1
         * to ensure that it was the posting of sem1 which caused the
         * second thread to wake up.
         */

        /*
         * Wait a little while, then check the second thread hasn't
         * already posted sem2, i.e. did not wait until sem1 was posted.
         */
        atomTimerDelay (SYSTEM_TICKS_PER_SEC/4);
        if (atomSemGet (&sem2, -1) != ATOM_WOULDBLOCK)
        {
            ATOMLOG (_STR("Did not wait\n"));
            failures++;
        }

        /* Post sem1 to stop the second thread blocking */
        else if (atomSemPut (&sem1) != ATOM_OK)
        {
            ATOMLOG (_STR("Put fail\n"));
            failures++;
        }

        /* Now check that the second thread has woken up and posted sem2 */
        else
        {
            /* Give the thread some time to wake up and post sem2 */
            atomTimerDelay (SYSTEM_TICKS_PER_SEC/4);

            /* Check that the second thread has now woken and posted sem2 */
            if (atomSemGet (&sem2, -1) != ATOM_OK)
            {
                ATOMLOG (_STR("Sem2 not posted\n"));
                failures++;
            }

        }

        /* Delete semaphores, test finished */
        if (atomSemDelete (&sem1) != ATOM_OK)
        {
            ATOMLOG (_STR("Delete failed\n"));
            failures++;
        }
        if (atomSemDelete (&sem2) != ATOM_OK)
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
 * @param[in] param Unused (optional thread entry parameter)
 *
 * @return None
 */
static void test_thread_func (uint32_t param)
{
    uint8_t status;

    /* Compiler warnings */
    param = param;

    /* Block on sem1. Main thread will post when we should wake up. */
    if ((status = atomSemGet (&sem1, 0)) != ATOM_OK)
    {
        /* Error getting semaphore, notify the status code */
        ATOMLOG (_STR("G%d\n"), status);
    }

    /* Post sem2 to notify that we received the sem1 notification */
    else if ((status = atomSemPut (&sem2)) != ATOM_OK)
    {
        /* Error putting semaphore, notify the status code */
        ATOMLOG (_STR("P%d\n"), status);
    }

    /* Loop forever */
    while (1)
    {
        atomTimerDelay (SYSTEM_TICKS_PER_SEC);
    }
}
