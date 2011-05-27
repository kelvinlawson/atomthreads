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
#include "atommutex.h"


/* Number of test threads */
#define NUM_TEST_THREADS      1


/* Test OS objects */
static ATOM_MUTEX mutex1;
static ATOM_TCB tcb[NUM_TEST_THREADS];
static uint8_t test_thread_stack[NUM_TEST_THREADS][TEST_THREAD_STACK_SIZE];


/* Global shared data */
static volatile int shared_data;


/* Forward declarations */
static void testCallback (POINTER cb_data);
static void test_thread_func (uint32_t param);


/**
 * \b test_start
 *
 * Start mutex test.
 *
 * This tests the ownership checks of the mutex library. Only threads
 * which own a mutex can release it. It should not be possible to
 * release a mutex if it is not owned by any thread, is owned by a
 * different thread, or at interrupt context. We test here that all
 * three cases are trapped.
 *
 * @retval Number of failures
 */
uint32_t test_start (void)
{
    int failures;
    ATOM_TIMER timer_cb;

    /* Default to zero failures */
    failures = 0;

    /* Create mutex */
    if (atomMutexCreate (&mutex1) != ATOM_OK)
    {
        ATOMLOG (_STR("Error creating mutex\n"));
        failures++;
    }
    else
    {
        /* Initialise the shared_data to zero */
        shared_data = 0;

        /* Attempt to release the mutex when not owned by any thread */
        if (atomMutexPut (&mutex1) != ATOM_ERR_OWNERSHIP)
        {
            ATOMLOG (_STR("Release error\n"));
            failures++;
        }

        /* Create second thread */
        else if (atomThreadCreate(&tcb[0], TEST_THREAD_PRIO, test_thread_func, 1,
              &test_thread_stack[0][0],
              TEST_THREAD_STACK_SIZE, TRUE) != ATOM_OK)
        {
            /* Fail */
            ATOMLOG (_STR("Error creating test thread\n"));
            failures++;
        }

        /*
         * The second thread has now been created and should take ownership
         * of the mutex. We wait a while and check that shared_data has been
         * modified, which proves to us that the thread has taken the mutex.
         */
        atomTimerDelay (SYSTEM_TICKS_PER_SEC/4);
        if (shared_data != 1)
        {
            ATOMLOG (_STR("Shared data unmodified\n"));
            failures++;
        }

        /* Check successful so far */
        if (failures == 0)
        {
            /*
             * Attempt to release the mutex again now that it is owned
             * by another thread.
             */
            if (atomMutexPut (&mutex1) != ATOM_ERR_OWNERSHIP)
            {
                ATOMLOG (_STR("Release error 2\n"));
                failures++;
            }

            /* Finally check that the mutex cannot be released from an ISR */

            /* Fill out the timer callback request structure */
            timer_cb.cb_func = testCallback;
            timer_cb.cb_data = NULL;
            timer_cb.cb_ticks = SYSTEM_TICKS_PER_SEC;

            /* Request the timer callback to run in one second */
            if (atomTimerRegister (&timer_cb) != ATOM_OK)
            {
                ATOMLOG (_STR("Error registering timer\n"));
                failures++;
            }

            /*
             * Wait two seconds for shared_date to be set to 2
             * indicating success. This happens if the timer
             * callback received the expected ownership error
             * when attempting to release the mutex.
             */
            else
            {
                atomTimerDelay (2 * SYSTEM_TICKS_PER_SEC);
                if (shared_data != 2)
                {
                    ATOMLOG (_STR("Context check failed\n"));
                    failures++;
                }
            }
        }

        /* Delete mutex, test finished */
        if (atomMutexDelete (&mutex1) != ATOM_OK)
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

    /* Block on the mutex */
    if ((status = atomMutexGet (&mutex1, 0)) != ATOM_OK)
    {
        /* Error getting mutex, notify the status code */
        ATOMLOG (_STR("G%d\n"), status);
    }

    /* Got the mutex */
    else
    {
        /* Set shared_data to signify that we think we have the mutex */
        shared_data = 1;
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
 * Attempt an atomMutexPut() on mutex1 from interrupt context.
 * Should receive an ATOM_ERR_OWNERSHIP error. Sets shared_data
 * to 2 if passes.
 *
 * @param[in] cb_data Not used
 */
static void testCallback (POINTER cb_data)
{
    /* Check the return value from atomMutexPut() */
    if (atomMutexPut(&mutex1) == ATOM_ERR_OWNERSHIP)
    {
        /* Received the error we expected, set shared_data to notify success */
        shared_data = 2;
    }
    else
    {
        /* Did not get expected error, don't set shared_data signifying fail */
    }

}
