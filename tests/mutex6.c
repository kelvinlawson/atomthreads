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


/* Number of times to lock the mutex during test */
#define TEST_LOCK_CNT       250


/* Number of test threads */
#define NUM_TEST_THREADS      1


/* Test OS objects */
static ATOM_MUTEX mutex1;
static ATOM_TCB tcb[NUM_TEST_THREADS];
static uint8_t test_thread_stack[NUM_TEST_THREADS][TEST_THREAD_STACK_SIZE];


/* Global shared data protected by mutex */
static volatile int shared_data;


/* Forward declarations */
static void test_thread_func (uint32_t param);


/**
 * \b test_start
 *
 * Start mutex test.
 *
 * This tests the lock count of a mutex. The mutex object should
 * count the number of times a thread has locked the mutex and
 * not fully release it for use by another thread until it has
 * been released the same number of times it was locked.
*
 * @retval Number of failures
 */
uint32_t test_start (void)
{
    int failures;
    int i;

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

        /* Take the mutex several times */
        for (i = 0; i < TEST_LOCK_CNT; i++)
        {
            if (atomMutexGet (&mutex1, 0) != ATOM_OK)
            {
                ATOMLOG (_STR("Error taking mutex\n"));
                failures++;
                break;
            }

        }

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
         * the mutex until we release it. We wait a while and check that
         * shared_data has not been modified.
         */
        for (i = 0; i < 4; i++)
        {
            /*
             * Sleep for a while to give the second thread a chance to
             * modify shared_data, thought it shouldn't until we
             * release the mutex enough times.
             */
            atomTimerDelay (SYSTEM_TICKS_PER_SEC/4);

            /* Check shared data. The second thread always sets it to one. */
            if (shared_data != 0)
            {
                ATOMLOG (_STR("Shared data modified\n"));
                failures++;
                break;
            }
        }

        /* Check successful so far */
        if (failures == 0)
        {
            /*
             * Release the mutex TEST_LOCK_CNT-1 times, after which we
             * should still own the mutex (until we release one more time).
             */
            for (i = 0; i < TEST_LOCK_CNT-1; i++)
            {
                if (atomMutexPut (&mutex1) != ATOM_OK)
                {
                    ATOMLOG (_STR("Failed release\n"));
                    failures++;
                }

            }

            /*
             * Wait a little while then check that shared_data has
             * not been modified (we should still own the mutex).
             */
            atomTimerDelay (SYSTEM_TICKS_PER_SEC/4);
            if (shared_data != 0)
            {
                ATOMLOG (_STR("Expected unmodified\n"));
                failures++;
            }

            /*
             * Release the mutex one more time, after which we should no
             * longer own the mutex (and wake up the second thread).
             */
            if (atomMutexPut (&mutex1) != ATOM_OK)
            {
                ATOMLOG (_STR("Failed release\n"));
                failures++;
            }

            /*
             * Wait a little while then check that shared_data has
             * been modified by the second thread.
             */
            atomTimerDelay (SYSTEM_TICKS_PER_SEC/4);
            if (shared_data != 1)
            {
                ATOMLOG (_STR("Expected modified\n"));
                failures++;
            }
        }

        /*
         * Finally attempt to release the mutex one more time, while
         * we no longer own the mutex. Either the second thread will
         * have ownership of it, or no thread will have ownership.
         * In both cases we expect to get an ownership error when we
         * attempt to release it.
         */
        if (atomMutexPut (&mutex1) != ATOM_ERR_OWNERSHIP)
        {
            ATOMLOG (_STR("Failed locked+1 release\n"));
            failures++;
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

    /* Repeatedly attempt to get the mutex and set shared_data to 1 */
    while (1)
    {
        /* Block on the mutex */
        if ((status = atomMutexGet (&mutex1, 0)) != ATOM_OK)
        {
            /* Error getting mutex, notify the status code */
            ATOMLOG (_STR("G%d\n"), status);
            break;
        }

        /* Got the mutex */
        else
        {
            /* Set shared_data to signify that we think we have the mutex */
            shared_data = 1;

            /* Release the mutex allowing the main thread to take it again */
            if ((status = atomMutexPut (&mutex1)) != ATOM_OK)
            {
                /* Error putting mutex, notify the status code */
                ATOMLOG (_STR("P%d\n"), status);
                break;
            }
        }

    }

    /* Loop forever - we only reach here on error */
    while (1)
    {
        atomTimerDelay (SYSTEM_TICKS_PER_SEC);
    }
}
