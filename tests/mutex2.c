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
#include "atommutex.h"
#include "atomtests.h"


/* Number of test threads */
#define NUM_TEST_THREADS      1


/* Test OS objects */
static ATOM_MUTEX mutex1, mutex2;
static ATOM_TCB tcb[NUM_TEST_THREADS];
static uint8_t test_thread_stack[NUM_TEST_THREADS][TEST_THREAD_STACK_SIZE];


/* Test result tracking */
static volatile int g_result, g_owned;


/* Forward declarations */
static void testCallback (POINTER cb_data);
static void test_thread_func (uint32_t param);


/**
 * \b test_start
 *
 * Start mutex test.
 *
 * This test exercises the atomMutexGet() and atomMutexPut() APIs including
 * forcing the various error indications which can be returned from the
 * APIs to ensure that handling for these corner cases have been correctly
 * implemented.
 *
 * @retval Number of failures
 */
uint32_t test_start (void)
{
    int failures;
    uint8_t status;
    ATOM_TIMER timer_cb;
    int count;

    /* Default to zero failures */
    failures = 0;

    /* Test parameter checks */
    if (atomMutexGet (NULL, 0) != ATOM_ERR_PARAM)
    {
        ATOMLOG (_STR("Get param failed\n"));
        failures++;
    }
    if (atomMutexPut (NULL) != ATOM_ERR_PARAM)
    {
        ATOMLOG (_STR("Put param failed\n"));
        failures++;
    }

    /* Test atomMutexGet() can not be called from interrupt context */
    g_result = 0;
    if (atomMutexCreate (&mutex1) != ATOM_OK)
    {
        ATOMLOG (_STR("Error creating test mutex1\n"));
        failures++;
    }
    else
    {
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

        /* Wait two seconds for g_result to be set indicating success */
        else
        {
            atomTimerDelay (2 * SYSTEM_TICKS_PER_SEC);
            if (g_result != 1)
            {
                ATOMLOG (_STR("Context check failed\n"));
                failures++;
            }
        }

        /* Delete the test mutex */
        if (atomMutexDelete (&mutex1) != ATOM_OK)
        {
            ATOMLOG (_STR("Mutex1 delete failed\n"));
            failures++;
        }
    }

    /* Create mutex1 which will be owned by us */
    if (atomMutexCreate (&mutex1) != ATOM_OK)
    {
        ATOMLOG (_STR("Error creating test mutex 1\n"));
        failures++;
    }

    /* Create mutex2 which will be owned by another thread */
    else if (atomMutexCreate (&mutex2) != ATOM_OK)
    {
        ATOMLOG (_STR("Error creating test mutex 2\n"));
        failures++;
    }

    /* Create a test thread, the sole purpose of which is to own mutex2 */
    g_owned = 0;
    if (atomThreadCreate(&tcb[0], TEST_THREAD_PRIO, test_thread_func, 0,
              &test_thread_stack[0][0],
              TEST_THREAD_STACK_SIZE, TRUE) != ATOM_OK)
    {
        /* Fail */
        ATOMLOG (_STR("Error creating test thread 1\n"));
        failures++;
    }

    /* Sleep until the test thread owns mutex2 */
    atomTimerDelay (SYSTEM_TICKS_PER_SEC);
    if (g_owned == 0)
    {
        ATOMLOG (_STR("Thread own fail\n"));
        failures++;
    }

    /* Test wait on mutex with timeout - should timeout while owned by another thread */
    if ((status = atomMutexGet (&mutex2, SYSTEM_TICKS_PER_SEC)) != ATOM_TIMEOUT)
    {
        ATOMLOG (_STR("Get %d\n"), status);
        failures++;
    }
    else
    {
        /* Success */
    }

    /* Test wait on mutex with no blocking - should return that owned by another thread */
    if ((status = atomMutexGet (&mutex2, -1)) != ATOM_WOULDBLOCK)
    {
        ATOMLOG (_STR("Wouldblock err %d\n"), status);
        failures++;
    }

    /* Test wait on mutex with no blocking when mutex is available */
    if (atomMutexGet (&mutex1, -1) != ATOM_OK)
    {
        ATOMLOG (_STR("Error taking mutex1\n"));
        failures++;
    }
    else
    {
        /* Relinquish ownership of mutex1 */
        if (atomMutexPut (&mutex1) != ATOM_OK)
        {
            ATOMLOG (_STR("Error posting mutex\n"));
            failures++;
        }
    }

    /* Test for lock count overflows with too many gets */
    count = 255;
    while (count--)
    {
        if (atomMutexGet (&mutex1, 0) != ATOM_OK)
        {
            ATOMLOG (_STR("Error getting mutex1\n"));
            failures++;
            break;
        }
    }

    /* The lock count should overflow this time */
    if (atomMutexGet (&mutex1, 0) != ATOM_ERR_OVF)
    {
        ATOMLOG (_STR("Error tracking overflow\n"));
        failures++;
    }
    else
    {
        /* Success */
    }

    /* Delete the test mutexes */
    if (atomMutexDelete (&mutex1) != ATOM_OK)
    {
        ATOMLOG (_STR("Error deleting mutex1\n"));
        failures++;
    }
    if (atomMutexDelete (&mutex2) != ATOM_OK)
    {
        ATOMLOG (_STR("Error deleting mutex2\n"));
        failures++;
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
 * \b testCallback
 *
 * Attempt an atomMutexGet() on mutex1 from interrupt context.
 * Should receive an ATOM_ERR_CONTEXT error. Sets g_result if passes.
 *
 * @param[in] cb_data Not used
 */
static void testCallback (POINTER cb_data)
{
    /* Check the return value from atomMutexGet() */
    if (atomMutexGet(&mutex1, 0) == ATOM_ERR_CONTEXT)
    {
        /* Received the error we expected, set g_result to notify success */
        g_result = 1;
    }
    else
    {
        /* Did not get expected error, don't set g_result signifying fail */
    }

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

    /*
     * Take mutex2 so that main thread can test mutex APIs on a mutex
     * which it does not own.
     */
    status = atomMutexGet(&mutex2, 0);
    if (status != ATOM_OK)
    {
        ATOMLOG (_STR("Mutex get (%d)\n"), status);
    }
    else
    {
        /* We took ownership of mutex2, set g_owned to notify success */
        g_owned = 1;
    }

    /* Wait forever */
    while (1)
    {
        atomTimerDelay (SYSTEM_TICKS_PER_SEC);
    }
}
