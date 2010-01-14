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
#include "atomsem.h"
#include "atomtests.h"
#include "atomuser.h"


/* Test OS objects */
static ATOM_SEM sem1;
static ATOM_TCB tcb1, tcb2, tcb3;
static uint8_t test1_thread_stack[TEST_THREAD_STACK_SIZE];
static uint8_t test2_thread_stack[TEST_THREAD_STACK_SIZE];
static uint8_t test3_thread_stack[TEST_THREAD_STACK_SIZE];


/* Test results */
static volatile int pass_flag[3];


/* Forward declarations */
static void test_thread_func (uint32_t data);


/**
 * \b test_start
 *
 * Start semaphore test.
 *
 * This test verifies the semaphore deletion API, by deleting a semaphore
 * on which multiple threads are blocking, and checking that all three
 * are woken up with an appropriate error code.
 *
 * @retval Number of failures
 */
uint32_t test_start (void)
{
    int failures;
    int i;

    /* Default to zero failures */
    failures = 0;

    /* Initialise pass status for all three threads to FALSE */
    for (i = 0; i < 3; i++)
    {
        pass_flag[i] = FALSE;
    }

    /* Test wakeup of three threads on semaphore deletion */
    if (atomSemCreate (&sem1, 0) != ATOM_OK)
    {
        ATOMLOG (_STR("Error creating test semaphore 1\n"));
        failures++;
    }

    else
    {
        /* Create test thread 1 */
        if (atomThreadCreate(&tcb1, TEST_THREAD_PRIO, test_thread_func, 0,
                  &test1_thread_stack[TEST_THREAD_STACK_SIZE - 1]) != ATOM_OK)
        {
            /* Fail */
            ATOMLOG (_STR("Error creating test thread 1\n"));
            failures++;
        }

        /* Create test thread 2 */
        else if (atomThreadCreate(&tcb2, TEST_THREAD_PRIO, test_thread_func, 1,
                  &test2_thread_stack[TEST_THREAD_STACK_SIZE - 1]) != ATOM_OK)
        {
            /* Fail */
            ATOMLOG (_STR("Error creating test thread 2\n"));
            failures++;
        }

        /* Create test thread 3 */
        else if (atomThreadCreate(&tcb3, TEST_THREAD_PRIO, test_thread_func, 2,
                  &test3_thread_stack[TEST_THREAD_STACK_SIZE - 1]) != ATOM_OK)
        {
            /* Fail */
            ATOMLOG (_STR("Error creating test thread 3\n"));
            failures++;
        }

        /* Test threads now created */
        else
        {
            /* Wait a while for threads to start blocking on sem1 */
            atomTimerDelay (SYSTEM_TICKS_PER_SEC/4);

            /* Delete sem1 now that all three threads should be blocking */
            if (atomSemDelete (&sem1) != ATOM_OK)
            {
                ATOMLOG (_STR("Delete fail\n"));
                failures++;
            }
            else
            {
                /* Wait a while for all three threads to wake up */
                atomTimerDelay (SYSTEM_TICKS_PER_SEC/4);

                /* Check that all three threads have passed */
                if ((pass_flag[0] != TRUE) || (pass_flag[1] != TRUE) || (pass_flag[2] != TRUE))
                {
                    ATOMLOG (_STR("Thread fail\n"));
                    failures++;
                }
            }
        }
    }

    /* Log final status */
    if (failures == 0)
    {
        ATOMLOG (_STR("Pass\n"));
    }
    else
    {
        ATOMLOG (_STR("Fail(%d)\n"), failures);
    }

    /* Quit */
    return failures;
}


/**
 * \b test_thread_func
 *
 * Entry point for test threads.
 *
 * @param[in] data Thread ID (0-2)
 *
 * @return None
 */
static void test_thread_func (uint32_t data)
{
    uint8_t status;
    int thread_id;

    /* Pull out the passed thread ID */
    thread_id = (int)data;

    /*
     * Wait on sem1 with timeout. We are expecting to be woken up
     * by the main thread while blocking.
     */
    status = atomSemGet(&sem1, (5 * SYSTEM_TICKS_PER_SEC));
    if (status != ATOM_ERR_DELETED)
    {
        ATOMLOG (_STR("Test1 thread woke without deletion (%d)\n"), status);
    }
    else
    {
        /* We were woken due to deletion as expected, set pass_flag to notify success */
        pass_flag[thread_id] = TRUE;
    }

    /* Wait forever */
    while (1)
    {
        atomTimerDelay (SYSTEM_TICKS_PER_SEC);
    }
}