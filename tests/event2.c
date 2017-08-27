/*
 * Copyright (c) 2017, jinsong yu, ramaxel. All rights reserved.
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
#include "atomevent.h"
#include "atomtests.h"




#define RX_COMPLETE_FLAG 0x01
#define TX_COMPLETE_FLAG 0x02


/* Number of test threads */
#define NUM_TEST_THREADS      2


/* Test OS objects */
static ATOM_EVENT event1;
static ATOM_TCB tcb[NUM_TEST_THREADS];
static uint8_t test_thread_stack[NUM_TEST_THREADS][TEST_THREAD_STACK_SIZE];


/* Forward declarations */
static void test1_thread_func (uint32_t param);



/**
 * \b test_start
 *
 * Start event test.
 *
 * This test exercises the event creation and deletion APIs, including
 * waking threads blocking on a event if the event is deleted.
 * Deletion wakeups are tested twice: once for a thread which is blocking
 * with a timeout and once for a thread which is blocking with no timeout.
 *
 * @retval Number of failures
 */
uint32_t test_start (void)
{
    int failures;
    int i;
    failures = 0;


    /* Test creation and deletion of event: good values */
    for (i = 0; i < 1000; i++)
    {
        if (atomEventCreate (&event1, "event1") == ATOM_OK)
        {
            if (atomEventDelete (&event1) == ATOM_OK)
            {
                /* Success */
            }
            else
            {
                /* Fail */
                ATOMLOG (_STR("Error deleting event1\n"));
                failures++;
                break;
            }
        }
        else
        {
            /* Fail */
            ATOMLOG (_STR("Error creating event1\n"));
            failures++;
            break;
        }
    }
    ATOMLOG (_STR("stress test for 1000 times atomEventCreate/atomEventDelete passed! \n"));

    /* Test creation and deletion of event: creation checks */
    if (atomEventCreate (NULL, 0) != ATOM_OK)
    {
        /* Success */
    }
    else
    {
        /* Fail */
        ATOMLOG (_STR("Bad event creation checks\n"));
        failures++;
    }

    /* Test creation and deletion of event: deletion checks */
    if (atomEventDelete (NULL) != ATOM_OK)
    {
        /* Success */
    }
    else
    {
        /* Fail */
        ATOMLOG (_STR("Bad event deletion checks\n"));
        failures++;
    }

    ATOMLOG (_STR("atomEventCreate/atomEventDelete parameter passed. \n"));


    /* Test wakeup of threads on event deletion (thread blocking with no timeout) */
    if (atomEventCreate (&event1, "event1") != ATOM_OK)
    {
        ATOMLOG (_STR("Error creating test event 1\n"));
        failures++;
    }


    else if (atomThreadCreate(&tcb[0], TEST_THREAD_PRIO, test1_thread_func, 0,
              &test_thread_stack[0][0],
              TEST_THREAD_STACK_SIZE, TRUE) != ATOM_OK)
    {
        /* Fail */
        ATOMLOG (_STR("Error creating test thread 1\n"));
        failures++;
    }
    else
    {

        /*
         * We have created two event. sem1 is for the other thread
         * to wait on, which we will delete from this thread. We want
         * to see that the other thread is woken up if its event
         * is deleted. This is indicated through event being posted
         * back to us.
         */

        /* Wait for the other thread to start blocking on sem1 */
        if (atomTimerDelay(SYSTEM_TICKS_PER_SEC) != ATOM_OK)
        {
            ATOMLOG (_STR("Failed timer delay\n"));
            failures++;
        }
        else
        {
        	ATOMLOG (_STR("atomEventDelete event\n"));
            /* The other thread will be blocking on sem1 now, delete sem1 */
            if (atomEventDelete(&event1) != ATOM_OK)
            {
                ATOMLOG (_STR("Failed event1 delete\n"));
                failures++;
            }
            else
            {
                /* delay and let thread1 to get the delete flag. */
                if (atomTimerDelay(SYSTEM_TICKS_PER_SEC) != ATOM_OK)
                {
                    ATOMLOG (_STR("Failed timer delay\n"));
                    failures++;
                }
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
 * \b test1_thread_func
 *
 * Entry point for test thread 1.
 *
 * @param[in] param Unused (optional thread entry parameter)
 *
 * @return None
 */
static void test1_thread_func (uint32_t param)
{
    uint8_t status;
    uint32_t actual_flags_ptr;

    /* Compiler warnings */
    param = param;

    /*
     * Wait on event with no timeout. We are expecting to be woken up
     * by the main thread delete while blocking.
     */
    status = atomEventGet(&event1, (RX_COMPLETE_FLAG | RX_COMPLETE_FLAG),ATOM_AND_CLEAR,&actual_flags_ptr,ATOM_WAIT_FOREVER);//wait wait option 0 is failed.
    if (status != ATOM_ERR_DELETED)
    {
        ATOMLOG (_STR("Test1 thread woke without deletion (%d)\n"), status);
    }
    else
    {
    	ATOMLOG (_STR("unblock the thread by delete the event \n"));
    }

    /* Wait forever */
    while (1)
    {
        atomTimerDelay (SYSTEM_TICKS_PER_SEC);
    }
}

