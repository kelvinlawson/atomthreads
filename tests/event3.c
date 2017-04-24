/*
 * Copyright (c) 2017 jinsong yu, ramaxel. All rights reserved.
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
#include "atomevent.h"


/* Number of test threads */
#define NUM_TEST_THREADS      3

#define RX_COMPLETE_FLAG 0x01
#define TX_COMPLETE_FLAG 0x02

/* Test OS objects */
static ATOM_EVENT event1;
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
 * Start event3 test.
 *
 * This stress-tests atomEventGet()/atomEventSet() with a timer ticker callback
 * continually calling atom_event_set() and several contexts continually
 * calling atomEventGet(). This stresses in particular the atomEventGet()
 * API, with one threads and main thread at different priorities get
 * simultaneously, as well as a timer callback set it from
 * interrupt context. In all cases the event is successful to get and set.
 *
 * This tests the thread-safety and interrupt-safety of the event
 * APIs.
 *
 * @retval Number of failures
 */
uint32_t test_start (void)
{
    int failures;
    uint32_t end_time,count;
    ATOM_TIMER timer_cb;
    uint32_t requested_flags;
    uint32_t actual_flags_ptr;
    requested_flags = (RX_COMPLETE_FLAG | RX_COMPLETE_FLAG);

    /* Default to zero failures */
    failures = 0;
    count = 0;

    /* Create sem with count of zero */
    if (atomEventCreate(&event1, "event1") != ATOM_OK)

    {
        ATOMLOG (_STR("Error creating test event 1\n"));
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

        /* Create thread 1   */
        else if (atomThreadCreate(&tcb[0], TEST_THREAD_PRIO , test_thread_func, 1,
              &test_thread_stack[0][0],
              TEST_THREAD_STACK_SIZE, TRUE) != ATOM_OK)
        {
            /* Fail */
            ATOMLOG (_STR("Error creating test thread 1\n"));
            failures++;
        }

        /* The test threads have now all been created */
        else
        {
            /*
             * Continually decrement the event while the test threads
             * and timer callbacks are continually incrementing it. The
             * test finishes after this runs without error for 5 seconds.
             */
            end_time = atomTimeGet() + (60*60*15 * SYSTEM_TICKS_PER_SEC);
            while (atomTimeGet() < end_time)
            {
                /* Decrement the event */
            	if(atomEventGet(&event1, requested_flags,ATOM_AND_CLEAR,&actual_flags_ptr,3*SYSTEM_TICKS_PER_SEC)!=ATOM_OK)//ATOM_WAIT_FOREVER
                {
            		ATOMLOG (_STR("atomEventGet failure, actual_flags_ptr %d \n"),(int)actual_flags_ptr);
                    failures++;
                    break;
                }
            	count++;
            	if(count%100 == 0)
            	{
            		ATOMLOG (_STR("atomEventGet atomTimeGet success, count %d \n"),(int)count);
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
        if (atomEventDelete (&event1) != ATOM_OK)
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
    int  failures,count;
    uint32_t requested_flags;
    uint32_t actual_flags_ptr;
    requested_flags = (RX_COMPLETE_FLAG | RX_COMPLETE_FLAG);

    /* Were we requested to sleep occasionally? */
    param = (int)param;

    /* Run until the main thread sets the finish flag or we get an error */
    failures = 0;

    while ((test_running == TRUE) && (failures == 0))
    {
        /* get the event each 2 system time ticks */

    	if(atomEventGet(&event1, requested_flags,ATOM_AND_CLEAR,&actual_flags_ptr,2*SYSTEM_TICKS_PER_SEC)!=ATOM_OK)//ATOM_WAIT_FOREVER
        {
            ATOMLOG (_STR("atomEventGet failure, actual_flags_ptr %d \n"),(int)actual_flags_ptr);
            failures++;
            break;
        }
    	count++;
    	if(count%100 == 0)
    	{
    		ATOMLOG (_STR("atomEventGet test_thread_func success, count %d \n"),(int)count);
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
 * set the event from interrupt context. This will be occurring while
 * atomEventGet() calls for 2 threads are in progress,
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

    /* set event1 */
    if ((status = atomEventSet(&event1, RX_COMPLETE_FLAG||TX_COMPLETE_FLAG,ATOM_OR)) != ATOM_OK)
    {
    	ATOMLOG (_STR("atomEventGet failure, actual_flags_ptr %d \n"),(int)status);
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
