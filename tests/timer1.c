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
#include "atomtimer.h"
#include "atomtests.h"


/* Test OS objects */
static ATOM_SEM sem1;


/* Forward declarations */
static void testCallback (POINTER cb_data);


/**
 * \b test_start
 *
 * Start timer test.
 *
 * This test exercises the atomTimerDelay() API. It checks that
 * the correct time delay is used, and also checks that error
 * checking is correctly implemented within the API.
 *
 * @retval Number of failures
 */
uint32_t test_start (void)
{
    int failures;
    uint32_t start_time, end_time;
    ATOM_TIMER timer_cb;

    /* Default to zero failures */
    failures = 0;

    /* Test parameter-checks */
    if (atomTimerDelay(0) != ATOM_ERR_PARAM)
    {
        ATOMLOG(_STR("Param\n"));
        failures++;
    }

    /* Create a semaphore for receiving test notification */
    if (atomSemCreate (&sem1, 0) != ATOM_OK)
    {
        ATOMLOG (_STR("SemCreate\n"));
        failures++;
    }
    else
    {
        /*
         * Create a timer callback which will attempt to
         * call atomTimerDelay() at interrupt context.
         */
        timer_cb.cb_func = testCallback;
        timer_cb.cb_data = NULL;
        timer_cb.cb_ticks = SYSTEM_TICKS_PER_SEC;

        /* Request the timer callback to run in one second */
        if (atomTimerRegister (&timer_cb) != ATOM_OK)
        {
            ATOMLOG (_STR("TimerRegister\n"));
            failures++;
        }

        /* Wait up to two seconds for sem1 to be posted indicating success */
        else if (atomSemGet (&sem1, 2 * SYSTEM_TICKS_PER_SEC) != ATOM_OK)
        {
            ATOMLOG (_STR("Context check\n"));
            failures++;
        }

        /* Delete the test semaphore */
        if (atomSemDelete (&sem1) != ATOM_OK)
        {
            ATOMLOG (_STR("Delete\n"));
            failures++;
        }
    }

    /*
     * Test that a 1 tick delay returns when system time
     * has increased by exactly 1 tick.
     */

     /*
      * We first delay by 1 tick to ensure that the thread
      * is running at the start of a new tick, which should
      * ensure that the time does not tick over between
      * setting start_time and actually calling
      * atomTimerDelay().
      */
    atomTimerDelay(1);

    /* Record the start time */
    start_time = atomTimeGet();

    /* Request a 1 tick sleep */
    if (atomTimerDelay(1) != ATOM_OK)
    {
        ATOMLOG (_STR("Delay1\n"));
        failures++;
    }
    else
    {
        /* Record the time we woke up */
        end_time = atomTimeGet();

        /* Check that time has advanced by exactly 1 tick */
        if ((end_time - start_time) != 1)
        {
            ATOMLOG (_STR("Tick1:%d\n"), (int)(end_time-start_time));
            failures++;
        }
    }

    /*
     * Test that a 2 tick delay returns when system time
     * has increased by exactly 2 ticks.
     */

     /*
      * We first delay by 1 tick to ensure that the thread
      * is running at the start of a new tick, which should
      * ensure that the time does not tick over between
      * setting start_time and actually calling
      * atomTimerDelay().
      */
    atomTimerDelay(1);

    /* Record the start time */
    start_time = atomTimeGet();

    /* Request a 2 tick sleep */
    if (atomTimerDelay(2) != ATOM_OK)
    {
        ATOMLOG (_STR("Delay2\n"));
        failures++;
    }
    else
    {
        /* Record the time we woke up */
        end_time = atomTimeGet();

        /* Check that time has advanced by exactly 2 ticks */
        if ((end_time - start_time) != 2)
        {
            ATOMLOG (_STR("Tick2:%d\n"), (int)(end_time-start_time));
            failures++;
        }
    }

    /*
     * Test that a 500 tick delay returns when system time
     * has increased by exactly 500 ticks.
     */

     /*
      * We first delay by 1 tick to ensure that the thread
      * is running at the start of a new tick, which should
      * ensure that the time does not tick over between
      * setting start_time and actually calling
      * atomTimerDelay().
      */
    atomTimerDelay(1);

    /* Record the start time */
    start_time = atomTimeGet();

    /* Request a 500 tick sleep */
    if (atomTimerDelay(500) != ATOM_OK)
    {
        ATOMLOG (_STR("Delay500\n"));
        failures++;
    }
    else
    {
        /* Record the time we woke up */
        end_time = atomTimeGet();

        /* Check that time has advanced by exactly 500 ticks */
        if ((end_time - start_time) != 500)
        {
            ATOMLOG (_STR("Tick500:%d\n"), (int)(end_time-start_time));
            failures++;
        }
    }

    /* Quit */
    return failures;

}


/**
 * \b testCallback
 *
 * Attempt an atomTimerDelay() call from interrupt context.
 * Should receive an ATOM_ERR_CONTEXT error. Posts sem1 if successful.
 *
 * @param[in] cb_data Not used
 */
static void testCallback (POINTER cb_data)
{
    /* Check the return value from atomTimerDelay() */
    if (atomTimerDelay(1) == ATOM_ERR_CONTEXT)
    {
        /* Received the error we expected, post sem1 to notify success */
        atomSemPut(&sem1);
    }
    else
    {
        /*
         * Did not get the expected error, don't post sem1 and the test
         * thread will time out, signifying an error.
         */
    }

}
