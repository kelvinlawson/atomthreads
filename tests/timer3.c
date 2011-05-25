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
 * This test exercises the atomTimerRegister() API. It tests that bad
 * parameters are trapped, and that timer callbacks occur at the
 * correct time.
 *
 * @retval Number of failures
 */
uint32_t test_start (void)
{
    int failures;
    ATOM_TIMER timer_cb;
    uint32_t expected_time;

    /* Default to zero failures */
    failures = 0;

    /* Create a semaphore for receiving test notifications */
    if (atomSemCreate (&sem1, 0) != ATOM_OK)
    {
        ATOMLOG (_STR("SemCreate\n"));
        failures++;
    }

    /* Test that bad parameters are trapped */

    /* NULL parameter check */
    if (atomTimerRegister(NULL) != ATOM_ERR_PARAM)
    {
        ATOMLOG (_STR("Param1\n"));
        failures++;
    }

    /* NULL callback function */
    timer_cb.cb_ticks = 1;
    timer_cb.cb_func = NULL;
    if (atomTimerRegister(&timer_cb) != ATOM_ERR_PARAM)
    {
        ATOMLOG (_STR("Param2\n"));
        failures++;
    }

    /* Zero ticks */
    timer_cb.cb_ticks = 0;
    timer_cb.cb_func = testCallback;
    if (atomTimerRegister(&timer_cb) != ATOM_ERR_PARAM)
    {
        ATOMLOG (_STR("Param3\n"));
        failures++;
    }

    /* Request a callback in 1 tick */

    /*
     * Sleep for one tick first to ensure we start near a
     * tick boundary. This should ensure that the timer
     * tick does not advance while we are setting up the
     * timer but before registering the timer.
     */
    atomTimerDelay(1);

    /* Request a callback in one tick time */
    timer_cb.cb_ticks = 1;

    /* We pass our testCallback() the expected end time */
    timer_cb.cb_func = testCallback;
    expected_time = atomTimeGet() + timer_cb.cb_ticks;
    timer_cb.cb_data = &expected_time;

    /* Register the timer callback */
    if (atomTimerRegister (&timer_cb) != ATOM_OK)
    {
        ATOMLOG (_STR("TimerReg1\n"));
        failures++;
    }

    /* Wait up to 5 ticks for sem1 to be posted indicating success */
    else if (atomSemGet (&sem1, 5) != ATOM_OK)
    {
        ATOMLOG (_STR("Tick1\n"));
        failures++;
    }


    /* Request a callback in 2 ticks */

    /*
     * Sleep for one tick first to ensure we start near a
     * tick boundary. This should ensure that the timer
     * tick does not advance while we are setting up the
     * timer but before registering the timer.
     */
    atomTimerDelay(1);

    /* Request a callback in 2 ticks time */
    timer_cb.cb_ticks = 2;

    /* We pass our testCallback() the expected end time */
    timer_cb.cb_func = testCallback;
    expected_time = atomTimeGet() + timer_cb.cb_ticks;
    timer_cb.cb_data = &expected_time;

    /* Register the timer callback */
    if (atomTimerRegister (&timer_cb) != ATOM_OK)
    {
        ATOMLOG (_STR("TimerReg2\n"));
        failures++;
    }

    /* Wait up to 5 ticks for sem1 to be posted indicating success */
    else if (atomSemGet (&sem1, 5) != ATOM_OK)
    {
        ATOMLOG (_STR("Tick2\n"));
        failures++;
    }


    /* Request a callback in 500 ticks */

    /*
     * Sleep for one tick first to ensure we start near a
     * tick boundary. This should ensure that the timer
     * tick does not advance while we are setting up the
     * timer but before registering the timer.
     */
    atomTimerDelay(1);

    /* Request a callback in 500 ticks time */
    timer_cb.cb_ticks = 500;

    /* We pass our testCallback() the expected end time */
    timer_cb.cb_func = testCallback;
    expected_time = atomTimeGet() + timer_cb.cb_ticks;
    timer_cb.cb_data = &expected_time;

    /* Register the timer callback */
    if (atomTimerRegister (&timer_cb) != ATOM_OK)
    {
        ATOMLOG (_STR("TimerReg500\n"));
        failures++;
    }

    /* Wait up to 600 ticks for sem1 to be posted indicating success */
    else if (atomSemGet (&sem1, 600) != ATOM_OK)
    {
        ATOMLOG (_STR("Tick500\n"));
        failures++;
    }

    /* Delete the test semaphore */
    if (atomSemDelete (&sem1) != ATOM_OK)
    {
        ATOMLOG (_STR("Delete\n"));
        failures++;
    }

    /* Quit */
    return failures;

}


/**
 * \b testCallback
 *
 * Check the time at which this timer callback occurs matches
 * the time we expected. Post sem1 if correct.
 *
 * @param[in] cb_data Not used
 */
static void testCallback (POINTER cb_data)
{
    uint32_t expected_end_time;

    /* Pull out the expected end time */
    expected_end_time = *(uint32_t *)cb_data;

    /*
     * Check the callback time (now) matches the time
     * we expected the callback.
     */
    if (atomTimeGet() == expected_end_time)
    {
        /* Called back when we expected, post sem1 to notify success */
        atomSemPut(&sem1);
    }
    else
    {
        /*
         * Not called at expected time, don't post sem1 and the test
         * thread will time out, signifying an error.
         */
    }

}
