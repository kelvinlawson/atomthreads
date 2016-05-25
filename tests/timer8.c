/*
 * Copyright (c) 2016, Kelvin Lawson. All rights reserved.
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
#include "atomtimer.h"
#include "atomtests.h"


/* Tick delay before each callback */
#define CALLBACK_TICKS			2

/* Number of re-registrations to make from callbacks */
#define NUM_CALLBACKS 			(SYSTEM_TICKS_PER_SEC / CALLBACK_TICKS)


/* Test OS objects */
static ATOM_TIMER timer_cb;


/* Global test data */
static int callback_ran_count;


/* Forward declarations */
static void testCallback (POINTER cb_data);


/**
 * \b test_start
 *
 * Start timer test.
 *
 * This test checks that a timer can be registered from within a timer
 * callback context. i.e. checks the kernel timer queue can handle additions
 * to the timer queue while already walking the queue during a timer callback.
 *
 * @retval Number of failures
 */
uint32_t test_start (void)
{
    int failures;

    /* Default to zero failures */
    failures = 0;

    /* Clear down the callback count */
    callback_ran_count = 0;

    /* Fill out timer request structures for 2 ticks from now */
    timer_cb.cb_ticks = CALLBACK_TICKS;
    timer_cb.cb_func = testCallback;
    timer_cb.cb_data = &callback_ran_count;

    /* Register timer */
    if (atomTimerRegister (&timer_cb) != ATOM_OK)
    {
        ATOMLOG (_STR("TimerReg\n"));
        failures++;
    }

    /* Timer was successfully created */
    else
    {
        /* Sleep for twice the period we expect the test to take */
        atomTimerDelay (2 * CALLBACK_TICKS * NUM_CALLBACKS);

        /*
        * We should now find that the timer callback has run the correct
        * number of times.
        */
        if (callback_ran_count != NUM_CALLBACKS)
        {
            ATOMLOG (_STR("Callback count %d vs %d\n"), callback_ran_count, NUM_CALLBACKS);
            failures++;
        }
    }

    /* Quit */
    return failures;

}


/**
 * \b testCallback
 *
 * Increment a counut to say we ran. Also register the same timer to occur
 * again, up to a fixed number of times. The main test procedure can wait
 * and check the count to see that all of the requested callbacks occurred
 * (and hence timer registration from within a timer callback worked).
 *
 * @param[in] cb_data Pointer to counter of callback runs
 */
static void testCallback (POINTER cb_data)
{
    int *ran_count_ptr = (int *)cb_data;

    /* Callback was called */
    *ran_count_ptr += 1;

    /* If the test is still running, register the same timer again */
    if (*ran_count_ptr < NUM_CALLBACKS)
    {
        /*  Same timer details as previously */
        timer_cb.cb_ticks = CALLBACK_TICKS;
        timer_cb.cb_func = testCallback;
        timer_cb.cb_data = &callback_ran_count;
        if (atomTimerRegister (&timer_cb) != ATOM_OK)
        {
            ATOMLOG (_STR("TimerCbReg\n"));
        }
    }
}
