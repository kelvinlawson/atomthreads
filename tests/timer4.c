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
#include "atomtimer.h"
#include "atomtests.h"


/* Test OS objects */
static ATOM_TIMER timer_cb[4];


/* Global test data */
static uint32_t cb_ticks[4];
static uint32_t start_time;


/* Forward declarations */
static void testCallback (POINTER cb_data);


/**
 * \b test_start
 *
 * Start timer test.
 *
 * This test exercises the atomTimerRegister() API, particularly the
 * linked lists used internally for managing an ordered list of timers.
 *
 * Several timers are registered out of order (in terms of the callback
 * time) and we check that the callbacks are called at the expected
 * times.
 *
 * @retval Number of failures
 */
uint32_t test_start (void)
{
    int failures;
    int i;

    /* Default to zero failures */
    failures = 0;

    /*
     * Sleep for one tick first to ensure we start near a
     * tick boundary. This should ensure that the timer
     * tick does not advance while we are setting up the
     * timers but before registering them.
     */
    atomTimerDelay(1);

    /*
     * Keep a note of the time we started the timers. This
     * can be added to the tick timers to calculate the final
     * time at which we expect the timers to expire.
     */
    start_time = atomTimeGet();

    /*
     * Fill out four timer request structures. Callbacks are
     * requested starting in one second, with the others
     * at 1 tick intervals thereafter.
     */
    for (i = 0; i < 4; i++)
    {
        /*
         * testCallback() is passed the expected
         * callback time via cb_data.
         */
        cb_ticks[i] = SYSTEM_TICKS_PER_SEC + i;
        timer_cb[i].cb_ticks = cb_ticks[i];
        timer_cb[i].cb_func = testCallback;
        timer_cb[i].cb_data = &cb_ticks[i];
    }

    /* Register the timers in a different order */
    if (atomTimerRegister (&timer_cb[1]) != ATOM_OK)
    {
        ATOMLOG (_STR("TimerReg1\n"));
        failures++;
    }
    else if (atomTimerRegister (&timer_cb[3]) != ATOM_OK)
    {
        ATOMLOG (_STR("TimerReg3\n"));
        failures++;
    }
    else if (atomTimerRegister (&timer_cb[2]) != ATOM_OK)
    {
        ATOMLOG (_STR("TimerReg2\n"));
        failures++;
    }
    else if (atomTimerRegister (&timer_cb[0]) != ATOM_OK)
    {
        ATOMLOG (_STR("TimerReg0\n"));
        failures++;
    }
    else
    {
        /* Successfully registered timers */

        /* Wait two seconds for callbacks to complete */
        if (atomTimerDelay(2 * SYSTEM_TICKS_PER_SEC) != ATOM_OK)
        {
            ATOMLOG (_STR("Wait\n"));
            failures++;
        }
        else
        {
            /*
             * The callbacks should have cleared down cb_ticks[x]
             * to zero if they were called at the expected
             * system time.
             */
            for (i = 0; i < 4; i++)
            {
                /* Check the callback has zeroed the area */
                if (cb_ticks[i] != 0)
                {
                    ATOMLOG (_STR("Clear%d\n"), i);
                    failures++;
                }
            }
        }
    }

    /* Quit */
    return failures;

}


/**
 * \b testCallback
 *
 * Check the time at which this timer callback occurs matches
 * the time we expected. Clear down the expected time location
 * if correct.
 *
 * @param[in] cb_data Not used
 */
static void testCallback (POINTER cb_data)
{
    uint32_t expected_end_time;

    /*
     * Pull out the number of ticks the timer was registered for
     * and add this to the start time, to get the final time at
     * which we expect the timers to expire.
     */
    expected_end_time = start_time + *(uint32_t *)cb_data;

    /*
     * Check the callback time (now) matches the time
     * we expected the callback.
     */
    if (atomTimeGet() == expected_end_time)
    {
        /* Called back when we expected, clear the passed location */
        *(uint32_t *)cb_data = 0;
    }
    else
    {
        /* Not called at expected time, don't clear the location */
    }

}