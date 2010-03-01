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
static int callback_ran_flag[4];


/* Forward declarations */
static void testCallback (POINTER cb_data);


/**
 * \b test_start
 *
 * Start timer test.
 *
 * This test exercises the atomTimerCancel() API, particularly its
 * behaviour when there are several timers registered. Four timers
 * are registered, two of which are cancelled, and the test confirms
 * that only the two which are not cancelled are called back.
 *
 * @retval Number of failures
 */
uint32_t test_start (void)
{
    int failures;
    int i;

    /* Default to zero failures */
    failures = 0;

    /* Clear down the ran flag for all four timers */
    for (i = 0; i < 4; i++)
    {
        callback_ran_flag[i] = FALSE;
    }

    /*
     * Fill out four timer request structures. Callbacks are
     * requested starting in one second, with the others
     * at 1 tick intervals thereafter.
     */
    for (i = 0; i < 4; i++)
    {
        /*
         * testCallback() is passed a pointer to the flag it
         * should set to notify that it has run.
         */
        timer_cb[i].cb_ticks = SYSTEM_TICKS_PER_SEC + i;
        timer_cb[i].cb_func = testCallback;
        timer_cb[i].cb_data = &callback_ran_flag[i];
    }

    /* Register all four timers */
    for (i = 0; i < 4; i++)
    {
        if (atomTimerRegister (&timer_cb[i]) != ATOM_OK)
        {
            ATOMLOG (_STR("TimerReg\n"));
            failures++;
        }
    }

    /* Check timers were successfully created */
    if (failures == 0)
    {
        /* Cancel two of the callbacks */
        if (atomTimerCancel (&timer_cb[1]) != ATOM_OK)
        {
            ATOMLOG (_STR("Cancel1\n"));
            failures++;
        }
        if (atomTimerCancel (&timer_cb[2]) != ATOM_OK)
        {
            ATOMLOG (_STR("Cancel2\n"));
            failures++;
        }

        /* Wait two seconds for callbacks to complete */
        if (atomTimerDelay(2 * SYSTEM_TICKS_PER_SEC) != ATOM_OK)
        {
            ATOMLOG (_STR("Wait\n"));
            failures++;
        }
        else
        {
            /*
             * We should now find that timer callbacks 0 and 3
             * have run, but 1 and 2 did not (due to cancellation).
             */
            if ((callback_ran_flag[0] != TRUE) || (callback_ran_flag[3] != TRUE)
                || (callback_ran_flag[1] != FALSE) || (callback_ran_flag[2] != FALSE))
            {
                ATOMLOG (_STR("Cancellations\n"));
                failures++;
            }
        }
    }

    /* Quit */
    return failures;

}


/**
 * \b testCallback
 *
 * Set a flag to say we ran. Some of the callbacks are
 * expected to execute this, while those that are
 * cancelled should not.
 *
 * @param[in] cb_data Not used
 */
static void testCallback (POINTER cb_data)
{
    /* Callback was called */
    *(int *)cb_data = TRUE;
}