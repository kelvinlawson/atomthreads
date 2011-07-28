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


/* Global test data */
static volatile int callback_ran_flag;


/* Forward declarations */
static void testCallback (POINTER cb_data);


/**
 * \b test_start
 *
 * Start timer test.
 *
 * This test exercises the atomTimerCancel() API. It tests that bad
 * parameters are trapped, and that it can be used to cancel an
 * in-progress timer callback request.
 *
 * @retval Number of failures
 */
uint32_t test_start (void)
{
    int failures;
    ATOM_TIMER timer_cb;

    /* Default to zero failures */
    failures = 0;

    /* Test parameter checks */
    if (atomTimerCancel(NULL) != ATOM_ERR_PARAM)
    {
        ATOMLOG (_STR("Param\n"));
        failures++;
    }

    /* Test cancel when timer not registered */
    if (atomTimerCancel(&timer_cb) != ATOM_ERR_NOT_FOUND)
    {
        ATOMLOG (_STR("NotFound\n"));
        failures++;
    }

    /* Test a callback can be cancelled */
    callback_ran_flag = FALSE;

    /* Request a callback in one second, no callback param required */
    timer_cb.cb_ticks = SYSTEM_TICKS_PER_SEC;
    timer_cb.cb_func = testCallback;

    /* Register the timer callback */
    if (atomTimerRegister (&timer_cb) != ATOM_OK)
    {
        ATOMLOG (_STR("TimerReg\n"));
        failures++;
    }
    else
    {
        /* Successfully registered for one second's time */

        /* Cancel the callback */
        if (atomTimerCancel (&timer_cb) != ATOM_OK)
        {
            ATOMLOG (_STR("TimerCancel\n"));
            failures++;
        }
        else
        {
            /* Successfully cancelled the callback */

            /* Wait two seconds, and check callback did not occur */
            if (atomTimerDelay(2 * SYSTEM_TICKS_PER_SEC) != ATOM_OK)
            {
                ATOMLOG (_STR("Wait\n"));
                failures++;
            }
            else
            {
                /* The ran flag should still be FALSE */
                if (callback_ran_flag != FALSE)
                {
                    ATOMLOG (_STR("Called back\n"));
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
 * Set a flag to say we ran. Expected not to run for a pass.
 *
 * @param[in] cb_data Not used
 */
static void testCallback (POINTER cb_data)
{
    /* Callback was called */
    callback_ran_flag = TRUE;
}
