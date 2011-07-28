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
#include "atomtests.h"


/* Test OS objects */
static ATOM_TIMER timer_cb[4];


/* Global test data */
static volatile uint32_t cb_order[4];
static int cb_cnt = 0;


/* Forward declarations */
static void testCallback (POINTER cb_data);


/**
 * \b test_start
 *
 * Start timer test.
 *
 * Test the behaviour of the timer subsystem on a system clock rollover.
 *
 * Sets the system clock to just before rollover and registers several
 * timers. Tests that all timer callbacks occur and that they occur in
 * in the correct order, when they span a timer rollover.
 *
 * @retval Number of failures
 */
uint32_t test_start (void)
{
    CRITICAL_STORE;
    int i, failures;

    /* Default to zero failures */
    failures = 0;

    /*
     * Lockout interrupts while registering to ensure that the clock does
     * not roll over under us while we are registering our test timers.
     */
    CRITICAL_START ();

    /* Set the clock to rollover - 6 */
    atomTimeSet (0xFFFFFFFA);

    /* Timer in 2 ticks (pre-rollover): should be called back first */
    timer_cb[0].cb_ticks = 2;
    timer_cb[0].cb_func = testCallback;
    timer_cb[0].cb_data = (POINTER)0;
    if (atomTimerRegister (&timer_cb[0]) != ATOM_OK)
    {
        ATOMLOG (_STR("TimerReg0\n"));
        failures++;
    }

    /* Timer in 10 ticks (post-rollover): should be called back last */
    timer_cb[1].cb_ticks = 10;
    timer_cb[1].cb_func = testCallback;
    timer_cb[1].cb_data = (POINTER)3;
    if (atomTimerRegister (&timer_cb[1]) != ATOM_OK)
    {
        ATOMLOG (_STR("TimerReg1\n"));
        failures++;
    }

    /* Timer in 4 ticks (pre-rollover): should be called back second */
    timer_cb[2].cb_ticks = 4;
    timer_cb[2].cb_func = testCallback;
    timer_cb[2].cb_data = (POINTER)1;
    if (atomTimerRegister (&timer_cb[2]) != ATOM_OK)
    {
        ATOMLOG (_STR("TimerReg2\n"));
        failures++;
    }

    /* Timer in 9 ticks (post-rollover): should be called back third */
    timer_cb[3].cb_ticks = 9;
    timer_cb[3].cb_func = testCallback;
    timer_cb[3].cb_data = (POINTER)2;
    if (atomTimerRegister (&timer_cb[3]) != ATOM_OK)
    {
        ATOMLOG (_STR("TimerReg3\n"));
        failures++;
    }

    /* Initialise the cb_order delay to known values */
    for (i = 0; i < 4; i++)
    {
        cb_order[i] = 99;
    }

    /* Unlock interrupts and let the test begin */
    CRITICAL_END ();

    /*
     * Wait 20 ticks for the callbacks to complete. Also tests another
     * timer registration via atomTimerDelay() for us.
     */
    atomTimerDelay (20);

    /* Check the order the callbacks came in matched our expectations */
    for (i = 0; i < 4; i++)
    {
        if (cb_order[i] != i)
        {
            ATOMLOG (_STR("T%d=%d\n"), i, (int)cb_order[i]);
            failures++;
        }
    }

    /* Quit */
    return failures;

}


/**
 * \b testCallback
 *
 * Timer callback. Store our cb_data value in cb_order[].
 * This allows us to check that the callback order was as expectd.
 *
 * @param[in] cb_data Not used
 */
static void testCallback (POINTER cb_data)
{
    int expected_order;

    /* Pull out the expected ordere */
    expected_order = (int)cb_data;

    /* Store our callback order in cb_order[] */
    cb_order[cb_cnt] = expected_order;

    /* Interrupts are locked out so we can modify cb_cnt without protection */
    cb_cnt++;
}
