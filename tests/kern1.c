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
static ATOM_TCB tcb1;
static uint8_t test_thread_stack[TEST_THREAD_STACK_SIZE];


/* Forward declarations */
static void test_thread_func (uint32_t param);


/**
 * \b test_start
 *
 * Start kernel test.
 *
 * This tests the handling of bad parameters within the public kernel APIs.
 *
 * Other than during initialisation, the only API that takes parameters
 * which require checking (and that application code might call) is the
 * thread create API.
 *
 * @retval Number of failures
 */
uint32_t test_start (void)
{
    int failures;

    /* Default to zero failures */
    failures = 0;

    /* atomThreadCreate: Pass a bad TCB pointer */
    if (atomThreadCreate (NULL, TEST_THREAD_PRIO, test_thread_func, 0,
            &test_thread_stack[0],
            TEST_THREAD_STACK_SIZE, TRUE) != ATOM_ERR_PARAM)
    {
        ATOMLOG (_STR("Bad TCB check\n"));
        failures++;
    }

    /* atomThreadCreate: Pass a bad entry point */
    if (atomThreadCreate (&tcb1, TEST_THREAD_PRIO, NULL, 0,
            &test_thread_stack[0],
            TEST_THREAD_STACK_SIZE, TRUE) != ATOM_ERR_PARAM)
    {
        ATOMLOG (_STR("Bad entry check\n"));
        failures++;
    }

    /* atomThreadCreate: Pass a bad stack pointer */
    if (atomThreadCreate (&tcb1, TEST_THREAD_PRIO, test_thread_func, 0,
            NULL, TEST_THREAD_STACK_SIZE, TRUE) != ATOM_ERR_PARAM)
    {
        ATOMLOG (_STR("Bad stack ptr check\n"));
        failures++;
    }

    /* atomThreadCreate: Pass a bad stack size */
    if (atomThreadCreate (&tcb1, TEST_THREAD_PRIO, test_thread_func, 0,
            &test_thread_stack[0], 0, TRUE) != ATOM_ERR_PARAM)
    {
        ATOMLOG (_STR("Bad stack size check\n"));
        failures++;
    }

    /* Quit */
    return failures;

}


/**
 * \b test_thread_func
 *
 * Entry point for test thread.
 *
 * @param[in] param Unused (optional thread entry parameter)
 *
 * @return None
 */
static void test_thread_func (uint32_t param)
{
    /* Compiler warnings */
    param = param;

    /* Wait forever */
    while (1)
    {
        atomTimerDelay (SYSTEM_TICKS_PER_SEC);
    }
}
