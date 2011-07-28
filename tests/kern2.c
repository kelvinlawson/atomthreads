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


/**
 * \b test_start
 *
 * Start kernel test.
 *
 * This is a basic test of the thread context-switch functionality. It
 * creates twenty local (byte) variables and schedules the thread out
 * for one second. If context-switch save/restore is not implemented
 * correctly, you might expect one or more of these local variables to
 * be corrupted by the time the thread is scheduled back in.
 *
 * Note that this is a fairly unsophisticated test, and a lot depends on
 * how the compiler deals with the variables, as well as what code is
 * executed while the thread is scheduled out. It should flag up any
 * major problems with the context-switch, however.
 *
 * @retval Number of failures
 */
uint32_t test_start (void)
{
    int failures;
    uint8_t one = 1;
    uint8_t two = 2;
    uint8_t three = 3;
    uint8_t four = 4;
    uint8_t five = 5;
    uint8_t six = 6;
    uint8_t seven = 7;
    uint8_t eight = 8;
    uint8_t nine = 9;
    uint8_t ten = 10;
    uint16_t eleven = 11;
    uint16_t twelve = 12;
    uint16_t thirteen = 13;
    uint16_t fourteen = 14;
    uint16_t fifteen = 15;
    uint32_t sixteen = 16;
    uint32_t seventeen = 17;
    uint32_t eighteen = 18;
    uint32_t nineteen = 19;
    uint32_t twenty = 20;

    /* Default to zero failures */
    failures = 0;

    /* Sleep for one second */
    atomTimerDelay(SYSTEM_TICKS_PER_SEC);

    /* Check all variables contain expected values */
    if (one != 1)
    {
        ATOMLOG (_STR("1(%d)\n"), (int)one);
        failures++;
    }
    if (two != 2)
    {
        ATOMLOG (_STR("2(%d)\n"), (int)two);
        failures++;
    }
    if (three != 3)
    {
        ATOMLOG (_STR("3(%d)\n"), (int)three);
        failures++;
    }
    if (four != 4)
    {
        ATOMLOG (_STR("4(%d)\n"), (int)four);
        failures++;
    }
    if (five != 5)
    {
        ATOMLOG (_STR("5(%d)\n"), (int)five);
        failures++;
    }
    if (six != 6)
    {
        ATOMLOG (_STR("6(%d)\n"), (int)six);
        failures++;
    }
    if (seven != 7)
    {
        ATOMLOG (_STR("7(%d)\n"), (int)seven);
        failures++;
    }
    if (eight != 8)
    {
        ATOMLOG (_STR("8(%d)\n"), (int)eight);
        failures++;
    }
    if (nine != 9)
    {
        ATOMLOG (_STR("9(%d)\n"), (int)nine);
        failures++;
    }
    if (ten != 10)
    {
        ATOMLOG (_STR("10(%d)\n"), (int)ten);
        failures++;
    }
    if (eleven != 11)
    {
        ATOMLOG (_STR("11(%d)\n"), (int)eleven);
        failures++;
    }
    if (twelve != 12)
    {
        ATOMLOG (_STR("12(%d)\n"), (int)twelve);
        failures++;
    }
    if (thirteen != 13)
    {
        ATOMLOG (_STR("13(%d)\n"), (int)thirteen);
        failures++;
    }
    if (fourteen != 14)
    {
        ATOMLOG (_STR("14(%d)\n"), (int)fourteen);
        failures++;
    }
    if (fifteen != 15)
    {
        ATOMLOG (_STR("15(%d)\n"), (int)fifteen);
        failures++;
    }
    if (sixteen != 16)
    {
        ATOMLOG (_STR("16(%d)\n"), (int)sixteen);
        failures++;
    }
    if (seventeen != 17)
    {
        ATOMLOG (_STR("17(%d)\n"), (int)seventeen);
        failures++;
    }
    if (eighteen != 18)
    {
        ATOMLOG (_STR("18(%d)\n"), (int)eighteen);
        failures++;
    }
    if (nineteen != 19)
    {
        ATOMLOG (_STR("19(%d)\n"), (int)nineteen);
        failures++;
    }
    if (twenty != 20)
    {
        ATOMLOG (_STR("20(%d)\n"), (int)twenty);
        failures++;
    }

    /* Quit */
    return failures;

}
