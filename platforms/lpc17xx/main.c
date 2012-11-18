/*
 * Copyright (c) 2012, Natie van Rooyen. All rights reserved.
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

#include <stdio.h>
#include "LPC17xx.h"
#include "drivers/lpc17xx_uart.h"
#include "modules.h"
#include "atom.h"
#include "tests/atomtests.h"



#ifndef ATOMTHREADS_TEST
#define ATOMTHREADS_TEST                "kern1"
#endif

// for mbed  board
#define MBED_LED1_GPIO                      (1 << 18)
#define MBED_LED2_GPIO                      (1 << 20)
#define MBED_LED3_GPIO                      (1 << 21)
#define MBED_LED4_GPIO                      (1 << 23)

#define MBED_LED_GET(led)                   (LPC_GPIO1->FIOSET & led)
#define MBED_LED_SET(led, on)               { if (on) LPC_GPIO1->FIOSET = led ; else LPC_GPIO1->FIOCLR = led ; }
#define MBED_LED_TOGGLE(led)                MBED_LED_SET(led, !MBED_LED_GET(led))
#define MBED_LED_COUNT(count)               MBED_LED_SET(MBED_LED1_GPIO, count & 1) ; MBED_LED_SET(MBED_LED2_GPIO, count & 2) ; \
                                            MBED_LED_SET(MBED_LED3_GPIO, count & 4) ; MBED_LED_SET(MBED_LED4_GPIO, count & 8) ;


#define TEST_STACK_BYTE_SIZE                512
#define IDLE_STACK_BYTE_SIZE                128

static unsigned char	test_stack[TEST_STACK_BYTE_SIZE] ;
static unsigned char	idle_stack[IDLE_STACK_BYTE_SIZE] ;
ATOM_TCB				test_tcb ;


/**
 * \b test_thread
 *
 * Function calling the test function of the Atomthreads test suite.
 *
 */
void
test_thread (uint32_t param)
{
    uint32_t failures ;
    CRITICAL_STORE ;

    failures = test_start ()  ;

    atomTimerDelay (10) ;
    CRITICAL_START() ;
    dbg_format_msg ("%s %s\r\n", ATOMTHREADS_TEST, failures ? "FAIL" : "PASS") ;
    CRITICAL_END() ;

	while(1) {
#ifdef BOARD_MBED_LP1768
        MBED_LED_TOGGLE(MBED_LED1_GPIO) ;
#endif
        atomTimerDelay (65) ;
	}

}

/**
 * \b main
 *
 * Initialize atomthreads and start a test_thread to run the Atomthreads test suite. 
 *
 */
int 
main(void)
{
	
#ifdef BOARD_MBED_LP1768
    LPC_GPIO1->FIODIR |= MBED_LED1_GPIO | MBED_LED2_GPIO | MBED_LED3_GPIO | MBED_LED4_GPIO ;
    MBED_LED_SET(MBED_LED1_GPIO | MBED_LED2_GPIO | MBED_LED3_GPIO | MBED_LED4_GPIO, 1);
#endif

    dbg_format_msg ("\r\nLPC17xx SystemCoreClock =  %d\r\n",SystemCoreClock) ;

    //atomthreads_stress_test (36) ;

    dbg_format_msg ("Atomthreads starting %s... \r\n", ATOMTHREADS_TEST) ;

    atomOSInit(&idle_stack[0], IDLE_STACK_BYTE_SIZE, TRUE) ;
    atomThreadCreate ((ATOM_TCB *)&test_tcb, TEST_THREAD_PRIO, test_thread, 0, 
            &test_stack[0], TEST_STACK_BYTE_SIZE, TRUE);
    atomOSStart() ;

    while(1) ;

	return 0 ;
}

