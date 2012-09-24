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
#include "modules.h"
#include <stdio.h>
#include <stdarg.h>
#include "atomport_private.h"
#include "atom.h"
#include "atomport.h"
#include "types.h"

SYSTICK_T*                const           board_systick        = (SYSTICK_T*)            BOARD_BASE_ADDRESS_SYSTICK ;
NVIC_T*                   const           board_nvic           = (NVIC_T*)               BOARD_BASE_ADDRESS_NVIC ;
SCB_T *                   const           board_scb            = (SCB_T*)                BOARD_BASE_ADDRESS_SCB ;
GPTM_TIMER_T *            const           board_gptm0          = (GPTM_TIMER_T*)         BOARD_BASE_ADDRESS_GPTIMER0 ;


/**
 * \b dbg_format_msg
 *
 * Same as printf. 
 *
 */
void 
dbg_format_msg (char *format, ...)
{
	va_list			args;
	static char		msg[256] ;
    CRITICAL_STORE ;

    va_start (args, format) ;
    CRITICAL_START() ;
	vsnprintf ((char*)msg, 256, (char*)format, args) ;
    printf (msg) ;
    CRITICAL_END() ;

}


/**
 * \b low_level_init
 *
 * Initializes the PIC and start the system timer tick intrerupt.
 *
 */
int
low_level_init (void)
{
    contextInit () ;

    //board_systick->STRELOAD = 0x010000 ;
    //board_systick->STCTRL = NVIC_STCTRL_CLK | 
    //                    NVIC_STCTRL_INTEN |
    //                    NVIC_STCTRL_ENABLE ;

    board_gptm0->CTL &= ~GPTM_TIMER_CTL_TAEN ;
    board_gptm0->CFG = 0 ;
    board_gptm0->TAMR = GPTM_TIMER_TMR_TMR_PERIODIC ;
    board_gptm0->TAILR = 0x10000 ;
    board_gptm0->IMR |= GPTM_TIMER_INT_TATOIM ;
    board_gptm0->CTL |= GPTM_TIMER_CTL_TAEN ;

    // board_nvic->ISER[0] = 0x80000 ;

    return 0 ;
}


/**
 * \b __context_preempt_handler
 *
 * System timer tic interupt handler.
 *
 */
void
__context_tick_handler (void) 
{

    if (1) {
        atomIntEnter();

        /* Call the OS system tick handler */
        atomTimerTick();

        board_gptm0->ICR |= GPTM_TIMER_INT_TATOIM ;

        /* Call the interrupt exit routine */
        atomIntExit(TRUE);

    }

}

void
dbg_hard_fault_handler_c (unsigned int * hardfault_args)
{
  unsigned int stacked_r0;
  unsigned int stacked_r1;
  unsigned int stacked_r2;
  unsigned int stacked_r3;
  unsigned int stacked_r12;
  unsigned int stacked_lr;
  unsigned int stacked_pc;
  unsigned int stacked_psr;
 
  stacked_r0 = ((unsigned long) hardfault_args[0]);
  stacked_r1 = ((unsigned long) hardfault_args[1]);
  stacked_r2 = ((unsigned long) hardfault_args[2]);
  stacked_r3 = ((unsigned long) hardfault_args[3]);
 
  stacked_r12 = ((unsigned long) hardfault_args[4]);
  stacked_lr = ((unsigned long) hardfault_args[5]);
  stacked_pc = ((unsigned long) hardfault_args[6]);
  stacked_psr = ((unsigned long) hardfault_args[7]);
 
  printf ("\r\n\r\n[Hard fault handler - all numbers in hex]\r\n");
  printf ("SP = 0x%x\r\n", hardfault_args);
  printf ("R0 = 0x%x\r\n", stacked_r0);
  printf ("R1 = 0x%x\r\n", stacked_r1);
  printf ("R2 = 0x%x\r\n", stacked_r2);
  printf ("R3 = 0x%x\r\n", stacked_r3);
  printf ("R12 = 0x%x\r\n", stacked_r12);
  printf ("LR [R14] = 0x%x  subroutine call return address\r\n", stacked_lr);
  printf ("PC [R15] = 0x%x  program counter\r\n", stacked_pc);
  printf ("PSR = 0x%x\r\n", stacked_psr);
  //printf ("BFAR = 0x%x\r\n", (*((volatile unsigned long *)(0xE000ED38))));
  //printf ("CFSR = 0x%x\r\n", (*((volatile unsigned long *)(0xE000ED28))));
  //printf ("HFSR = 0x%x\r\n", (*((volatile unsigned long *)(0xE000ED2C))));
  //printf ("DFSR = 0x%x\r\n", (*((volatile unsigned long *)(0xE000ED30))));
  //printf ("AFSR = 0x%x\r\n", (*((volatile unsigned long *)(0xE000ED3C))));
  // printf ("SCB_SHCSR = %x\n", SCB->SHCSR);
 
  while (1);

}

