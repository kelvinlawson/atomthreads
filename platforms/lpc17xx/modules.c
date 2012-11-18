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
    //CRITICAL_STORE ;

    va_start (args, format) ;
    //CRITICAL_START() ;
    
	vsniprintf ((char*)msg, 256, (char*)format, args) ;
    LPC17xx_UART_PutString (msg) ;
    //CRITICAL_END() ;

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
    SystemInit () ;
    SystemCoreClockUpdate ();
    //contextInit () ;
    NVIC_SetPriority (PendSV_IRQn, 0xFF) ;
    LPC17xx_UART_Init (115200) ;
    SysTick_Config (1000000) ;

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

    /* Call the interrupt enter routine */
    atomIntEnter();

    /* Call the OS system tick handler */
    atomTimerTick();

    /* Call the interrupt exit routine */
    atomIntExit(TRUE);

}

/**
 * \b dbg_mem_dump_40
 *
 * Dumps size bytes of memory from data.
 *
 */
void dbg_mem_dump_40 (unsigned int* data, int size)
{
	int j ;

    dbg_format_msg ("Dump %d bytes at %.8X:\r\n",size * 4, (unsigned int)data) ; 
    data =  (unsigned int*)((unsigned int)data & ~0x3) ;
    for (j=0; j<size-3; j+=4) {
	    dbg_format_msg (" :%.8X: %.8X %.8X %.8X %.8X\r\n", (unsigned int)&data[j], data[j+0], data[j+1], data[j+2], data[j+3]) ; 
    }

    if (size-j == 3) {
	    dbg_format_msg (" :%.8X: %.8X %.8X %.8X\r\n", (unsigned int)&data[j], data[j+0], data[j+1], data[j+2]) ; 
    } else if (size-j == 2) {
	    dbg_format_msg (" :%.8X: %.8X %.8X\r\n", (unsigned int)&data[j], data[j+0], data[j+1]) ; 
    } else if (size-j == 1) {
	    dbg_format_msg (" :%.8X: %.8X\r\n", (unsigned int)&data[j], data[j+0]) ; 
    }

}

/**
 * \b dbg_fault_handler
 *
 * Prints cortex m exception debug information.
 *
 */
void
dbg_fault_handler (unsigned int * hardfault_args)
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
 
  dbg_format_msg ("\r\n\r\n[Hard fault handler - all numbers in hex]\r\n");
  dbg_format_msg ("SP = 0x%x\r\n", hardfault_args);
  dbg_format_msg ("R0 = 0x%x\r\n", stacked_r0);
  dbg_format_msg ("R1 = 0x%x\r\n", stacked_r1);
  dbg_format_msg ("R2 = 0x%x\r\n", stacked_r2);
  dbg_format_msg ("R3 = 0x%x\r\n", stacked_r3);
  dbg_format_msg ("R12 = 0x%x\r\n", stacked_r12);
  dbg_format_msg ("LR [R14] = 0x%x  subroutine call return address\r\n", stacked_lr);
  dbg_format_msg ("PC [R15] = 0x%x  program counter\r\n", stacked_pc);
  dbg_format_msg ("PSR = 0x%x\r\n", stacked_psr);
  //printf ("BFAR = 0x%x\r\n", (*((volatile unsigned long *)(0xE000ED38))));
  //printf ("CFSR = 0x%x\r\n", (*((volatile unsigned long *)(0xE000ED28))));
  //printf ("HFSR = 0x%x\r\n", (*((volatile unsigned long *)(0xE000ED2C))));
  //printf ("DFSR = 0x%x\r\n", (*((volatile unsigned long *)(0xE000ED30))));
  //printf ("AFSR = 0x%x\r\n", (*((volatile unsigned long *)(0xE000ED3C))));
  // printf ("SCB_SHCSR = %x\n", SCB->SHCSR);
  
  dbg_mem_dump_40 (hardfault_args, 0x40) ;

  while (1);

}

