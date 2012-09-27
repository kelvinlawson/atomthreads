/*
  Copyright (c) 2012, Natie van Rooyen. All rights reserved.
 
  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:
 
  1. Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
  2. Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
  3. No personal names or organizations' names associated with the
     Atomthreads project may be used to endorse or promote products
     derived from this software without specific prior written permission.
 
  THIS SOFTWARE IS PROVIDED BY THE ATOMTHREADS PROJECT AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
  TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE PROJECT OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
*/

/* lm3s Startup Script */

.section .vectors,"x",%progbits
.syntax unified
.thumb


.global    __interrupt_vector_table
.extern    archTickHandler
.extern    archPendSVHandler
.extern    dbg_hard_fault_handler_c

/**
 *  \b __interrupt_vector_table
 *
 */
__interrupt_vector_table:
.long    __c_stack_top__
.long    reset_Handler
.long    fault_Handler
.long    fault_Handler
.long    fault_Handler
.long    fault_Handler
.long    fault_Handler
.long    0
.long    0
.long    0
.long    0
.long    sys_Handler
.long    sys_Handler
.long    0
.long    archPendSVHandler
.long    sys_Handler

/* External interrupts */
.long    default_Handler                      // GPIO Port A
.long    default_Handler                      // GPIO Port B
.long    default_Handler                      // GPIO Port C
.long    default_Handler                      // GPIO Port D
.long    default_Handler                      // GPIO Port E
.long    default_Handler                      // UART0 Rx and Tx
.long    default_Handler                      // UART1 Rx and Tx
.long    default_Handler                      // SSI0 Rx and Tx
.long    default_Handler                      // I2C0 Master and Slave
.long    default_Handler                      // PWM Fault
.long    default_Handler                      // PWM Generator 0
.long    default_Handler                      // PWM Generator 1
.long    default_Handler                      // PWM Generator 2
.long    default_Handler                      // Quadrature Encoder 0
.long    default_Handler                      // ADC Sequence 0
.long    default_Handler                      // ADC Sequence 1
.long    default_Handler                      // ADC Sequence 2
.long    default_Handler                      // ADC Sequence 3
.long    default_Handler                      // Watchdog timer
.long    archTickHandler                      // Timer 0 subtimer A
.long    default_Handler                      // Timer 0 subtimer B
.long    default_Handler                      // Timer 1 subtimer A
.long    default_Handler                      // Timer 1 subtimer B
.long    default_Handler                      // Timer 2 subtimer A
.long    default_Handler                      // Timer 2 subtimer B
.long    default_Handler                      // Analog Comparator 0
.long    default_Handler                      // Analog Comparator 1
.long    default_Handler                      // Analog Comparator 2
.long    default_Handler                      // System Control (PLL, OSC, BO)
.long    default_Handler                      // FLASH Control
.long    default_Handler                      // GPIO Port F
.long    default_Handler                      // GPIO Port G
.long    default_Handler                      // GPIO Port H
.long    default_Handler                      // UART2 Rx and Tx
.long    default_Handler                      // SSI1 Rx and Tx
.long    default_Handler                      // Timer 3 subtimer A
.long    default_Handler                      // Timer 3 subtimer B
.long    default_Handler                      // I2C1 Master and Slave
.long    default_Handler                      // Quadrature Encoder 1
.long    default_Handler                      // CAN0
.long    default_Handler                      // CAN1
.long    default_Handler                      // CAN2
.long    default_Handler                      // Ethernet
.long    default_Handler                      // Hibernate

/**
 *  \b sys_Handler
 *
 *  @return None
 */
.thumb_func
sys_Handler:
    B .

/**
 *  \b default_Handler
 *
 *  @return None
 */
.thumb_func
default_Handler:
    B .

/**
 *  \b fault_Handler
 *
 *  @return None
 */
 .thumb_func
fault_Handler:
    tst     lr, #4
    ite     eq
    mrseq   r0, MSP
    mrsne   r0, PSP
    b       dbg_hard_fault_handler_c


.section .startup,"x",%progbits
.syntax unified
.thumb

.global reset_Handler
.extern initialise_monitor_handles
.extern low_level_init
.extern main


/**
 *  \b reset_Handler
 *
 *  
 *
 *  @return None
 */
.thumb_func
reset_Handler:

/* 
 * Initialize the data and bss sections.
 */
init_data:
    ldr    r0, .ETEXT
    ldr    r1, .DATA
    ldr    r2, .EDATA
    sub    r2, r2, r1
    cmp    r2, #0
    beq    init_bss
init_data_copy:
    ldrb   r4, [r0], #1
    strb   r4, [r1], #1
    subs   r2, r2, #1
    bne    init_data_copy
init_bss:
    mov    r0, #0
    ldr    r1, = .BSS
    ldr    r2, = .EBSS
    sub    r2, r2, r1
    cmp    r2, #0
    beq    init_done
init_bss_zero:
    strb   r0, [r1], #1
    subs   r2, r2, #1
    bne    init_bss_zero
init_done:

    /*
     * The following call initializes the function pointers for stdio etc.
     * These are used by the semihosting interface.
     *
     * This function is implemented in newlib.
     */
    bl initialise_monitor_handles
    
    /*
     * Platform specific low level initialization.
     */
    bl low_level_init 

    /*
     * Call the application's entry point.
     */
    bl main


.BSS:    .long    _bss
.EBSS:   .long    _ebss
.ETEXT:  .long    _etext
.DATA:   .long    _data
.EDATA:  .long    _edata

.end