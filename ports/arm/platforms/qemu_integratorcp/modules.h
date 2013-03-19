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
#ifndef __MODULES_H__
#define __MODULES_H__

/*
 *  Module definitions to use with the ARM Integrator/CP (ARM926EJ-S)
 */

#include "atomport.h"


/* IO definitions (access restrictions to peripheral registers) */
#define     __I     volatile           /*!< defines 'read only' permissions                 */
#define     __O     volatile           /*!< defines 'write only' permissions                */
#define     __IO    volatile           /*!< defines 'read / write' permissions              */


// *****************************************************************************
// INTEGRATORCP TIMER
// *****************************************************************************
typedef struct ICP_TIMER_S {
                                                    // offset  read/write   word size  reset           Description
        __IO uint32_t       LOAD      ;                 // 0x0000  Read/write   32         0x00000000      Load value for Timer
        __I  uint32_t       VALUE     ;                 // 0x0004  Read         32         0xFFFFFFFF      The current value for Timer
        __IO uint8_t        CONTROL   ;                 // 0x0008  Read/write   8          0x20            Timer control register
        __O  uint32_t       INTCLR    ;                 // 0x000C  Write        - -                        Timer interrupt clear
        __I  uint32_t       RIS       ;                 // 0x0010  Read         1          0x0             Timer raw interrupt status
        __I  uint32_t       MIS       ;                 // 0x0014  Read         1          0x0             Timer masked interrupt status
        __IO uint32_t       BGLOAD    ;                 // 0x0018  Read/write   32         0x00000000      Background load value for Timer

} ICP_TIMER_T, *PICP_TIMER_T ;

// -------- ICP_TIMER_LOAD : (LOAD Offset: 0x00) Load value for Timer -------- 
// -------- ICP_TIMER_VALUE : (LOAD Offset: 0x04) The current value for Timer -------- 
// -------- ICP_TIMER_CONTROL : (CONTROL Offset: 0x04) Timer control register -------- 
#define ICP_TIMER_CONTROL_MASK                 ((unsigned int)0x0F << 0)        // Timer control mask
    #define ICP_TIMER_CONTROL_ENABLE           ((unsigned int)0x01 << 7)        // Timer enable: 0 = disabled 1 = enabled.
    #define ICP_TIMER_CONTROL_MODE             ((unsigned int)0x01 << 6)        // Timer mode: 0 = free running, counts once and then wraps to 0xFFFF 1 = periodic, reloads from load register at the end of each count..
    #define ICP_TIMER_CONTROL_IE               ((unsigned int)0x01 << 5)        // Interrupt enable.
    #define ICP_TIMER_CONTROL_R                ((unsigned int)0x01 << 4)        // Unused, always write as 0s.
    #define ICP_TIMER_CONTROL_PRESCALE_MASK    ((unsigned int)0x03 << 2)        // Prescale divisor
    #define ICP_TIMER_CONTROL_PRESCALE_NONE    ((unsigned int)0x00 << 2)        // 
    #define ICP_TIMER_CONTROL_PRESCALE_16      ((unsigned int)0x01 << 2)        // 
    #define ICP_TIMER_CONTROL_PRESCALE_256     ((unsigned int)0x02 << 2)        // 
#define ICP_TIMER_CONTROL_TIMER_SIZE           ((unsigned int)0x01 << 1)        // Selects 16/32 bit counter operation: 0 = 16-bit counter (default) 1 = 32-bit counter For 16-bit mode, write the high 16 bits of the 32-bit value as 0.
#define ICP_TIMER_CONTROL_ONE_SHOT             ((unsigned int)0x01 << 0)        // Selects one-shot or wrapping counter mode: 0 = wrapping mode (default) 1 = one-shot mode
// -------- ICP_TIMER_INTCLR : (INTCLR Offset: 0x0C) Timer interrupt clear -------- 
// -------- ICP_TIMER_RIS : (RIS Offset: 0x10) Timer raw interrupt status -------- 
// -------- ICP_TIMER_MIS : (MIS Offset: 0x14) Timer masked interrupt status -------- 
#define ICP_TIMER_INT                          ((unsigned int)0x01 << 0)        // Interrupt
// -------- ICP_TIMER_BGLOAD : (BGLOAD Offset: 0x18) Timer masked interrupt status -------- 


// *****************************************************************************
// INTEGRATORCP PIC
// *****************************************************************************
typedef struct ICP_PIC_S {
                                                    // offset  read/write   word size  reset           Description
        __I  uint32_t       IRQ_STATUS      ;           // 0x0000  Read         22                         IRQ gated interrupt status
        __I  uint32_t       IRQ_RAWSTAT     ;           // 0x0004  Read         22                         IRQ raw interrupt status
        __IO uint32_t       IRQ_ENABLESET   ;           // 0x0008  Read/write   22                         IRQ enable set
        __O  uint32_t       IRQ_ENABLECLR   ;           // 0x000C  Write        22                         IRQ enable clear
        __IO uint32_t       INT_SOFTSET     ;           // 0x0010  Read/write   16                         Software interrupt set
        __O  uint32_t       INT_SOFTCLR     ;           // 0x0014  Write        16                         Software interrupt clear
             uint32_t       RESERVED[2]     ;           // 0x0018  
        __I  uint32_t       FIQ_STATUS      ;           // 0x0020  Read         22                         FIQ gated interrupt status
        __I  uint32_t       FIQ_RAWSTAT     ;           // 0x0024  Read         22                         FIQ raw interrupt status
        __IO uint32_t       FIQ_ENABLESET   ;           // 0x0028  Read/write   22                         FIQ enable set
        __O  uint32_t       FIQ_ENABLECLR   ;           // 0x002C  Write-only   22                         FIQ enable clear

} ICP_PIC_T, *PICP_PIC_T ;

// -------- ICP_PIC_IRQ_STATUS : (IRQ_STATUS Offset: 0x00) IRQ gated interrupt status -------- 
// -------- ICP_PIC_IRQ_RAWSTAT : (IRQ_RAWSTAT Offset: 0x04) IRQ raw interrupt status -------- 
// -------- ICP_PIC_IRQ_ENABLESET : (IRQ_ENABLESET Offset: 0x08) IRQ enable set -------- 
// -------- ICP_PIC_IRQ_ENABLECLR : (IRQ_ENABLECLR Offset: 0x0C) IRQ enable clear -------- 
#define ICP_PIC_IRQ_MASK                       ((unsigned int)0x3FFFFF << 0)    // IRQ mask
    #define ICP_PIC_IRQ_TIMERINT2              ((unsigned int)0x01 << 7)        // TIMERINT2 Counter-timer 2 interrupt
    #define ICP_PIC_IRQ_TIMERINT1              ((unsigned int)0x01 << 6)        // TIMERINT1 Counter-timer 1 interrupt
    #define ICP_PIC_IRQ_TIMERINT0              ((unsigned int)0x01 << 5)        // TIMERINT0 Counter-timer 0 interrupt
    #define ICP_PIC_IRQ_SOFTINT                ((unsigned int)0x01 << 0)        // OFTINT Software interrupt
// -------- ICP_PIC_INT_SOFTSET : (INT_SOFTSET Offset: 0x10) Software interrupt set -------- 
// -------- ICP_PIC_INT_SOFTCLR : (INT_SOFTCLR Offset: 0x14) Software interrupt clear -------- 



/* module definitions */
#define BOARD_BASE_ADDRESS_TIMER_0             0x13000000 
#define BOARD_BASE_ADDRESS_PIC                 0x14000000 

extern ICP_TIMER_T*       const                board_timer_0 ;
extern ICP_PIC_T*         const                board_pic ;


/* Function prototypes */
extern int              low_level_init (void) ;


#endif /* __MODULES_H__ */
