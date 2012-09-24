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
 *  Module definitions to use with the Stellaris LM3S6965 Microcontroller
 */

#include "atomport.h"


typedef volatile unsigned int       REG_DWORD ;
typedef volatile unsigned short     REG_WORD ;
typedef volatile unsigned char      REG_BYTE ;


// *****************************************************************************
// The Stellaris General-Purpose Timer Module (GPTM)
// *****************************************************************************
typedef struct GPTM_TIMER_S {

                                            // offset  read/write       reset           Description
        REG_DWORD       CFG       ;         // 0x000   R/W              0x00000000     GPTM Configuration 345
        REG_DWORD       TAMR      ;         // 0x004   R/W              0x00000000     GPTM TimerA Mode 346
        REG_DWORD       TBMR      ;         // 0x008   R/W              0x00000000     GPTM TimerB Mode 348
        REG_DWORD       CTL       ;         // 0x00C   R/W              0x00000000     GPTM Control 350
        REG_DWORD       Reserved[2] ;       // 0x010
        REG_DWORD       IMR       ;         // 0x018   R/W              0x00000000     GPTM Interrupt Mask 353
        REG_DWORD       RIS       ;         // 0x01C   RO               0x00000000     GPTM Raw Interrupt Status 355
        REG_DWORD       MIS       ;         // 0x020   RO               0x00000000     GPTM Masked Interrupt Status 356
        REG_DWORD       ICR       ;         // 0x024   W1C              0x00000000     GPTM Interrupt Clear 357
        REG_DWORD       TAILR     ;         // 0x028   R/W              0xFFFFFFFF     GPTM TimerA Interval Load 359
        REG_DWORD       TBILR     ;         // 0x02C   R/W              0x0000FFFF     GPTM TimerB Interval Load 360
        REG_DWORD       TAMATCHR  ;         // 0x030   R/W              0xFFFFFFFF     GPTM TimerA Match 361
        REG_DWORD       TBMATCHR  ;         // 0x034   R/W              0x0000FFFF     GPTM TimerB Match 362
        REG_DWORD       TAPR      ;         // 0x038   R/W              0x00000000     GPTM TimerA Prescale 363
        REG_DWORD       TBPR      ;         // 0x03C   R/W              0x00000000     GPTM TimerB Prescale 364
        REG_DWORD       TAPMR     ;         // 0x040   R/W              0x00000000     GPTM TimerA Prescale Match 365
        REG_DWORD       TBPMR     ;         // 0x044   R/W              0x00000000     GPTM TimerB Prescale Match 366
        REG_DWORD       TAR       ;         // 0x048   RO               0xFFFFFFFF     GPTM TimerA 367
        REG_DWORD       TBR       ;         // 0x04C   RO               0x0000FFFF     GPTM TimerB 368

} GPTM_TIMER_T, *PGPTM_TIMER_T ;

// -------- GPTM_TIMER_CFG : (CFG Offset: 0x00) This register configures the global operation of the GPTM module -------- 
#define GPTM_TIMER_CFG_MASK                 ((unsigned int)0x07 << 0)		// 
    #define GPTM_TIMER_CFG_32BIT            ((unsigned int)0x00 << 0)		// 32-bit timer configuration
    #define GPTM_TIMER_CFG_32BIT_RT         ((unsigned int)0x01 << 0)		// 32-bit real-time clock (RTC) counter configuration
// -------- GPTM_TIMER_TAMR : (TAMR Offset: 0x04) This register configures the GPTM based on the configuration selected in the GPTMCFG register -------- 
// -------- GPTM_TIMER_TBMR : (TBMR Offset: 0x08) This register configures the GPTM based on the configuration selected in the GPTMCFG register -------- 
#define GPTM_TIMER_TMR_TAMS                 ((unsigned int)0x01 << 3)		// GPTM TimerA Alternate Mode Select. 0 Capture mode is enabled. 1 PWM mode is enabled
#define GPTM_TIMER_TMR_TCMR                 ((unsigned int)0x01 << 2)		// GPTM TimerA Capture Mode. 0 Edge-Count mode. 1 Edge-Time mode.
#define GPTM_TIMER_TMR_TMR_ONE_SHOT         ((unsigned int)0x01 << 0)		// One-Shot Timer mode
#define GPTM_TIMER_TMR_TMR_PERIODIC         ((unsigned int)0x02 << 0)		// Periodic Timer mode
#define GPTM_TIMER_TMR_TMR_CAPTURE          ((unsigned int)0x03 << 0)		// Capture mode
// -------- GPTM_TIMER_CTL : (CTL Offset: 0x0C) This register is used alongside the GPTMCFG and GMTMTnMR registers to fine-tune the timer configuration -------- 
#define GPTM_TIMER_CTL_TBPWML                 ((unsigned int)0x01 << 14)		// GPTM TimerB PWM Output Level. 0 Output is unaffected. 1 Output is inverted.
#define GPTM_TIMER_CTL_TBOTE                  ((unsigned int)0x01 << 13)		// GPTM TimerB Output Trigger Enable. 0 The output TimerB ADC trigger is disabled. 1 The output TimerB ADC trigger is enabled.
#define GPTM_TIMER_CTL_TBEVENT_MASK           ((unsigned int)0x03 << 10)		// GPTM TimerB Event Mode
    #define GPTM_TIMER_CTL_TBEVENT_PE         ((unsigned int)0x00 << 10)		// Positive edge
    #define GPTM_TIMER_CTL_TBEVENT_NE         ((unsigned int)0x01 << 10)		// Negative edge
    #define GPTM_TIMER_CTL_TBEVENT_NE         ((unsigned int)0x03 << 10)		// Both edges
#define GPTM_TIMER_CTL_TBSTALL                ((unsigned int)0x01 << 9)		// GPTM Timer B Stall Enable. 0 Timer B continues counting while the processor is halted by the debugger
#define GPTM_TIMER_CTL_TBEN                   ((unsigned int)0x01 << 8)		// GPTM TimerB Enable
// --------
#define GPTM_TIMER_CTL_TAPWML                 ((unsigned int)0x01 << 6)		// GPTM TimerA PWM Output Level. 0 Output is unaffected. 1 Output is inverted.
#define GPTM_TIMER_CTL_TAOTE                  ((unsigned int)0x01 << 5)		// GPTM TimerA Output Trigger Enable. 0 The output TimerB ADC trigger is disabled. 1 The output TimerB ADC trigger is enabled.
#define GPTM_TIMER_CTL_RTCEN                  ((unsigned int)0x01 << 4)		// GPTM RTC Enable
#define GPTM_TIMER_CTL_TAEVENT_MASK           ((unsigned int)0x03 << 2)		// GPTM TimerA Event Mode
    #define GPTM_TIMER_CTL_TAEVENT_PE         ((unsigned int)0x00 << 2)		// Positive edge
    #define GPTM_TIMER_CTL_TAEVENT_NE         ((unsigned int)0x01 << 2)		// Negative edge
    #define GPTM_TIMER_CTL_TAEVENT_NE         ((unsigned int)0x03 << 2)		// Both edges
#define GPTM_TIMER_CTL_TASTALL                ((unsigned int)0x01 << 1)		// GPTM Timer A Stall Enable. 0 Timer B continues counting while the processor is halted by the debugger
#define GPTM_TIMER_CTL_TAEN                   ((unsigned int)0x01 << 0)		// GPTM TimerA Enable
// -------- GPTM_TIMER_IMR : (IMR Offset: 0x18) This register allows software to enable/disable GPTM controller-level interrupts. -------- 
// -------- GPTM_TIMER_RIS : (RIS Offset: 0x1C) This register shows the state of the GPTM's internal interrupt signal. -------- 
// -------- GPTM_TIMER_MIS : (MIS Offset: 0x20) This register show the state of the GPTM's controller-level interrupt. -------- 
// -------- GPTM_TIMER_ICR : (ICR Offset: 0x24) This register is used to clear the status bits in the GPTMRIS and GPTMMIS registers. -------- 
#define GPTM_TIMER_INT_CBEIM                 ((unsigned int)0x01 << 10)		// GPTM CaptureB Event Interrupt Mask
#define GPTM_TIMER_INT_CBMIM                 ((unsigned int)0x01 << 9)		// GPTM CaptureB Match Interrupt Mask
#define GPTM_TIMER_INT_TBTOIM                ((unsigned int)0x01 << 8)		// GPTM TimerB Time-Out Interrupt Mask
// --------
#define GPTM_TIMER_INT_RTCIM                 ((unsigned int)0x01 << 3)		// GPTM RTC Interrupt Mask
#define GPTM_TIMER_INT_CAEIM                 ((unsigned int)0x01 << 2)		// GPTM CaptureA Event Interrupt Mask
#define GPTM_TIMER_INT_CAMIM                 ((unsigned int)0x01 << 1)		// GPTM CaptureA Match Interrupt Mask
#define GPTM_TIMER_INT_TATOIM                ((unsigned int)0x01 << 0)		// GPTM TimerA Time-Out Interrupt Mask



// *****************************************************************************
// Cortex M System Timer (SysTick) 
// *****************************************************************************
typedef struct SYSTICK_S {

			REG_DWORD		Res0[1]  ;              // 0xE000E000
			REG_DWORD		ICT   ;                 // 0xE000E004
			REG_DWORD		Res1[2]  ;              // 0xE000E008
			REG_DWORD		STCTRL   ;              // 0xE000E010
			REG_DWORD		STRELOAD ;              // 0xE000E014
			REG_DWORD		STCURRENT;              // 0xE000E018
			REG_DWORD		STCALIB ;               // 0xE000E01C
			REG_DWORD		Res2[56] ;              // 0xE000E020

} SYSTICK_T, *PSYSTICK_T ;

// -------- SYSTICK_STCTRL : (STCTRL Offset: 0xE000E010) SysTick Control and Status Register -------- 
#define SYSTICK_STCTRL_COUNT                 ((unsigned int)0x1 << 16)		// 0 - The SysTick timer has not counted to 0 since the last time this bit was read.
#define SYSTICK_STCTRL_CLK                   ((unsigned int)0x1 << 2)		// 1 - System clock
#define SYSTICK_STCTRL_INTEN                 ((unsigned int)0x1 << 1)		// 1 - An interrupt is generated to the NVIC when SysTick counts to 0.
#define SYSTICK_STCTRL_ENABLE                ((unsigned int)0x1 << 1)		// Enables SysTick to operate in a multi-shot way.
// -------- SYSTICK_STRELOAD : (STRELOAD Offset: 0xE000E014) Reload Value -------- 
#define SYSTICK_STRELOAD_MASK                ((unsigned int)0xFFFFFF << 0)		// IRQ mask
// -------- SYSTICK_STCURRENT : (STCURRENT Offset: 0xE000E018) SysTick Current Value Register -------- 


// *****************************************************************************
// Cortex M Nested Vectored Interrupt Controller 
// *****************************************************************************
typedef struct NVIC_S {

            REG_DWORD		ISER[2]  ;              // 0xE000E100
			REG_DWORD		Res3[30] ;              // 0xE000E120
			REG_DWORD		ICER[2]  ;              // 0xE000E180
			REG_DWORD		Res4[30] ;              // 0xE000E1A0
			REG_DWORD		ISPR[2]  ;              // 0xE000E200
			REG_DWORD		Res5[30] ;              // 0xE000E220
			REG_DWORD		ICPR[2]  ;              // 0xE000E280
			REG_DWORD		Res6[30] ;              // 0xE000E2A0
			REG_DWORD		IABR[2]  ;              // 0xE000E300
			REG_DWORD		Res7[64] ;              // 0xE000E320
			REG_DWORD		IPR[2]  ;              // 0xE000E400
			// REG_DWORD		Res7[515]  ;            // 0xE000E4F4

} NVIC_T, *PNVIC_T ;

#define NVIC_EXCEPTION_RESET                        1
#define NVIC_EXCEPTION_NMI                          2
#define NVIC_EXCEPTION_HARD_FAULT                   3
#define NVIC_EXCEPTION_MEM_MANAGEMENT               4
#define NVIC_EXCEPTION_BUS_FAULT                    5
#define NVIC_EXCEPTION_USAGE_FAULT                  6
#define NVIC_EXCEPTION_SVCALL                       11
#define NVIC_EXCEPTION_DEBUG_MON                    12
#define NVIC_EXCEPTION_PEND_SV                      14
#define NVIC_EXCEPTION_SYS_TICK                     15

// *****************************************************************************
// System Control Block (SCB) Registers 
// *****************************************************************************
typedef struct SCB_S {

            REG_DWORD       CPUID ;                 // 0xE000ED00
            REG_DWORD       ICSR ;                  // 0xE000ED04
            REG_DWORD       VTOR ;                  // 0xE000ED08
            REG_DWORD       AIRCR ;                 // 0xE000ED0C
            REG_DWORD       SCR ;                   // 0xE000ED10
            REG_DWORD       CCR ;                   // 0xE000ED14

            REG_DWORD       SYS_PRIO[3] ;           // 0xE000ED18
            REG_DWORD       SYSHNDCTRL ;            // 0xE000ED24
            //REG_DWORD       FAULTSTAT ;           // 0xE000ED28
            //REG_DWORD       HFAULTSTAT ;          // 0xE000ED2C

} SCB_T, *PSCB_T ;



#define BOARD_BASE_ADDRESS_SYSTICK                      0xE000E000 
#define BOARD_BASE_ADDRESS_NVIC                         0xE000E100 
#define BOARD_BASE_ADDRESS_SCB                          0xE000ED00 
#define BOARD_BASE_ADDRESS_GPTIMER0                     0x40030000 

extern SYSTICK_T*       const                           board_systick ;
extern NVIC_T*          const                           board_nvic ;
extern SCB_T*           const                           board_scb ;
extern GPTM_TIMER_T*    const                           board_gptm0 ;



extern int              low_level_init (void) ;
extern void             dbg_format_msg (char *format, ...)  ;
extern void             dbg_hard_fault_handler_c (unsigned int * hardfault_args) ;

#define DBG_MESSAGE(fmt_str)				{  dbg_format_msg fmt_str  ; }

#endif /* __MODULES_H__ */
