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


.global    contextInit
.global    contextSwitch
.global    contextStart
.global    contextEnableInterrupts
.global    contextEnterCritical
.global    contextExitCritical
.global    pendSV_Handler
.global    tick_Handler


.global    __context_tick_handler


/**/
.equ NVIC_INT_CTRL,                     0xE000ED04        //    Interrupt control state register
.equ NVIC_PENDSVSET,                    0x10000000        //    Value to trigger PendSV exception
.equ NVIC_PR_12_15_ADDR,                0xE000ED20        //    System Handlers 12-15 Priority Register Address
.equ NVIC_PENDS_VPRIORITY,              0x00F00000        //    PendSV priority is minimal (0xFF -- 0x00FF0000)

#ifdef PLATFORM_QEMU_LM3S_HACK
.equ NVIC_ISER,							0xE000E100
.equ NVIC_ICER,							0xE000E180
.equ NVIC_I_TIMER0,						0x80000
#endif


.syntax unified
.text
.thumb


/**
 *  \b contextInit
 *
 *  Architecture-specific one time initialization.
 *
 *  Configures PendSV priority to lowest.
 *
 *  @return None
 */
contextInit:
        LDR            r1, =NVIC_PR_12_15_ADDR        // Load the System 12-15 Priority Register
        LDR            r0, [r1]
        ORR            r0, r0, #NVIC_PENDS_VPRIORITY  // set PRI_14 (PendSV) to 0xFF - minimal
        STR            r0, [r1]

        BX             lr

/**
 *  \b contextSwitch
 *
 *  Architecture-specific context switch routine.
 *
 *  Note that interrupts are always locked out when this routine is
 *  called. For cooperative switches, the scheduler will have entered
 *  a critical region. For preemptions (called from an ISR), the
 *  interrupts will have disabled in the tick_Handler.
 *
 *  @param[in] [r0] -> Address to save old stack pointer
 *  @param[in] [r1] -> Address where new stack pointer is stored
 *
 *  @return None
 */
contextSwitch:
        LDR            r2, =context_new_stack_ptr
        STR            r1, [r2]

        LDR            r2, =context_save_stack_ptr
        LDR            r1, [r2]
        CMP            r1, #0                //    if contextSwitch is going to be called again before pend_sv
        IT             EQ
        STREQ          r0, [r2]

        LDR            R0, =NVIC_INT_CTRL    //    Trigger the PendSV exception (causes context switch)
        LDR            R1, =NVIC_PENDSVSET
        STR            R1, [R0]

        BX             lr

/**
 *  \b contextStart
 *
 *  Architecture-specific context start routine.
 *
 *  @param[in] [r0] -> Address where stack pointer is stored
 *
 *  @return Does not return
 */
contextStart:
        LDR            r1, =context_new_stack_ptr
        STR            r0, [r1]
        LDR            r1, =context_save_stack_ptr
        MOV            r0, #0
        STR            r0, [r1]
        LDR            r0, =NVIC_INT_CTRL    //    Trigger the PendSV exception (causes context switch)
        LDR            r1, =NVIC_PENDSVSET
        STR            r1, [r0]

        BX             lr

/**
 *  \b contextEnableInterrupts
 *
 *  Enables interrupts on the processor
 *
 *  @return None
 */
contextEnableInterrupts:
#ifndef PLATFORM_QEMU_LM3S_HACK
        CPSIE          i
#else
        LDR            r1, =NVIC_ISER
        LDR            r0, =NVIC_I_TIMER0			   
		STR            r0, [r1]
#endif
        BX             lr


/**
 *  \b contextExitCritical
 *
 *  Exit critical section (restores interrupt posture)
 *
 *  @param[in] r0 Interrupt Posture
 *
 *  @return None
 */
contextExitCritical:
#ifndef PLATFORM_QEMU_LM3S_HACK
        MSR            PRIMASK, r0
#else
        LDR            r1, =NVIC_ISER
        STR            r0, [r1]			   
#endif
        BX             lr

/**
 *  \b contextEnterCritical
 *
 *  Enter critical section (disables interrupts)
 *
 *  @return Current interrupt posture
 */
contextEnterCritical:
#ifndef PLATFORM_QEMU_LM3S_HACK
        MRS            r0, PRIMASK
        CPSID          i
#else
        LDR            r1, =NVIC_ISER
        LDR            r0, [r1]			   
        LDR            r1, =NVIC_ICER
 		STR            r0, [r1]
#endif
        BX             lr

/**
 *  \b PendSV_Handler
 *
 *  CortexM3 PendSV_Handler. Switch context to a new stack.
 *
 *  @return None
 */
 .thumb_func
pendSV_Handler:
#ifndef PLATFORM_QEMU_LM3S_HACK
        CPSID          i                   //    Disable core int
#else
        LDR            r0, =NVIC_ICER
        LDR            r1, =NVIC_I_TIMER0			   
		STR            r1, [r0]
#endif

        LDR            r1, =context_save_stack_ptr
        LDR            r0, [r1]			   //    Load old (current) stack pointer address

        LDR            r2, =context_new_stack_ptr
        LDR            r2, [r2]            //    Load new stack pointer address
        CMP            r0, r2
        BEQ            pendsv_handler_exit

        CMP            r0, #0
        BEQ            pendsv_handler_new_stack
		                                   //    Save context
        MRS            r3, PSP             //    Get PSP point
        STMDB          r3!, {R4-R11}       //    Store r4-r11
        STR            r3, [r0]            //    Save old stack pointer
        MOV            r3, #0
        STR            r3, [r1]

pendsv_handler_new_stack:
                                           //    Restore context
        LDR            r2, [r2]            //    Load new stack pointer
        LDMIA          r2!, {r4-r11}       //    Restore context
        MSR            PSP, r2             //    Mov new stack point to PSP

pendsv_handler_exit:
#ifndef PLATFORM_QEMU_LM3S_HACK
        CPSIE         i                    //    Enable core int
#else
        LDR            r0, =NVIC_ISER
        LDR            r1, =NVIC_I_TIMER0			   
		STR            r1, [r0]
#endif

        ORR           lr, lr, #0x04        //    Ensure exception return uses process stack
        BX            lr                   //    Exit interrupt


/**
 *  \b Tick_Handler
 *
 *  System timer tick interrupt handler.
 *
 *  @return None
 */
 .thumb_func
tick_Handler:
        PUSH {r4-r11, lr}

#ifndef PLATFORM_QEMU_LM3S_HACK
        cpsid  I                           //   Disable core int
#else
        LDR            r0, =NVIC_ICER
        LDR            r1, =NVIC_I_TIMER0		   
		STR            r1, [r0]
#endif

        BL  __context_tick_handler

#ifndef PLATFORM_QEMU_LM3S_HACK
        cpsie  I                           //   Enable core int
#else
        LDR            r0, =NVIC_ISER
        LDR            r1, =NVIC_I_TIMER0			   
		STR            r1, [r0]
#endif

        POP {r4-r11, pc}


/**/
.data
context_new_stack_ptr:         .long    0x00000000
context_save_stack_ptr:        .long    0x00000000


