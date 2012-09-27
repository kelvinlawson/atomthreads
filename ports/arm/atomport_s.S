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



/**/



.global    archIRQHandler
.global    contextEnterCritical
.global    contextExitCritical
.global    contextEnableInterrupts
.global    contextId
.global    contextStart
.global    contextSwitch
.global    contextInit


.extern __context_preempt_handler 

/**/
.equ USR_MODE,            0x10
.equ FIQ_MODE,            0x11
.equ IRQ_MODE,            0x12
.equ SVC_MODE,            0x13
.equ ABT_MODE,            0x17
.equ UND_MODE,            0x1B
.equ SYS_MODE,            0x1F

.equ I_BIT,               0x80        /* when I bit is set, IRQ is disabled */
.equ F_BIT,               0x40        /* when F bit is set, FIQ is disabled */


.text
.code 32

/**
 *  \b contextInit
 *
 *  Architecture-specific one time initialization.
 *
 *  @return None
 */
contextInit:

        BX      lr

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
    STMFD       sp!, {r4 - r11, lr}             /* Save registers */

#ifdef CONTEXT_THREAD_ID
    MRC         p15, 0, r3, c13, c0, 2
    STMFD       sp!, {r3}
#endif

    STR         sp, [r0]                        /* Save old stack pointer */
    LDR         r1, [r1]                        /* Load new stack pointer */
    MOV         sp, r1                           

#ifdef CONTEXT_THREAD_ID
    LDMFD       sp!, {r3}
    MCR         p15, 0, r3, c13, c0, 2
#endif
	
    LDMFD       sp!, {r4 - r11, pc}             /* Load new registers */

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
    LDR         r0, [r0]
    MOV         sp, r0                          /* Load new stack pointer */

#ifdef CONTEXT_THREAD_ID
    LDMFD       sp!, {r3}
    MCR         p15, 0, r3, c13, c0, 2
#endif
	
    LDMFD       sp!, {r4 - r11, pc}             /* Load new registers */

/**
 *  \b contextId
 *
 *  Returns a unique ID for the context
 *
 *  @return ID
 */
contextId:
#ifdef CONTEXT_THREAD_ID
    MRC         p15, 0, r0, c13, c0, 2
#else
    MOV         r0, #0
#endif
    BX          lr

/**
 *  \b contextEnableInterrupts
 *
 *  Enables interrupts on the processor
 *
 *  @return None
 */
contextEnableInterrupts:
    MRS         r0, CPSR
    MOV         r1, #I_BIT
    BIC         r0, r0, r1
    MSR         CPSR_c, r0
    BX          lr

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
    MSR         CPSR_cxsf, r0
    BX          lr

/**
 *  \b contextEnterCritical
 *
 *  Enter critical section (disables interrupts)
 *
 *  @return Current interrupt posture
 */
contextEnterCritical:
    MRS         r0, CPSR
    ORR         r1, r0, #I_BIT
    MSR         CPSR_cxsf, r1
    BX          lr



/**
 *  \b archIRQHandler
 *
 *  IRQ entry point.
 *
 *  Save the process/thread context onto its own stackm before calling __context_preempt_handler ().
 *  __context_preempt_handler() might switch stacks. On return the same context is poped from the 
 *  stack and  control is returned to the process.
 *
 *  @return None
 */
archIRQHandler:

    MSR         cpsr_c, #(SVC_MODE | I_BIT)     /* Save current process context in process stack */
    STMFD       sp!, {r0 - r3, ip, lr}          
                                                
    MSR         cpsr_c, #(IRQ_MODE | I_BIT)     /* Save lr_irq and spsr_irq in process stack */
    SUB         lr, lr, #4                      
    MOV         r1, lr                          
    MRS         r2, spsr                        
    MSR         cpsr_c, #(SVC_MODE | I_BIT)     
    STMFD       sp!, {r1, r2}                   
                                                
    BL          __context_preempt_handler       /* Dispatch the interrupt to archTickHandler for
                                                   the timer tick interrupt or a simular function
                                                   for other interrupts which might call atomthread
                                                   functions. */

    LDMFD       sp!, {r1, r2}                   /* Restore lr_irq and spsr_irq from process stack */
    MSR         cpsr_c, #(IRQ_MODE | I_BIT)     
    STMFD       sp!, {r1}                       
    MSR         spsr_cxsf, r2                   
                                                
    MSR         cpsr_c, #(SVC_MODE | I_BIT)     /* Restore process regs */
    LDMFD       sp!, {r0 - r3, ip, lr}          
                                                
    MSR         cpsr_c, #(IRQ_MODE | I_BIT)     /* Exit from IRQ */
    LDMFD       sp!, {pc}^



