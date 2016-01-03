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
.global    archContextSwitch
.global    archFirstThreadRestore


.extern __interrupt_dispatcher 

/* When using newlib, reentrancy context needs to be updated on task switch */
.extern _impure_ptr


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
 * \b archContextSwitch
 *
 * Architecture-specific context switch routine.
 *
 * Note that interrupts are always locked out when this routine is
 * called. For cooperative switches, the scheduler will have entered
 * a critical region. For preemptions (called from an ISR), the
 * ISR will have disabled interrupts on entry.
 *
 * @param[in] old_tcb_ptr Pointer to the thread being scheduled out
 * @param[in] new_tcb_ptr Pointer to the thread being scheduled in
 *
 * @return None
 *
 * void archContextSwitch (ATOM_TCB *old_tcb_ptr, ATOM_TCB *new_tcb_ptr)
 */
archContextSwitch:
    STMFD       sp!, {r4 - r11, lr}             /* Save registers */

    ADD         r4, r1, #4                      /* Add offset to get address of new_tcb_ptr->reent context (second TCB element) */
    LDR         r5, = _impure_ptr               /* Get address of _impure_ptr into r5 */
    STR         r4, [r5]                        /* Store new_tcb_ptr->reent context in _impure_ptr */

    STR         sp, [r0]                        /* Save old SP in old_tcb_ptr->sp_save_ptr (first TCB element) */
    LDR         r1, [r1]                        /* Load new SP from new_tcb_ptr->sp_save_ptr (first TCB element) */
    MOV         sp, r1                           

    LDMFD       sp!, {r4 - r11, pc}             /* Load new registers */


/**
 * \b archFirstThreadRestore
 *
 * Architecture-specific function to restore and start the first thread.
 * This is called by atomOSStart() when the OS is starting.
 *
 * This function will be largely similar to the latter half of
 * archContextSwitch(). Its job is to restore the context for the
 * first thread, and finally enable interrupts (although we actually
 * enable interrupts in thread_shell() for new threads in this port
 * rather than doing it explicitly here).
 *
 * It expects to see the context saved in the same way as if the
 * thread has been previously scheduled out, and had its context
 * saved. That is, archThreadContextInit() will have been called
 * first (via atomThreadCreate()) to create a "fake" context save
 * area, containing the relevant register-save values for a thread
 * restore.
 *
 * Note that you can create more than one thread before starting
 * the OS - only one thread is restored using this function, so
 * all other threads are actually restored by archContextSwitch().
 * This is another reminder that the initial context set up by
 * archThreadContextInit() must look the same whether restored by
 * archFirstThreadRestore() or archContextSwitch().
 *
 * @param[in] new_tcb_ptr Pointer to the thread being scheduled in
 *
 * @return None
 *
 * void archFirstThreadRestore (ATOM_TCB *new_tcb_ptr)
 */
archFirstThreadRestore:

    ADD         r4, r0, #4                      /* Add offset to get address of new_tcb_ptr->reent context (second TCB element) */
    LDR         r5, = _impure_ptr               /* Get address of _impure_ptr into r5 */
    STR         r4, [r5]                        /* Store new_tcb_ptr->reent context in _impure_ptr */

    LDR         r0, [r0]                        /* Get SP (sp_save_ptr is conveniently first element of TCB) */
    MOV         sp, r0                          /* Load new stack pointer */
    LDMFD       sp!, {r4 - r11, pc}             /* Load new registers */


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
 *  Save the process/thread context onto its own stack before calling __interrupt_dispatcher().
 *  __interrupt_dispatcher() might switch stacks. On return the same context is popped from the 
 *  stack and control is returned to the process.
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
                                                
    BL          __interrupt_dispatcher          /* Dispatch the interrupt to platform folder for
                                                   the timer tick interrupt or a simular function
                                                   for other interrupts. Some of those IRQs may
                                                   call Atomthreads kernel routines and cause a
                                                   thread switch. */

    LDMFD       sp!, {r1, r2}                   /* Restore lr_irq and spsr_irq from process stack */
    MSR         cpsr_c, #(IRQ_MODE | I_BIT)     
    STMFD       sp!, {r1}                       
    MSR         spsr_cxsf, r2                   
                                                
    MSR         cpsr_c, #(SVC_MODE | I_BIT)     /* Restore process regs */
    LDMFD       sp!, {r0 - r3, ip, lr}          
                                                
    MSR         cpsr_c, #(IRQ_MODE | I_BIT)     /* Exit from IRQ */
    LDMFD       sp!, {pc}^

