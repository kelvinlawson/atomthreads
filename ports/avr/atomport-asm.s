/*
 * Copyright (c) 2010, Atomthreads Project. All rights reserved.
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

#include <avr/io.h>


.section .text

/*
 * \b archContextSwitch
 *
 * Architecture-specific context switch routine.
 *
 * Note that interrupts are always locked out when this routine is
 * called. For cooperative switches, the scheduler will have entered
 * a critical region. For preemptions (called from an ISR), the
 * ISR will have disabled interrupts on entry.
 *
 * Note that this function might have been coded in C, but gcc
 * was generating prologue and epilogue code to handle the parameters.
 * Worse, with the naked attribute set it generated half of the
 * prologue/epilogue. Rather than work around the gcc code generation,
 * which may change from compiler version to compiler version, we
 * just write this function in asm, where we have absolute control
 * over the code generation. Important during register saves/restores.
 *
 * @param[in] old_tcb_ptr Pointer to the thread being scheduled out
 * @param[in] new_tcb_ptr Pointer to the thread being scheduled in
 *
 * @return None
 *
 * void archContextSwitch (ATOM_TCB *old_tcb_ptr, ATOM_TCB *new_tcb_ptr)
 */
.global archContextSwitch
archContextSwitch:

    /**
     * Parameter locations:
     *  old_tcb_ptr = R25-R24
     *  new_tcb_ptr = R23-R22
     */

    /**
     * If this is a cooperative context switch (a thread has called us
     * to schedule itself out), gcc will have saved any of the
     * registers R18-R27 and R30-R31 which it does not want us to clobber.
     * Any registers of that set which it did not need to save are safe
     * not to be saved by us anyway. Hence for cooperative context
     * switches we only need to save those registers which gcc expects
     * us _not_ to modify, that is R2-R17 and R28-R29.
     *
     * If we were called from an interrupt routine (because a thread
     * is being preemptively scheduled out), the situation is exactly
     * the same. Any ISR which calls out to a subroutine will have
     * similarly saved those registers which it needs us not to
     * clobber. In the case of an interrupt, that is every single
     * register of the set R18-R27 and R30-R31. (gcc cannot establish
     * which of those registers actually need to be saved because
     * the information is not available to an ISR). Again, we only
     * need to save the registers R2-R17 and R28-29, because these
     * are expected to be unclobbered by a subroutine.
     *
     * Note that in addition to saving R18-R27 and R30-R31, gcc also
     * saves R0, R1 and SREG when entering ISRs. In the case of a
     * cooperative context switch, it is not necessary to save these.
     */

    /**
     * Save registers R2-R17, R28-R29.
     */
    push r2
    push r3
    push r4
    push r5
    push r6
    push r7
    push r8
    push r9
    push r10
    push r11
    push r12
    push r13
    push r14
    push r15
    push r16
    push r17
    push r28
    push r29

    /**
     * On devices with large program space we also save RAMPZ, EIND.
     * Note that GCC 4.3 and later to actually save RAMPZ when called
     * via an interrupt handler, which means that we end up stacking
     * RAMPZ twice. However we do need to save RAMPZ for cooperative
     * context switches where we are called via a function call rather
     * than an ISR (at this time GCC does not save RAMPZ before function
     * calls). This has the added benefit that we continue to support
     * GCC < 4.3, but with the added overhead of the double-stacking for
     * newer versions of GCC.
     *
     * An alternative method that would work for GCC >= 4.3 only would be
     * to detect whether we were called from an interrupt handler and not
     * save RAMPZ under those circumstances.
     */
#ifdef __AVR_3_BYTE_PC__
    in r0,_SFR_IO_ADDR(RAMPZ)
    push r0
    in r0,_SFR_IO_ADDR(EIND)
    push r0
#endif

    /**
     * Save the final stack pointer to the TCB. The parameter pointing to
     * the old TCB is still untouched in R25-R24. We have saved R16/R17
     * and R28/R29 so we can use them for our own purposes now. We must be
     * careful not to use R23-R22, however, as these still contain the
     * other parameter, new_tcb_ptr.
     */
    in  r16,_SFR_IO_ADDR(SPL)  /* Get the current SP into general regs */
    in  r17,_SFR_IO_ADDR(SPH)  /* R16/R17 which are now free to use. */

    mov r28,r24         /* Move old_tcb_ptr param into the Y-regs so we */
    mov r29,r25         /* can access the TCB via a pointer. */

    st  Y,r16           /* Store SPH/SPL to old_tcb_ptr->sp_save_ptr which */
    std Y+1,r17         /* is conveniently the first member of the TCB. */


    /**
     * At this point, all of the current thread's context has been saved
     * so we no longer care about keeping the contents of any registers.
     *
     * The stack frame if this is a cooperative switch looks as follows:
     *
     *    <Any of R18-R27 and R30-R31 that the calling function saves>
     *    <Return address to calling function>
     *    <R2>
     *    <R3>
     *     ||
     *    <R16>
     *    <R17>
     *    <R28>
     *    <R29>
     *    <RAMPZ>     (Only certain devices)
     *    <EIND>      (Only certain devices)
     *
     * The stack frame if this was a preemptive switch looks as follows:
     *
     *   <R1>    // saved by ISR
     *   <R0>    //
     *   <SREG>  //
     *   <RAMPZ> //   (Only certain devices)
     *   <R18>   //
     *   <R19>   //
     *    ||     //
     *   <R26>   //
     *   <R27>   //
     *   <R30>   //
     *   <R31>   //
     *   <Return address to ISR>
     *   <Any stacking and return addresses between ISR and this call>
     *   <R2>
     *   <R3>
     *    ||
     *   <R16>
     *   <R17>
     *   <R28>
     *   <R29>
     *   <RAMPZ>     (Only certain devices)
     *   <EIND>      (Only certain devices)
     *
     *
     * In addition, the thread's stack pointer (after context-save) is
     * stored in the thread's TCB.
     */

    /**
     * We are now ready to restore the new thread's context. We switch
     * our stack pointer to the new thread's stack pointer, and pop
     * all of its context off the stack. When we have finished popping
     * all registers (R2-R17 and R28-R29), we are ready to return.
     *
     * Note that any further registers that needed to be saved for the
     * thread will be restored on exiting this function. If the new
     * thread previously scheduled itself out cooperatively, the
     * original calling function will restore any registers it chose
     * to save. If the new thread was preempted, we will return to the
     * ISR which will restore all other system registers, before
     * returning to the interrupted thread.
     */

    /**
     * Get the new thread's stack pointer off the TCB (new_tcb_ptr).
     * new_tcb_ptr is still stored in the parameter registers, R23-R22.
     * We are free to use any other registers, however, as we haven't
     * yet popped any of the new thread's context off its stack.
     */
    mov r28,r22         /* Move new_tcb_ptr into the Y-regs so we */
    mov r29,r23         /* can access the TCB via a pointer. */

    ld r16,Y            /* Load new_tcb_ptr->sp_save_ptr into R16/R17. */
    ldd r17,Y+1         /* It is conveniently the first member of the TCB. */

    out _SFR_IO_ADDR(SPL),r16  /* Set our stack pointer to the new thread's */
    out _SFR_IO_ADDR(SPH),r17  /* stack pointer, from its TCB. */

    /**
     * On devices with large program space we also restore RAMPZ, EIND.
     */
#ifdef __AVR_3_BYTE_PC__
    pop r0
    in r0,_SFR_IO_ADDR(EIND)
    pop r0
    in r0,_SFR_IO_ADDR(RAMPZ)
#endif

    /**
     * Restore registers R2-R17, R28-R29.
     */
    pop r29
    pop r28
    pop r17
    pop r16
    pop r15
    pop r14
    pop r13
    pop r12
    pop r11
    pop r10
    pop r9
    pop r8
    pop r7
    pop r6
    pop r5
    pop r4
    pop r3
    pop r2

    /**
     * The return address on the stack will now be the new thread's return
     * address - i.e. although we just entered this function from a
     * function called by the old thread, now that we have restored the new
     * thread's context, we actually return from this function to wherever
     * the new thread was when it was scheduled out. This could be either a
     * regular C routine if the new thread previously scheduled itself out
     * cooperatively, or it could be an ISR if this new thread was
     * previously preempted (on exiting the ISR, execution will return to
     * wherever the new thread was originally interrupted).
     */

    /**
     * Note that we always just perform a RET here. Although we may
     * come in from an ISR and leave through a regular C routine for
     * another thread (and visa versa) this is OK, because we don't
     * actually need to perform a RETI to tell the processor the
     * interrupt is finished. The only extra thing that RETI does is to
     * set the I bit (interrupts enabled). If we enter from a regular
     * thread context, but leave through an ISR return address and a RETI,
     * that just returns and handily enables interrupts for us. Similarly
     * if we enter from an ISR and leave back into some thread context
     * calls, interrupts will remain disabled through the regular RET
     * calls, and we will reenable interrupts in the CRITICAL_END() call
     * when we unlock interrupts.
     */

    ret


/**
 * \b archFirstThreadRestore
 *
 * Architecture-specific function to restore and start the first thread.
 * This is called by atomOSStart() when the OS is starting.
 *
 * This function will be largely similar to the latter half of
 * archContextSwitch(). Its job is to restore the context for the
 * first thread, and finally enable interrupts.
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
.global archFirstThreadRestore
archFirstThreadRestore:

    /**
     * Parameter locations:
     *  new_tcb_ptr = R25-R24
     */

    /**
     * First thread restores in the AVR port expect to see R2-R17 and
     * R28-R29 stored as context. The context will look exactly like it
     * would had a thread cooperatively scheduled itself out. That is,
     * these registers will be stored on the stack, and above those will
     * be the return address of the calling function. In this case we
     * will have set up this "fake" context in archThreadContextInit(),
     * and above these registers will be the return address of the thread
     * entry point. A "ret" or "reti" instruction will therefore direct
     * the processor to the thread entry point, by popping this "return
     * address" off the stack.
     */

    /**
     * Get the new thread's stack pointer off the TCB (new_tcb_ptr).
     * new_tcb_ptr is stored in the parameter registers, R25-R24.
     * We are free to use any other registers, however, as we haven't
     * yet popped any of the new thread's context off its stack.
     */
    mov r28,r24         /* Move new_tcb_ptr into the Y-regs so we */
    mov r29,r25         /* can access the TCB via a pointer. */

    ld r16,Y            /* Load new_tcb_ptr->sp_save_ptr into R16/R17. */
    ldd r17,Y+1         /* It is conveniently the first member of the TCB. */

    out _SFR_IO_ADDR(SPL),r16  /* Set our stack pointer to the new thread's */
    out _SFR_IO_ADDR(SPH),r17  /* stack pointer, from its TCB. */

    /**
     * On devices with large program space we also restore RAMPZ, EIND.
     */
#ifdef __AVR_3_BYTE_PC__
    pop r0
    in r0,_SFR_IO_ADDR(EIND)
    pop r0
    in r0,_SFR_IO_ADDR(RAMPZ)
#endif

    /**
     * Restore registers R2-R17, R28-R29.
     */
    pop r29
    pop r28
    pop r17
    pop r16
    pop r15
    pop r14
    pop r13
    pop r12
    pop r11
    pop r10
    pop r9
    pop r8
    pop r7
    pop r6
    pop r5
    pop r4
    pop r3
    pop r2

    /**
     * The "return address" left on the stack now will be the new
     * thread's entry point. RETI will take us there as if we had
     * actually been there before calling this subroutine, whereas
     * the return address was actually set up by archThreadContextInit().
     *
     * As discussed above, this function is responsible for enabling
     * interrupts once all context has been restored. We can do this
     * using a single RETI instruction (return and enable interrupts),
     * but it is also safe at this point to have two separate
     * instructions:
     *   sei    // enable interrupts
     *   ret    // return to new thread entry point
     */
    reti


