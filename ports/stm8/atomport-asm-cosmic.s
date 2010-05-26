;
; Copyright (c) 2010, Atomthreads Project. All rights reserved.
;
; Redistribution and use in source and binary forms, with or without
; Modification, are permitted provided that the following conditions
; are met:
;
; 1. Redistributions of source code must retain the above copyright
;	   notice, this list of conditions and the following disclaimer.
; 2. Redistributions in binary form must reproduce the above copyright
;    notice, this list of conditions and the following disclaimer in the
;    documentation and/or other materials provided with the distribution.
; 3. No personal names or organizations' names associated with the
;    Atomthreads project may be used to endorse or promote products
;    derived from this software without specific prior written permission.
;
; THIS SOFTWARE IS PROVIDED BY THE ATOMTHREADS PROJECT AND CONTRIBUTORS
; "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
; TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
; PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE PROJECT OR CONTRIBUTORS BE
; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
; POSSIBILITY OF SUCH DAMAGE.
;


; Cosmic assembler routines


;  Export functions to other modules
xdef _archContextSwitch, _archFirstThreadRestore


;  \b archContextSwitch
;
;  Architecture-specific context switch routine.
;
;  Note that interrupts are always locked out when this routine is
;  called. For cooperative switches, the scheduler will have entered
;  a critical region. For preemptions (called from an ISR), the
;  ISR will have disabled interrupts on entry.
;
;  @param[in] old_tcb_ptr Pointer to the thread being scheduled out
;  @param[in] new_tcb_ptr Pointer to the thread being scheduled in
;
;  @return None
;
;  void archContextSwitch (ATOM_TCB *old_tcb_ptr, ATOM_TCB *new_tcb_ptr)
_archContextSwitch:

    ; Parameter locations (Cosmic calling convention):
    ;   old_tcb_ptr = X register (word-width)
    ;   new_tcb_ptr = stack (word-width)

    ; STM8 CPU Registers:
    ;
    ; A, X, Y: Standard working registers
    ; SP: Stack pointer
    ; PC: Program counter
    ; CC: Code condition register
    ;
    ; Cosmic compiler virtual registers:
    ;
    ; c_x, c_y: Scratch memory areas saved by ISRs
    ; c_lreg: Scratch memory area only saved by ISRs with @svlreg
    ;
    ; If this is a cooperative context switch (a thread has called us
    ; to schedule itself out), the Cosmic compiler will have saved any
    ; of the registers which it does not want us to clobber. There are
    ; no registers which are expected to retain their value across a
    ; function call, hence for cooperative context switches with this
    ; compiler we do not actually need to save any registers at all.
    ;
    ; If we were called from an interrupt routine (because a thread
    ; is being preemptively scheduled out), the situation is exactly
    ; the same. Any ISR which calls out to a subroutine will have
    ; similarly saved all registers which it needs us not to clobber
    ; which in the case of this compiler is all registers. Again, we
    ; do not need to save any registers because no registers are
	; expected to be unclobbered by a subroutine. Note that it is
	; necessary to add the @svlreg modifier to ISRs which call out to
	; the OS in order to force a save of c_lreg. The rest of the CPU
	; registers and the c_x and c_y virtual registers are, however,
    ; always saved by ISRs which call out to C subroutines.
    ;
    ; This is an unusual context switch routine, because it does not
	; need to actually save any registers. Instead, the act of
	; calling this function causes all registers which must not be
	; clobbered to be saved on the stack anyway in the case of
	; cooperative context switches. For preemptive switches, the
	; interrupt service routine which calls out to here also causes
	; all registers to be saved in a similar fashion.

    ; We do have to do some work in here though: we need to store
    ; the current stack pointer to the current thread's TCB, and
    ; switch in the new thread by taking the stack pointer from
    ; the new thread's TCB and making that our new stack pointer.

    ; The parameter pointing to the the old TCB (a word-width
    ;	pointer) is still untouched in the X register.

    ; Store current stack pointer as first entry in old_tcb_ptr
    ldw Y, SP    ; Move current stack pointer into Y register
    ldw (X), Y   ; Store current stack pointer at first offset in TCB


    ; At this point, all of the current thread's context has been saved
    ; so we no longer care about keeping the contents of any registers.
    ; We do still need the first two bytes on the current thread's stack,
    ; however, which contain new_tcb_ptr (a pointer to the TCB of the
    ; thread which we wish to switch in).
    ;
    ; Our stack frame now contains all registers (if this is a preemptive
    ; switch due to an interrupt handler) or those registers which the
    ; calling function did not wish to be clobbered (if this is a
    ; cooperative context switch). It also contains the return address
    ; which will be either a function called via an ISR (for preemptive
    ; switches) or a function called from thread context (for cooperative
    ; switches). Finally, the stack also contains the aforementioned
    ; word which is the new_tcb_ptr parameter passed via the stack.
    ;
    ; In addition, the thread's stack pointer (after context-save) is
    ; stored in the thread's TCB.

    ; We are now ready to restore the new thread's context. In most
    ; architecture ports we would typically switch our stack pointer
    ; to the new thread's stack pointer, and pop all of its context
    ; off the stack, before returning to the caller (the original
    ; caller when the new thread was last scheduled out). In this
    ; port, however, we do not need to actually restore any
    ; registers here because none are saved when we switch out (at
    ; least not by this function). We switch to the new thread's
    ; stack pointer and then return to the original caller, which
    ; will restore any registers which had to be saved.

    ; Get the new thread's stack pointer off the TCB (new_tcb_ptr).
    ; new_tcb_ptr is still stored in the previous thread's stack.
    ; We are free to use any registers here.

    ; Pull the new_tcb_ptr parameter from the stack into X register
    ldw X,($3,SP)

    ; Pull the first entry out of new_tcb_ptr (the new thread's
    ; stack pointer) into X register.
    ldw X,(X)

    ; Switch our current stack pointer to that of the new thread.
    ldw SP,X

    ; Normally we would start restoring registers from the new
    ; thread's stack here, but we don't save/restore any. We're
    ; almost done.

    ; The return address on the stack will now be the new thread's return
    ; address - i.e. although we just entered this function from a
    ; function called by the old thread, now that we have restored the new
    ; thread's stack, we actually return from this function to wherever
    ; the new thread was when it was previously scheduled out. This could
    ; be either a regular C routine if the new thread previously scheduled
    ; itself out cooperatively, or it could be an ISR if this new thread was
    ; previously preempted (on exiting the ISR, execution will return to
    ; wherever the new thread was originally interrupted).

    ; Return to the caller. Note that we always use a regular RET here
    ; because this is a subroutine regardless of whether we were called
    ; during an ISR or by a thread cooperatively switching out. The
    ; difference between RET and IRET on the STM8 architecture is that
    ; RET only pops the return address off the stack, while IRET also
    ; pops from the stack all of the CPU registers saved when the ISR
    ; started, including restoring the interrupt-enable bits from the CC
    ; register.
    ;
    ; It is important that whenever we call this function (whether from
    ; an ISR for preemptive switches or from thread context for
    ; cooperative switches) interrupts are always disabled. This means
    ; that whichever method by which we leave this routine we always
    ; have to reenable interrupts, so we can use the same context-switch
    ; routine for preemptive and cooperative switches.
    ;
    ; The possible call/return paths are as follows:
    ;
    ; Scenario 1 (cooperative -> cooperative):
    ;  Thread A: cooperatively switches out
    ;    * Thread A relinquishes control / cooperatively switches out
    ;    * Interrupts already disabled by kernel for cooperative reschedules
    ;    * Partial register context saved by calling function
    ;    * Call here at thread context
    ;    * Switch to Thread B
    ;  Thread B (was previously cooperatively switched out):
    ;    * Stack context for Thread B contains its return address
    ;    * Return to function which was called at thread context
    ;    * Interrupts are reenabled by CRITICAL_END() call in kernel
    ;    * Return to Thread B application code
    ;
    ; Scenario 2 (preemptive -> preemptive):
    ;  Thread A: preemptively switches out
    ;    * ISR occurs
    ;    * Interrupts disabled by CPU at ISR entry (assume no nesting allowed)
    ;    * Full register context saved by CPU at ISR entry
    ;    * Call here at ISR context
    ;    * Switch to Thread B
    ;  Thread B (was previously preemptively switched out):
    ;    * Stack context for Thread B contains its return address
    ;      and all context saved by the CPU on ISR entry
    ;    * Return to function which was called at ISR context
    ;    * Eventually returns to calling ISR which calls IRET
    ;    * IRET performs full register context restore
    ;    * IRET reenables interrupts
    ;    * Return to Thread B application code
    ;
    ; Scenario 3 (cooperative -> preemptive):
    ;  Thread A: cooperatively switches out
    ;    * Thread A relinquishes control / cooperatively switches out
    ;    * Interrupts already disabled by kernel for cooperative reschedules
    ;    * Partial register context saved by calling function
    ;    * Call here at thread context
    ;    * Switch to Thread B
    ;  Thread B (was previously preemptively switched out):
    ;    * Stack context for Thread B contains its return address
    ;      and all context saved by the CPU on ISR entry
    ;    * Return to function which was called at ISR context
    ;    * Eventually returns to calling ISR which calls IRET
    ;    * IRET performs full register context restore
    ;    * IRET reenables interrupts
    ;    * Return to Thread B application code
    ;
    ; Scenario 4 (preemptive -> cooperative):
    ;  Thread A: preemptively switches out
    ;    * ISR occurs
    ;    * Interrupts disabled by CPU at ISR entry (assume no nesting allowed)
    ;    * Full register context saved by CPU at ISR entry
    ;    * Call here at ISR context
    ;    * Switch to Thread B
    ;  Thread B (was previously cooperatively switched out):
    ;    * Stack context for Thread B contains its return address
    ;    * Return to function which was called at thread context
    ;    * Interrupts are reenabled by CRITICAL_END() call in kernel
    ;    * Return to Thread B application code
    ;
    ; The above shows that it does not matter whether we are rescheduling
    ; from/to thread context or ISR context. It is perfectly valid to
    ; enter here at ISR context but leave via a thread which previously
    ; cooperatively switched out because:
    ; 1. Although the CPU handles ISRs differently by automatically
    ;    stacking all 6 CPU registers, and restoring them on an IRET,
    ;    we handle this because we switch the stack pointer to a
    ;    different thread's stack. Because the stack pointer is
    ;    switched, it does not matter that on entry via ISRs more
    ;    registers are saved on the original thread's stack than entries
    ;    via non-ISRs. Those extra registers will be restored properly
    ;    by an IRET when the thread is eventually scheduled back in
    ;    (which could be a long way off). This assumes that the CPU does
    ;    not have hidden behaviour that occurs on interrupts, and we can
    ;    in fact trick it into leaving via another thread's call stack,
    ;     and performing the IRET much later.
    ; 2. Although the CPU handles ISRs differently by setting the CC
    ;    register interrupt-enable bits on entry/exit, we handle this
    ;    anyway by always assuming interrupts are disabled on entry
    ;    and exit regardless of the call path.

    ; Return from subroutine
    ret


; \b archFirstThreadRestore
;
; Architecture-specific function to restore and start the first thread.
; This is called by atomOSStart() when the OS is starting. Its job is to
; restore the context for the first thread and start running at its
; entry point.
;
; All new threads have a stack context pre-initialised with suitable
; data for being restored by either this function or the normal
; function used for scheduling threads in, archContextSwitch(). Only
; the first thread run by the system is launched via this function,
; after which all other new threads will first be run by
; archContextSwitch().
;
; Typically ports will implement something similar here to the
; latter half of archContextSwitch(). In this port the context
; switch does not restore many registers, and instead relies on the
; fact that returning from any function which called
; archContextSwitch() will restore any of the necessary registers.
; For new threads which have never been run there is no calling
; function which will restore context on return, therefore we
; do not restore many register values here. It is not necessary
; for the new threads to have initialised values for the scratch
; registers A, X and Y or the code condition register CC which
; leaves SP and PC. SP is restored because this is always needed to
; switch to a new thread's stack context. It is not necessary to
; restore PC, because the thread's entry point is in the stack
; context (when this function returns using RET the PC is
; automatically changed to the thread's entry point because the
; entry point is stored in the preinitialised stack).
;
; When new threads are started interrupts must be enabled, so there
; is some scope for enabling interrupts in the CC here. It must be
; done for all new threads, however, not just the first thread, so
; we use a different system. We instead use a thread shell routine
; which all functions run when they are first started, and
; interrupts are enabled in there. This allows us to avoid having
; to enable interrupts both in here and in the normal context
; switch routine (archContextSwitch()). For the normal context
; switch routine we would otherwise need to pass in notification of
; and implement special handling for the first time a thread is
; restored.
;
; In summary, first threads do not require a set of CPU registers
; to be initialised to known values, so we only set SP to the new
; thread's stack pointer. PC is restored for free because the RET
; call at the end of this function pops the return address off the
; stack.
;
; Note that you can create more than one thread before starting
; the OS - only one thread is restored using this function, so
; all other threads are actually restored by archContextSwitch().
; This is another reminder that the initial context set up by
; archThreadContextInit() must look the same whether restored by
; archFirstThreadRestore() or archContextSwitch().
;
; @param[in] new_tcb_ptr Pointer to the thread being scheduled in
;
; @return None
;
; void archFirstThreadRestore (ATOM_TCB *new_tcb_ptr)
_archFirstThreadRestore:

    ; Parameter locations:
    ;  new_tcb_ptr = X register (word-width)

    ; As described above, first thread restores in this port do not
    ; expect any initial register context to be pre-initialised in
    ; the thread's stack area. The thread's initial stack need only
    ; contain the thread's initial entry point, and we do not even
    ; "restore" that within this function. We leave the thread's entry
    ; point in the stack, and RET at the end of the function pops it
    ; off and "returns" to the entry point as if we were called from
    ; there.
    ;
    ; The one thing we do need to set in here, though, is the thread's
    ; stack pointer. This is available from the passed thread TCB
    ; structure.

    ; Get the new thread's stack pointer off the TCB (new_tcb_ptr).
    ; new_tcb_ptr is stored in the parameter register X. The stack
    ; pointer it conveniently located at the top of the TCB so no
    ; indexing is required to pull it out.
    ldw X,(X)

    ; Switch our current stack pointer to that of the new thread.
    ldw SP,X

    ; The "return address" left on the stack now will be the new
    ; thread's entry point. RET will take us there as if we had
    ; actually been there before calling this subroutine, whereas
    ; the return address was actually set up by archThreadContextInit().
    ret


    end
