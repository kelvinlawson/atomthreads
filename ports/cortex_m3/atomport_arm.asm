;
; Copyright (c) 2012, Natie van Rooyen. All rights reserved.
;
; Redistribution and use in source and binary forms, with or without
; modification, are permitted provided that the following conditions
; are met:
;
; 1. Redistributions of source code must retain the above copyright
;    notice, this list of conditions and the following disclaimer.
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


    PRESERVE8    {TRUE}
    AREA    UTILS,    CODE,    READONLY
;--
        EXPORT    contextInit
        EXPORT    contextSwitch
        EXPORT    contextStart
        EXPORT    contextEnableInterrupts
        EXPORT    contextEnterCritical
        EXPORT    contextExitCritical
        EXPORT    pendSV_Handler
        EXPORT    tick_Handler

        EXTERN    archTickHandler

;--
NVIC_INT_CTRL                     EQU            0xE000ED04        ;    Interrupt control state register
NVIC_PENDSVSET                    EQU            0x10000000        ;    Value to trigger PendSV exception
NVIC_PR_12_15_ADDR                EQU            0xE000ED20        ;    System Handlers 12-15 Priority Register Address
NVIC_PENDS_VPRIORITY              EQU            0x00FF0000        ;    PendSV priority is minimal (0xFF)

;--
;  \b contextInit
;
;  Architecture-specific one time initialization.
;
;  Configures PendSV priority to lowest.
;
;  @return None
;
contextInit
        LDR            r1, =NVIC_PR_12_15_ADDR        ;-- Load the System 12-15 Priority Register
        LDR            r0, [r1]
        ORR            r0, r0, #NVIC_PENDS_VPRIORITY  ;-- set PRI_14 (PendSV) to 0xFF - minimal
        STR            r0, [r1]

        BX             lr

;--
;  \b contextSwitch
;
;  Architecture-specific context switch routine.
;
;  Note that interrupts are always locked out when this routine is
;  called. For cooperative switches, the scheduler will have entered
;  a critical region. For preemptions (called from an ISR), the
;  interrupts will have disabled in the tick_Handler.
;
;  @param[in] [r0] -> Address to save old stack pointer
;  @param[in] [r1] -> Address where new stack pointer is stored
;
;  @return None
;
contextSwitch
        LDR            r2, =context_new_stack_ptr
        STR            r1, [r2]

        LDR            r2, =context_save_stack_ptr
        LDR            r1, [r2]
        TEQ            r1, #0                ;    if contextSwitch is going to be called again before pend_sv
        STREQ          r0, [r2]

        LDR            R0, =NVIC_INT_CTRL    ;    Trigger the PendSV exception (causes context switch)
        LDR            R1, =NVIC_PENDSVSET
        STR            R1, [R0]

        BX             lr

;--
;  \b contextStart
;
;  Architecture-specific context start routine.
;
;  @param[in] [r0] -> Address where stack pointer is stored
;
;  @return Does not return
;
contextStart
        LDR            r1, =context_new_stack_ptr
        STR            r0, [r1]
        LDR            r1, =context_save_stack_ptr
        MOV            r0, #0
        STR            r0, [r1]
        LDR            r0, =NVIC_INT_CTRL    ;    Trigger the PendSV exception (causes context switch)
        LDR            r1, =NVIC_PENDSVSET
        STR            r1, [r0]

        BX             lr

;--
;  \b contextEnableInterrupts
;
;  Enables interrupts on the processor
;
;  @return None
;
contextEnableInterrupts
        CPSIE          i
        BX             lr


;--
;  \b contextExitCritical
;
;  Exit critical section (restores interrupt posture)
;
;  @param[in] r0 Interrupt Posture
;
;  @return None
;
contextExitCritical
        MSR            PRIMASK, r0
        BX             lr


;--
;  \b contextEnterCritical
;
;  Enter critical section (disables interrupts)
;
;  @return Current interrupt posture
;
contextEnterCritical
        MRS            r0, PRIMASK
        CPSID          i
        BX             lr

;--
;  \b PendSV_Handler
;
;  CortexM3 PendSV_Handler. Switch context to a new stack.
;
;  @return None
;
pendSV_Handler
        CPSID          i                   ;    Disable core int

        LDR            r1, =context_save_stack_ptr
        LDR            r0, [r1]			   ;    Load old (current) stack pointer address

        LDR            r2, =context_new_stack_ptr
        LDR            r2, [r2]            ;    Load new stack pointer address
        TEQ            r0, r2
        BEQ            pendsv_handler_exit

        TEQ            r0, #0
        BEQ            pendsv_handler_new_stack
		                                   ;    Save context
        MRS            r3, PSP             ;    Get PSP point
        STMDB          r3!, {R4-R11}       ;    Store r4-r11
        STR            r3, [r0]            ;    Save old stack pointer
        MOV            r3, #0
        STR            r3, [r1]

pendsv_handler_new_stack
                                           ;    Restore context
        LDR            r2, [r2]            ;    Load new stack pointer
        LDMIA          r2!, {r4-r11}       ;    Restore context
        MSR            PSP, r2             ;    Mov new stack point to PSP

pendsv_handler_exit
        CPSIE         i                    ;    Enable core int

        ORR           lr, lr, #0x04        ;    Ensure exception return uses process stack
        BX            lr                   ;    Exit interrupt


;--
;  \b Tick_Handler
;
;  System timer tick interrupt handler.
;
;  @return None
;
tick_Handler
        PUSH {r4-r11, lr}
        cpsid  I                           ;   Disable core int
        BL  archTickHandler
        cpsie  I                           ;   Enable core int
        POP {r4-r11, pc}


;--
context_new_stack_ptr         DCD    0x00000000
context_save_stack_ptr        DCD    0x00000000

;--
    END
