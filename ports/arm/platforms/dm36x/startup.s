.section .vectors, "x"

.global __interrupt_vector_table
.extern __irq_stack_top__
.extern __fiq_stack_top__
.extern __svc_stack_top__

.extern __null_handler 


.equ USR_MODE,            0x10
.equ FIQ_MODE,            0x11
.equ IRQ_MODE,            0x12
.equ SVC_MODE,            0x13
.equ ABT_MODE,            0x17
.equ UND_MODE,            0x1B
.equ SYS_MODE,            0x1F

.equ I_BIT,               0x80            /* when I bit is set, IRQ is disabled */
.equ F_BIT,               0x40            /* when F bit is set, FIQ is disabled */


__interrupt_vector_table:

  B Reset_Handler /* Reset */
  ldr PC,=Exception_Handler  /* Undefined */
  ldr PC,=Exception_Handler  /* SWI */
  ldr PC,=Exception_Handler  /* Prefetch Abort */
  ldr PC,=Exception_Handler  /* Data Abort */
  ldr PC,=Exception_Handler  /* reserved */
  ldr PC,=archIRQHandler/* IRQ */
  ldr PC,=Exception_Handler  /* FIQ */
 

Reset_Handler:

    MSR CPSR_c,#(IRQ_MODE | I_BIT | F_BIT)
    LDR sp,=__irq_stack_top__ /* set the IRQ stack pointer */
    MSR CPSR_c,#(FIQ_MODE | I_BIT | F_BIT)
    LDR sp,=__fiq_stack_top__ /* set the FIQ stack pointer */
    MSR CPSR_c,#(SVC_MODE | I_BIT | F_BIT)
    LDR sp,=__svc_stack_top__ /* set the SVC stack pointer */

    BL low_level_init
    BL _mainCRTStartup

/**
 *  \b Exception_Handler
 *
 *  IRQ entry point.
 *
 *  Save the process/thread context onto its own stack before calling __interrupt_dispatcher().
 *  __interrupt_dispatcher() might switch stacks. On return the same context is popped from the 
 *  stack and control is returned to the process.
 *
 *  @return None
 */
Exception_Handler:

    MSR         cpsr_c, #(SVC_MODE | I_BIT)     /* Save current process context in process stack */
    STMFD       sp!, {r0 - r3, ip, lr}          
                                                
    MSR         cpsr_c, #(IRQ_MODE | I_BIT)     /* Save lr_irq and spsr_irq in process stack */
    SUB         lr, lr, #4                      
    MOV         r1, lr                          
    MRS         r2, spsr                        
    MSR         cpsr_c, #(SVC_MODE | I_BIT)     
    STMFD       sp!, {r1, r2}                   
                                                
    BL          __null_handler          /* Dispatch the interrupt to platform folder for
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

  B .

