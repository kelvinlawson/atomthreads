.section .vectors, "x"

.global __interrupt_vector_table
.global __irq_stack_top__
.global __fiq_stack_top__
.global __svc_stack_top__

.global bsp_ints_enable
.global bsp_ints_disable
.global bsp_ints_restore


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
  B . /* Undefined */
  B . /* SWI */
  B . /* Prefetch Abort */
  B . /* Data Abort */
  B . /* reserved */
  B IRQ_Handler /* IRQ */
  B . /* FIQ */

 

Reset_Handler:


	MSR CPSR_c,#(IRQ_MODE | I_BIT | F_BIT)
	LDR sp,=__irq_stack_top__ /* set the IRQ stack pointer */
	MSR CPSR_c,#(FIQ_MODE | I_BIT | F_BIT)
	LDR sp,=__fiq_stack_top__ /* set the FIQ stack pointer */
	MSR CPSR_c,#(SVC_MODE | I_BIT | F_BIT)
	LDR sp,=__svc_stack_top__ /* set the SVC stack pointer */

	BL low_level_init
	BL _mainCRTStartup


  B .

IRQ_Handler:
	B archIRQHandler
	
