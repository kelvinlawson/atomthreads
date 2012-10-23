/**************************************************************************//**
 * @file     startup.c
 * @brief    
 * @version  
 * @date     
 *
 * @note
 * Copyright (C) 2010 NXP Semiconductors(NXP). All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * products. This software is supplied "AS IS" without any warranties.
 * NXP Semiconductors assumes no responsibility or liability for the
 * use of the software, conveys no license or title under any patent,
 * copyright, or mask work right to the product. NXP Semiconductors
 * reserves the right to make changes in the software without
 * notification. NXP Semiconductors also make no representation or
 * warranty that such application will be suitable for the specified
 * use without further testing or modification.
 ******************************************************************************/
#define WEAK __attribute__ ((weak))
#define ALIAS(f) __attribute__ ((weak, alias (#f)))


#include "system_LPC17xx.h"
#include "atomport_private.h"




     void ResetISR(void);
WEAK void NMI_Handler(void);
WEAK void HardFault_Handler(void);
WEAK void MemManage_Handler(void);
WEAK void BusFault_Handler(void);
WEAK void UsageFault_Handler(void);
WEAK void SVC_Handler(void);
WEAK void DebugMon_Handler(void);
WEAK void PendSV_Handler(void);
WEAK void SysTick_Handler(void);
WEAK void IntDefault_Handler(void);


//*****************************************************************************
//
// The entry point for the application.
// __main() is the entry point for Redlib based applications
// main() is the entry point for Newlib based applications
//
//*****************************************************************************

extern int main(void);
//*****************************************************************************
//
// External declaration for the pointer to the stack top from the Linker Script
//
//*****************************************************************************
extern void _vStackTop(void);


//*****************************************************************************
//
// The vector table.
// This relies on the linker script to place at correct location in memory.
//
//*****************************************************************************
extern void (* const g_pfnVectors[])(void);
__attribute__ ((section(".isr_vector")))
void (* const g_pfnVectors[])(void) = {
	// Core Level - CM3
	&_vStackTop, // The initial stack pointer
	ResetISR,								// The reset handler
	NMI_Handler,							// The NMI handler
	HardFault_Handler,						// The hard fault handler
	MemManage_Handler,						// The MPU fault handler
	BusFault_Handler,						// The bus fault handler
	UsageFault_Handler,						// The usage fault handler
	0,										// Reserved
	0,										// Reserved
	0,										// Reserved
	0,										// Reserved
	SVC_Handler,							// SVCall handler
	DebugMon_Handler,						// Debug monitor handler
	0,										// Reserved
	archPendSVHandler,						// The PendSV handler
	archTickHandler /*SysTick_Handler*/,						// The SysTick handler

	// Chip Level - LPC17
	IntDefault_Handler,					// 16, 0x40 - WDT
	IntDefault_Handler,					// 17, 0x44 - TIMER0
	IntDefault_Handler,					// 18, 0x48 - TIMER1
	IntDefault_Handler,					// 19, 0x4c - TIMER2
	IntDefault_Handler,					// 20, 0x50 - TIMER3
	IntDefault_Handler,					// 21, 0x54 - UART0
	IntDefault_Handler,					// 22, 0x58 - UART1
	IntDefault_Handler,					// 23, 0x5c - UART2
	IntDefault_Handler,					// 24, 0x60 - UART3
	IntDefault_Handler,					// 25, 0x64 - PWM1
	IntDefault_Handler,					// 26, 0x68 - I2C0
	IntDefault_Handler,					// 27, 0x6c - I2C1
	IntDefault_Handler,					// 28, 0x70 - I2C2
	IntDefault_Handler,					// 29, 0x74 - SPI
	IntDefault_Handler,					// 30, 0x78 - SSP0
	IntDefault_Handler,					// 31, 0x7c - SSP1
	IntDefault_Handler,					// 32, 0x80 - PLL0 (Main PLL)
	IntDefault_Handler,					// 33, 0x84 - RTC
	IntDefault_Handler,					// 34, 0x88 - EINT0
	IntDefault_Handler,					// 35, 0x8c - EINT1
	IntDefault_Handler,					// 36, 0x90 - EINT2
	IntDefault_Handler,					// 37, 0x94 - EINT3
	IntDefault_Handler,					// 38, 0x98 - ADC
	IntDefault_Handler,					// 39, 0x9c - BOD
	IntDefault_Handler,					// 40, 0xA0 - USB
	IntDefault_Handler,					// 41, 0xa4 - CAN
	IntDefault_Handler,					// 42, 0xa8 - GP DMA
	IntDefault_Handler,					// 43, 0xac - I2S
	IntDefault_Handler,					// 44, 0xb0 - Ethernet
	IntDefault_Handler,					// 45, 0xb4 - RITINT
	IntDefault_Handler,					// 46, 0xb8 - Motor Control PWM
	IntDefault_Handler,					// 47, 0xbc - Quadrature Encoder
	IntDefault_Handler,					// 48, 0xc0 - PLL1 (USB PLL)
	IntDefault_Handler,					// 49, 0xc4 - USB Activity interrupt to wakeup
	IntDefault_Handler, 				// 50, 0xc8 - CAN Activity interrupt to wakeup
};

//*****************************************************************************
// Functions to carry out the initialization of RW and BSS data sections. These
// are written as separate functions rather than being inlined within the
// ResetISR() function in order to cope with MCUs with multiple banks of
// memory.
//*****************************************************************************
__attribute__ ((section(".after_vectors")))
void data_init(unsigned int romstart, unsigned int start, unsigned int len) {
	unsigned int *pulDest = (unsigned int*) start;
	unsigned int *pulSrc = (unsigned int*) romstart;
	unsigned int loop;
	for (loop = 0; loop < len; loop = loop + 4)
		*pulDest++ = *pulSrc++;
}

__attribute__ ((section(".after_vectors")))
void bss_init(unsigned int start, unsigned int len) {
	unsigned int *pulDest = (unsigned int*) start;
	unsigned int loop;
	for (loop = 0; loop < len; loop = loop + 4)
		*pulDest++ = 0;
}


//*****************************************************************************
// The following symbols are constructs generated by the linker, indicating
// the location of various points in the "Global Section Table". This table is
// created by the linker via the Code Red managed linker script mechanism. It
// contains the load address, execution address and length of each RW data
// section and the execution and length of each BSS (zero initialized) section.
//*****************************************************************************
extern unsigned int __data_section_table;
extern unsigned int __data_section_table_end;
extern unsigned int __bss_section_table;
extern unsigned int __bss_section_table_end;


//*****************************************************************************
// Reset entry point for your code.
// Sets up a simple runtime environment and initializes the C/C++
// library.
//*****************************************************************************
__attribute__ ((section(".after_vectors")))
void
ResetISR(void) {

    //
    // Copy the data sections from flash to SRAM.
    //
	unsigned int LoadAddr, ExeAddr, SectionLen;
	unsigned int *SectionTableAddr;

	// Load base address of Global Section Table
	SectionTableAddr = &__data_section_table;

    // Copy the data sections from flash to SRAM.
	while (SectionTableAddr < &__data_section_table_end) {
		LoadAddr = *SectionTableAddr++;
		ExeAddr = *SectionTableAddr++;
		SectionLen = *SectionTableAddr++;
		data_init(LoadAddr, ExeAddr, SectionLen);
	}
	// At this point, SectionTableAddr = &__bss_section_table;
	// Zero fill the bss segment
	while (SectionTableAddr < &__bss_section_table_end) {
		ExeAddr = *SectionTableAddr++;
		SectionLen = *SectionTableAddr++;
		bss_init(ExeAddr, SectionLen);
	}



	low_level_init();




	main();


	//
	// main() shouldn't return, but if it does, we'll just enter an infinite loop
	//
	while (1) ;	
}

//*****************************************************************************
// Default exception handlers. Override the ones here by defining your own
// handler routines in your application code.
//*****************************************************************************
__attribute__ ((section(".after_vectors")))  __attribute__( ( naked ) )
void NMI_Handler(void)
{
    while(1) ;

}

__attribute__ ((section(".after_vectors")))  __attribute__( ( naked ) )
void HardFault_Handler(void)
{

    __asm volatile
    (
        " tst lr, #4                    \n"
        " ite eq                        \n"
        " mrseq r0, msp                 \n"
        " mrsne r0, psp                 \n"
        " b dbg_fault_handler    \n"
    );

    while(1) ;

}

__attribute__ ((section(".after_vectors")))  __attribute__( ( naked ) )
void MemManage_Handler(void)  
{
    while(1) ;

}

__attribute__ ((section(".after_vectors")))  __attribute__( ( naked ) )
void BusFault_Handler(void)  
{
    while(1) ;

}

__attribute__ ((section(".after_vectors")))  __attribute__( ( naked ) )
void UsageFault_Handler(void)
{
    while(1) ;

}

__attribute__ ((section(".after_vectors")))  __attribute__( ( naked ) )
void SVC_Handler(void)
{
    while(1) ;

}

__attribute__ ((section(".after_vectors")))  __attribute__( ( naked ) )
void DebugMon_Handler(void)
{
    while(1) ;

}

__attribute__ ((section(".after_vectors")))  __attribute__( ( naked ) )
void PendSV_Handler(void)
{
    while(1) ;

}

__attribute__ ((section(".after_vectors")))  __attribute__( ( naked ) )
void SysTick_Handler(void)
{
    while(1) ;

}

__attribute__ ((section(".after_vectors")))  __attribute__( ( naked ) )
void IntDefault_Handler(void)
{
    while(1) ;

}
