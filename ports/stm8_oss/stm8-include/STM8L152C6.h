/*-------------------------------------------------------------------------

  STM8L152C6.h - Device Declarations

  STM8L/STM8AL, medium density with ROM bootloader

  Copyright (C) 2020, Georg Icking-Konert

  Ultra-low-power 8-bit MCU with 32 Kbytes Flash, 16 MHz CPU, integrated EEPROM 

  datasheet: https://www.st.com/resource/en/datasheet/stm8l152c6.pdf
  reference: RM0031 https://www.st.com/content/ccc/resource/technical/document/reference_manual/2e/3b/8c/8f/60/af/4b/2c/CD00218714.pdf/files/CD00218714.pdf/jcr:content/translations/en.CD00218714.pdf

  MIT License

  Copyright (c) 2020 Georg Icking-Konert

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.

-------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------
  MODULE DEFINITION FOR MULTIPLE INCLUSION
-------------------------------------------------------------------------*/
#ifndef STM8L152C6_H
#define STM8L152C6_H

// DEVICE NAME
#define DEVICE_STM8L152C6

// DEVICE FAMILY
#define FAMILY_STM8L

// required for C++
#ifdef __cplusplus
  extern "C" {
#endif


/*-------------------------------------------------------------------------
  INCLUDE FILES
-------------------------------------------------------------------------*/
#include <stdint.h>


/*-------------------------------------------------------------------------
  COMPILER SPECIFIC SETTINGS
-------------------------------------------------------------------------*/

// Cosmic compiler
#if defined(__CSMC__)

  // macros to unify ISR declaration and implementation
  #define ISR_HANDLER(func,irq)  @far @interrupt void func(void)      ///< handler for interrupt service routine
  #define ISR_HANDLER_TRAP(func) void @far @interrupt func(void)      ///< handler for trap service routine

  // definition of inline functions
  #define INLINE                 @inline                              ///< keyword for inline functions

  // common assembler instructions
  #define NOP()                  _asm("nop")                          ///< perform a nop() operation (=minimum delay)
  #define DISABLE_INTERRUPTS()   _asm("sim")                          ///< disable interrupt handling
  #define ENABLE_INTERRUPTS()    _asm("rim")                          ///< enable interrupt handling
  #define TRIGGER_TRAP           _asm("trap")                         ///< trigger a trap (=soft interrupt) e.g. for EMC robustness (see AN1015)
  #define WAIT_FOR_INTERRUPT()   _asm("wfi")                          ///< stop code execution and wait for interrupt
  #define ENTER_HALT()           _asm("halt")                         ///< put controller to HALT mode
  #define SW_RESET()             _asm("dc.b $75")                     ///< reset via illegal opcode (works for all devices)

  // data type in bit fields
  #define BITS                   unsigned int                         ///< data type in bit structs (follow C90 standard)


// IAR Compiler
#elif defined(__ICCSTM8__)

  // include intrinsic functions
  #include <intrinsics.h>

  // macros to unify ISR declaration and implementation
  #define STRINGVECTOR(x) #x
  #define VECTOR_ID(x) STRINGVECTOR( vector = (x) )
  #define ISR_HANDLER( a, b )  \
    _Pragma( VECTOR_ID( (b)+2 ) )        \
    __interrupt void (a)( void )
  #define ISR_HANDLER_TRAP(a) \
    _Pragma( VECTOR_ID( 1 ) ) \
    __interrupt void (a) (void)  

  // definition of inline functions
  #define INLINE                 static inline                        ///< keyword for inline functions

  // common assembler instructions
  #define NOP()                  __no_operation()                     ///< perform a nop() operation (=minimum delay)
  #define DISABLE_INTERRUPTS()   __disable_interrupt()                ///< disable interrupt handling
  #define ENABLE_INTERRUPTS()    __enable_interrupt()                 ///< enable interrupt handling
  #define TRIGGER_TRAP           __trap()                             ///< trigger a trap (=soft interrupt) e.g. for EMC robustness (see AN1015)
  #define WAIT_FOR_INTERRUPT()   __wait_for_interrupt()               ///< stop code execution and wait for interrupt
  #define ENTER_HALT()           __halt()                             ///< put controller to HALT mode
  #define SW_RESET()             __asm("dc8 0x75")                    ///< reset via illegal opcode (works for all devices)

  // data type in bit fields
  #define BITS                   unsigned char                        ///< data type in bit structs (deviating from C90 standard)


// SDCC compiler
#elif defined(__SDCC)

  // store SDCC version in preprocessor friendly way
  #define SDCC_VERSION (__SDCC_VERSION_MAJOR * 10000 \
                      + __SDCC_VERSION_MINOR * 100 \
                      + __SDCC_VERSION_PATCH)

  // unify ISR declaration and implementation
  #define ISR_HANDLER(func,irq)   void func(void) __interrupt(irq)    ///< handler for interrupt service routine
  #if SDCC_VERSION >= 30403  // traps require >=v3.4.3
    #define ISR_HANDLER_TRAP(func)  void func() __trap                ///< handler for trap service routine
  #else
    #error traps require SDCC >=3.4.3. Please update!
  #endif

  // definition of inline functions
  #define INLINE                 static inline                        ///< keyword for inline functions

  // common assembler instructions
  #define NOP()                  __asm__("nop")                       ///< perform a nop() operation (=minimum delay)
  #define DISABLE_INTERRUPTS()   __asm__("sim")                       ///< disable interrupt handling
  #define ENABLE_INTERRUPTS()    __asm__("rim")                       ///< enable interrupt handling
  #define TRIGGER_TRAP           __asm__("trap")                      ///< trigger a trap (=soft interrupt) e.g. for EMC robustness (see AN1015)
  #define WAIT_FOR_INTERRUPT()   __asm__("wfi")                       ///< stop code execution and wait for interrupt
  #define ENTER_HALT()           __asm__("halt")                      ///< put controller to HALT mode
  #define SW_RESET()             __asm__(".db 0x75")                  ///< reset via illegal opcode (works for all devices)

  // data type in bit fields
  #define BITS                   unsigned int                         ///< data type in bit structs (follow C90 standard)

// unsupported compiler -> stop
#else
  #error: compiler not supported
#endif


/*-------------------------------------------------------------------------
  FOR CONVENIENT PIN ACCESS
-------------------------------------------------------------------------*/

#define PIN0     0x01
#define PIN1     0x02
#define PIN2     0x04
#define PIN3     0x08
#define PIN4     0x10
#define PIN5     0x20
#define PIN6     0x40
#define PIN7     0x80


/*-------------------------------------------------------------------------
  DEVICE MEMORY (size in bytes)
-------------------------------------------------------------------------*/

// RAM
#define RAM_ADDR_START                0x000000
#define RAM_ADDR_END                  0x0007FF
#define RAM_SIZE                      2048


// EEPROM
#define EEPROM_ADDR_START             0x001000
#define EEPROM_ADDR_END               0x0013FF
#define EEPROM_SIZE                   1024


// OPTION
#define OPTION_ADDR_START             0x004800
#define OPTION_ADDR_END               0x0048FF
#define OPTION_SIZE                   256


// SFR1
#define SFR1_ADDR_START               0x005000
#define SFR1_ADDR_END                 0x0057FF
#define SFR1_SIZE                     2048


// BOOTROM
#define BOOTROM_ADDR_START            0x006000
#define BOOTROM_ADDR_END              0x0067FF
#define BOOTROM_SIZE                  2048


// SFR2
#define SFR2_ADDR_START               0x007F00
#define SFR2_ADDR_END                 0x007FFF
#define SFR2_SIZE                     256


// FLASH
#define FLASH_ADDR_START              0x008000
#define FLASH_ADDR_END                0x00FFFF
#define FLASH_SIZE                    32768


// MEMORY WIDTH (>32kB flash exceeds 16bit, as flash starts at 0x8000)
#define FLASH_ADDR_WIDTH            16                    ///< width of address space
#define FLASH_POINTER_T             uint16_t              ///< address variable type


/*-------------------------------------------------------------------------
  UNIQUE IDENTIFIER (size in bytes)
-------------------------------------------------------------------------*/

#define UID_ADDR_START                0x4926                ///< start address of unique identifier
#define UID_SIZE                      12                    ///< size of unique identifier [B]
#define UID(N)                        (*((uint8_t*) (UID_ADDR_START+N)))    ///< read unique identifier byte N


/*-------------------------------------------------------------------------
  ISR Vector Table (SDCC, IAR)
  Notes:
    - IAR has an IRQ offset of +2 compared to datasheet and below numbers
    - Cosmic uses a separate, device specific file 'stm8_interrupt_vector.c'
    - different interrupt sources may share the same IRQ
-------------------------------------------------------------------------*/

// interrupt                                   IRQ
#define _TLI_VECTOR_                             0          
#define _FLASH_EOP_VECTOR_                       1          ///< FLASH_EOP interrupt vector: enable: FLASH_CR1.IE, pending: FLASH_IAPSR.EOP, priority: ITC_SPR1.VECT1SPR
#define _FLASH_WR_PG_DIS_VECTOR_                 1          ///< FLASH_WR_PG_DIS interrupt vector: enable: FLASH_CR1.IE, pending: FLASH_IAPSR.WR_PG_DIS, priority: ITC_SPR1.VECT1SPR
#define _DMA1_CH0_HT_VECTOR_                     2          ///< DMA1_CH0_HT interrupt vector: enable: DMA1_C0CR.HTIE, pending: DMA1_C0SPR.HTIF, priority: ITC_SPR1.VECT2SPR
#define _DMA1_CH0_TC_VECTOR_                     2          ///< DMA1_CH0_TC interrupt vector: enable: DMA1_C0CR.TCIE, pending: DMA1_C0SPR.TCIF, priority: ITC_SPR1.VECT2SPR
#define _DMA1_CH1_HT_VECTOR_                     2          ///< DMA1_CH1_HT interrupt vector: enable: DMA1_C1CR.HTIE, pending: DMA1_C1SPR.HTIF, priority: ITC_SPR1.VECT2SPR
#define _DMA1_CH1_TC_VECTOR_                     2          ///< DMA1_CH1_TC interrupt vector: enable: DMA1_C1CR.TCIE, pending: DMA1_C1SPR.TCIF, priority: ITC_SPR1.VECT2SPR
#define _DMA1_CH2_HT_VECTOR_                     3          ///< DMA1_CH2_HT interrupt vector: enable: DMA1_C2CR.HTIE, pending: DMA1_C2SPR.HTIF, priority: ITC_SPR1.VECT3SPR
#define _DMA1_CH2_TC_VECTOR_                     3          ///< DMA1_CH2_TC interrupt vector: enable: DMA1_C2CR.TCIE, pending: DMA1_C2SPR.TCIF, priority: ITC_SPR1.VECT3SPR
#define _DMA1_CH3_HT_VECTOR_                     3          ///< DMA1_CH3_HT interrupt vector: enable: DMA1_C3CR.HTIE, pending: DMA1_C3SPR.HTIF, priority: ITC_SPR1.VECT3SPR
#define _DMA1_CH3_TC_VECTOR_                     3          ///< DMA1_CH3_TC interrupt vector: enable: DMA1_C3CR.TCIE, pending: DMA1_C3SPR.TCIF, priority: ITC_SPR1.VECT3SPR
#define _RTC_ALARM_VECTOR_                       4          ///< RTC_ALARM interrupt vector: enable: RTC_CR2.ALRAIE, pending: RTC_ISR2.ALRAF, priority: ITC_SPR2.VECT4SPR
#define _RTC_WAKEUP_VECTOR_                      4          ///< RTC_WAKEUP interrupt vector: enable: RTC_CR2.WUTIE, pending: RTC_ISR2.WUTF, priority: ITC_SPR2.VECT4SPR
#define _EXTIE_VECTOR_                           5          ///< EXTIE interrupt vector: enable: EXTI_CR3.PEIS, pending: EXTI_SR2.PEF, priority: ITC_SPR2.VECT5SPR
#define _EXTIF_VECTOR_                           5          ///< EXTIF interrupt vector: enable: EXTI_CR3.PFIS, pending: EXTI_SR2.PFF, priority: ITC_SPR2.VECT5SPR
#define _EXTIPVD_VECTOR_                         5          ///< EXTIPVD interrupt vector: enable: PWR_CSR1.PVDE, pending: PWR_CSR1.PVDIF, priority: ITC_SPR2.VECT5SPR
#define _EXTIB_VECTOR_                           6          ///< EXTIB interrupt vector: enable: EXTI_CR3.PBIS, pending: EXTI_SR2.PBF, priority: ITC_SPR2.VECT6SPR
#define _EXTID_VECTOR_                           7          ///< EXTID interrupt vector: enable: EXTI_CR3.PDIS, pending: EXTI_SR2.PDF, priority: ITC_SPR2.VECT7SPR
#define _EXTI0_VECTOR_                           8          ///< EXTI0 interrupt vector: enable: EXTI_CR1.P0IS, pending: EXTI_SR1.P0F, priority: ITC_SPR3.VECT8SPR
#define _EXTI1_VECTOR_                           9          ///< EXTI1 interrupt vector: enable: EXTI_CR1.P1IS, pending: EXTI_SR1.P1F, priority: ITC_SPR3.VECT9SPR
#define _EXTI2_VECTOR_                           10         ///< EXTI2 interrupt vector: enable: EXTI_CR1.P2IS, pending: EXTI_SR1.P2F, priority: ITC_SPR3.VECT10SPR
#define _EXTI3_VECTOR_                           11         ///< EXTI3 interrupt vector: enable: EXTI_CR1.P3IS, pending: EXTI_SR1.P3F, priority: ITC_SPR3.VECT11SPR
#define _EXTI4_VECTOR_                           12         ///< EXTI4 interrupt vector: enable: EXTI_CR2.P4IS, pending: EXTI_SR1.P4F, priority: ITC_SPR4.VECT12SPR
#define _EXTI5_VECTOR_                           13         ///< EXTI5 interrupt vector: enable: EXTI_CR2.P5IS, pending: EXTI_SR1.P5F, priority: ITC_SPR4.VECT13SPR
#define _EXTI6_VECTOR_                           14         ///< EXTI6 interrupt vector: enable: EXTI_CR2.P6IS, pending: EXTI_SR1.P6F, priority: ITC_SPR4.VECT14SPR
#define _EXTI7_VECTOR_                           15         ///< EXTI7 interrupt vector: enable: EXTI_CR2.P7IS, pending: EXTI_SR1.P7F, priority: ITC_SPR4.VECT15SPR
#define _LCD_SOF_VECTOR_                         16         ///< LCD_SOF interrupt vector: enable: LCD_CR3.SOFIE, pending: LCD_CR3.SOF, priority: ITC_SPR4.VECT16SPR
#define _CLK_CSS_VECTOR_                         17         ///< CLK_CSS interrupt vector: enable: CLK_CSSR.CSSDIE, pending: CLK_CSSR.CSSD, priority: ITC_SPR4.VECT17SPR
#define _CLK_SWITCH_VECTOR_                      17         ///< CLK_SWITCH interrupt vector: enable: CLK_SWCR.SWIEN, pending: CLK_SWCR.SWIF, priority: ITC_SPR4.VECT17SPR
#define _TIM1_BIF_VECTOR_                        17         ///< TIM1_BIF interrupt vector: enable: TIM1_IER.BIE, pending: TIM1_SR1.BIF, priority: ITC_SPR4.VECT17SPR
#define _ADC1_AWD_VECTOR_                        18         ///< ADC1_AWD interrupt vector: enable: ADC1_CR1.AWDIE, pending: ADC1_SR.AWD, priority: ITC_SPR5.VECT18SPR
#define _ADC1_EOC_VECTOR_                        18         ///< ADC1_EOC interrupt vector: enable: ADC1_CR1.EOCIE, pending: ADC1_SR.EOC, priority: ITC_SPR5.VECT18SPR
#define _ADC1_OVER_VECTOR_                       18         ///< ADC1_OVER interrupt vector: enable: ADC1_CR1.OVERIE, pending: ADC1_SR.OVER, priority: ITC_SPR5.VECT18SPR
#define _COMP_EF1_VECTOR_                        18         ///< COMP_EF1 interrupt vector: enable: COMP_CSR1.IE1, pending: COMP_CSR1.EF1, priority: ITC_SPR5.VECT18SPR
#define _COMP_EF2_VECTOR_                        18         ///< COMP_EF2 interrupt vector: enable: COMP_CSR2.IE2, pending: COMP_CSR2.EF2, priority: ITC_SPR5.VECT18SPR
#define _TIM2_BIF_VECTOR_                        19         ///< TIM2_BIF interrupt vector: enable: TIM2_IER.BIE, pending: TIM2_SR1.BIF, priority: ITC_SPR5.VECT19SPR
#define _TIM2_OVR_UIF_VECTOR_                    19         ///< TIM2_OVR_UIF interrupt vector: enable: TIM2_IER.UIE, pending: TIM2_SR1.UIF, priority: ITC_SPR5.VECT19SPR
#define _TIM2_TIF_VECTOR_                        19         ///< TIM2_TIF interrupt vector: enable: TIM2_IER.TIE, pending: TIM2_SR1.TIF, priority: ITC_SPR5.VECT19SPR
#define _TIM2_CAPCOM_CC1IF_VECTOR_               20         ///< TIM2_CAPCOM_CC1IF interrupt vector: enable: TIM2_IER.CC1IE, pending: TIM2_SR1.CC1IF, priority: ITC_SPR6.VECT20SPR
#define _TIM2_CAPCOM_CC2IF_VECTOR_               20         ///< TIM2_CAPCOM_CC2IF interrupt vector: enable: TIM2_IER.CC2IE, pending: TIM2_SR1.CC2IF, priority: ITC_SPR6.VECT20SPR
#define _TIM3_BIF_VECTOR_                        21         ///< TIM3_BIF interrupt vector: enable: TIM3_IER.BIE, pending: TIM3_SR1.BIF, priority: ITC_SPR5.VECT21SPR
#define _TIM3_OVR_UIF_VECTOR_                    21         ///< TIM3_OVR_UIF interrupt vector: enable: TIM3_IER.UIE, pending: TIM3_SR1.UIF, priority: ITC_SPR5.VECT21SPR
#define _TIM3_TIF_VECTOR_                        21         ///< TIM3_TIF interrupt vector: enable: TIM3_IER.TIE, pending: TIM3_SR1.TIF, priority: ITC_SPR5.VECT21SPR
#define _TIM3_CAPCOM_CC1IF_VECTOR_               22         ///< TIM3_CAPCOM_CC1IF interrupt vector: enable: TIM3_IER.CC1IE, pending: TIM3_SR1.CC1IF, priority: ITC_SPR6.VECT22SPR
#define _TIM3_CAPCOM_CC2IF_VECTOR_               22         ///< TIM3_CAPCOM_CC2IF interrupt vector: enable: TIM3_IER.CC2IE, pending: TIM3_SR1.CC2IF, priority: ITC_SPR6.VECT22SPR
#define _TIM1_OVR_UIF_VECTOR_                    23         ///< TIM1_OVR_UIF interrupt vector: enable: TIM1_IER.UIE, pending: TIM1_SR1.UIF, priority: ITC_SPR6.VECT23SPR
#define _TIM1_TIF_VECTOR_                        23         ///< TIM1_TIF interrupt vector: enable: TIM1_IER.TIE, pending: TIM1_SR1.TIF, priority: ITC_SPR6.VECT23SPR
#define _TIM1_CAPCOM_CC1IF_VECTOR_               24         ///< TIM1_CAPCOM_CC1IF interrupt vector: enable: TIM1_IER.CC1IE, pending: TIM1_SR1.CC1IF, priority: ITC_SPR6.VECT24SPR
#define _TIM1_CAPCOM_CC2IF_VECTOR_               24         ///< TIM1_CAPCOM_CC2IF interrupt vector: enable: TIM1_IER.CC2IE, pending: TIM1_SR1.CC2IF, priority: ITC_SPR6.VECT24SPR
#define _TIM1_CAPCOM_CC3IF_VECTOR_               24         ///< TIM1_CAPCOM_CC3IF interrupt vector: enable: TIM1_IER.CC3IE, pending: TIM1_SR1.CC2IF, priority: ITC_SPR6.VECT24SPR
#define _TIM1_CAPCOM_CC4IF_VECTOR_               24         ///< TIM1_CAPCOM_CC4IF interrupt vector: enable: TIM1_IER.CC4IE, pending: TIM1_SR1.CC4IF, priority: ITC_SPR6.VECT24SPR
#define _TIM1_CAPCOM_COMIF_VECTOR_               24         ///< TIM1_CAPCOM_COMIF interrupt vector: enable: TIM1_IER.COMIE, pending: TIM1_SR1.COMIF, priority: ITC_SPR6.VECT24SPR
#define _TIM4_TIF_VECTOR_                        25         ///< TIM4_TIF interrupt vector: enable: TIM4_IER.TIE, pending: TIM4_SR1.TIF, priority: ITC_SPR7.VECT25SPR
#define _TIM4_UIF_VECTOR_                        25         ///< TIM4_UIF interrupt vector: enable: TIM4_IER.UIE, pending: TIM4_SR1.UIF, priority: ITC_SPR7.VECT25SPR
#define _SPI_MODF_VECTOR_                        26         ///< SPI_MODF interrupt vector: enable: SPI1_ICR.ERRIE, pending: SPI1_SR.MODF, priority: ITC_SPR7.VECT26SPR
#define _SPI_OVR_VECTOR_                         26         ///< SPI_OVR interrupt vector: enable: SPI1_ICR.ERRIE, pending: SPI1_SR.OVR, priority: ITC_SPR7.VECT26SPR
#define _SPI_RXNE_VECTOR_                        26         ///< SPI_RXNE interrupt vector: enable: SPI1_ICR.RXIE, pending: SPI1_SR.RXNE, priority: ITC_SPR7.VECT26SPR
#define _SPI_TXE_VECTOR_                         26         ///< SPI_TXE interrupt vector: enable: SPI1_ICR.TXIE, pending: SPI1_SR.TXE, priority: ITC_SPR7.VECT26SPR
#define _SPI_WKUP_VECTOR_                        26         ///< SPI_WKUP interrupt vector: enable: SPI1_ICR.WKIE, pending: SPI1_SR.WKUP, priority: ITC_SPR7.VECT26SPR
#define _USART_T_TC_VECTOR_                      27         ///< USART_T_TC interrupt vector: enable: USART1_CR2.TCIEN, pending: USART1_SR.TC, priority: ITC_SPR7.VECT27SPR
#define _USART_T_TXE_VECTOR_                     27         ///< USART_T_TXE interrupt vector: enable: USART1_CR2.TIEN, pending: USART1_SR.TXE, priority: ITC_SPR7.VECT27SPR
#define _USART_R_IDLE_VECTOR_                    28         ///< USART_R_IDLE interrupt vector: enable: USART1_CR2.ILIEN, pending: USART1_SR.IDLE, priority: ITC_SPR7.VECT28SPR
#define _USART_R_OR_VECTOR_                      28         ///< USART_R_OR interrupt vector: enable: USART1_CR2.RIEN, pending: USART1_SR.OR, priority: ITC_SPR7.VECT28SPR
#define _USART_R_PE_VECTOR_                      28         ///< USART_R_PE interrupt vector: enable: USART1_CR1.PIEN, pending: USART1_SR.PE, priority: ITC_SPR7.VECT28SPR
#define _USART_R_RXNE_VECTOR_                    28         ///< USART_R_RXNE interrupt vector: enable: USART1_CR2.RIEN, pending: USART1_SR.RXNE, priority: ITC_SPR7.VECT28SPR
#define _I2C_ADD10_VECTOR_                       29         ///< I2C_ADD10 interrupt vector: enable: I2C1_ITR.ITEVTEN, pending: I2C1_SR1.ADD10, priority: ITC_SPR8.VECT29SPR
#define _I2C_ADDR_VECTOR_                        29         ///< I2C_ADDR interrupt vector: enable: I2C1_ITR.ITEVTEN, pending: I2C1_SR1.ADDR, priority: ITC_SPR8.VECT29SPR
#define _I2C_AF_VECTOR_                          29         ///< I2C_AF interrupt vector: enable: I2C1_ITR.ITERREN, pending: I2C1_SR2.AF, priority: ITC_SPR8.VECT29SPR
#define _I2C_ARLO_VECTOR_                        29         ///< I2C_ARLO interrupt vector: enable: I2C1_ITR.ITERREN, pending: I2C1_SR2.ARLO, priority: ITC_SPR8.VECT29SPR
#define _I2C_BERR_VECTOR_                        29         ///< I2C_BERR interrupt vector: enable: I2C1_ITR.ITERREN, pending: I2C1_SR2.BERR, priority: ITC_SPR8.VECT29SPR
#define _I2C_BTF_VECTOR_                         29         ///< I2C_BTF interrupt vector: enable: I2C1_ITR.ITEVTEN, pending: I2C1_SR1.BTF, priority: ITC_SPR8.VECT29SPR
#define _I2C_OVR_VECTOR_                         29         ///< I2C_OVR interrupt vector: enable: I2C1_ITR.ITERREN, pending: I2C1_SR2.OVR, priority: ITC_SPR8.VECT29SPR
#define _I2C_RXNE_VECTOR_                        29         ///< I2C_RXNE interrupt vector: enable: I2C1_ITR.ITBUFEN, pending: I2C1_SR1.RXNE, priority: ITC_SPR8.VECT29SPR
#define _I2C_SB_VECTOR_                          29         ///< I2C_SB interrupt vector: enable: I2C1_ITR.ITEVTEN, pending: I2C1_SR1.SB, priority: ITC_SPR8.VECT29SPR
#define _I2C_STOPF_VECTOR_                       29         ///< I2C_STOPF interrupt vector: enable: I2C1_ITR.ITEVTEN, pending: I2C1_SR1.STOPF, priority: ITC_SPR8.VECT29SPR
#define _I2C_TXE_VECTOR_                         29         ///< I2C_TXE interrupt vector: enable: I2C1_ITR.ITBUFEN, pending: I2C1_SR1.TXE, priority: ITC_SPR8.VECT29SPR
#define _I2C_WUFH_VECTOR_                        29         ///< I2C_WUFH interrupt vector: enable: I2C1_ITR.ITEVTEN, pending: I2C1_SR2.WUFH, priority: ITC_SPR8.VECT29SPR


/*-------------------------------------------------------------------------
  DEFINITION OF STM8 PERIPHERAL REGISTERS
-------------------------------------------------------------------------*/

//------------------------
// Module ADC1
//------------------------

/** struct containing ADC1 module registers */
typedef struct {

  /** ADC1 configuration register 1 (CR1 at 0x5340) */
  union {

    /// bytewise access to CR1
    uint8_t  byte;

    /// bitwise access to register CR1
    struct {
      BITS   ADON                : 1;      // bit 0
      BITS   START               : 1;      // bit 1
      BITS   CONT                : 1;      // bit 2
      BITS   EOCIE               : 1;      // bit 3
      BITS   AWDIE               : 1;      // bit 4
      BITS   RES                 : 2;      // bits 5-6
      BITS   OVERIE              : 1;      // bit 7
    };  // CR1 bitfield

    /// register _ADC1_CR1 reset value
    #define sfr_ADC1_CR1_RESET_VALUE   ((uint8_t) 0x00)

  } CR1;


  /** ADC1 configuration register 2 (CR2 at 0x5341) */
  union {

    /// bytewise access to CR2
    uint8_t  byte;

    /// bitwise access to register CR2
    struct {
      BITS   SMTP1               : 3;      // bits 0-2
      BITS   EXTSEL0             : 1;      // bit 3
      BITS   EXTSEL1             : 1;      // bit 4
      BITS   TRIG_EDGE0          : 1;      // bit 5
      BITS   TRIG_EDGE1          : 1;      // bit 6
      BITS   PRESC               : 1;      // bit 7
    };  // CR2 bitfield

    /// register _ADC1_CR2 reset value
    #define sfr_ADC1_CR2_RESET_VALUE   ((uint8_t) 0x00)

  } CR2;


  /** ADC1 configuration register 3 (CR3 at 0x5342) */
  union {

    /// bytewise access to CR3
    uint8_t  byte;

    /// bitwise access to register CR3
    struct {
      BITS   CHSEL               : 5;      // bits 0-4
      BITS   SMTP2               : 3;      // bits 5-7
    };  // CR3 bitfield

    /// register _ADC1_CR3 reset value
    #define sfr_ADC1_CR3_RESET_VALUE   ((uint8_t) 0x1F)

  } CR3;


  /** ADC1 status register (SR at 0x5343) */
  union {

    /// bytewise access to SR
    uint8_t  byte;

    /// bitwise access to register SR
    struct {
      BITS   EOC                 : 1;      // bit 0
      BITS   AWD                 : 1;      // bit 1
      BITS   OVER                : 1;      // bit 2
      BITS                       : 5;      // 5 bits
    };  // SR bitfield

    /// register _ADC1_SR reset value
    #define sfr_ADC1_SR_RESET_VALUE   ((uint8_t) 0x00)

  } SR;


  /** ADC1 data register high (DRH at 0x5344) */
  union {

    /// bytewise access to DRH
    uint8_t  byte;

    /// bitwise access to register DRH
    struct {
      BITS   CONV_DATA8          : 1;      // bit 0
      BITS   CONV_DATA9          : 1;      // bit 1
      BITS   CONV_DATA10         : 1;      // bit 2
      BITS   CONV_DATA11         : 1;      // bit 3
      BITS                       : 4;      // 4 bits
    };  // DRH bitfield

    /// register _ADC1_DRH reset value
    #define sfr_ADC1_DRH_RESET_VALUE   ((uint8_t) 0x00)

  } DRH;


  /** ADC1 data register low (DRL at 0x5345) */
  union {

    /// bytewise access to DRL
    uint8_t  byte;

    /// bitwise access to register DRL
    struct {
      BITS   CONV_DATA0          : 1;      // bit 0
      BITS   CONV_DATA1          : 1;      // bit 1
      BITS   CONV_DATA2          : 1;      // bit 2
      BITS   CONV_DATA3          : 1;      // bit 3
      BITS   CONV_DATA4          : 1;      // bit 4
      BITS   CONV_DATA5          : 1;      // bit 5
      BITS   CONV_DATA6          : 1;      // bit 6
      BITS   CONV_DATA7          : 1;      // bit 7
    };  // DRL bitfield

    /// register _ADC1_DRL reset value
    #define sfr_ADC1_DRL_RESET_VALUE   ((uint8_t) 0x00)

  } DRL;


  /** ADC1 high threshold register high (HTRH at 0x5346) */
  union {

    /// bytewise access to HTRH
    uint8_t  byte;

    /// bitwise access to register HTRH
    struct {
      BITS   HT8                 : 1;      // bit 0
      BITS   HT9                 : 1;      // bit 1
      BITS   HT10                : 1;      // bit 2
      BITS   HT11                : 1;      // bit 3
      BITS                       : 4;      // 4 bits
    };  // HTRH bitfield

    /// register _ADC1_HTRH reset value
    #define sfr_ADC1_HTRH_RESET_VALUE   ((uint8_t) 0x0F)

  } HTRH;


  /** ADC1 high threshold register low (HTRL at 0x5347) */
  union {

    /// bytewise access to HTRL
    uint8_t  byte;

    /// bitwise access to register HTRL
    struct {
      BITS   HT0                 : 1;      // bit 0
      BITS   HT1                 : 1;      // bit 1
      BITS   HT2                 : 1;      // bit 2
      BITS   HT3                 : 1;      // bit 3
      BITS   HT4                 : 1;      // bit 4
      BITS   HT5                 : 1;      // bit 5
      BITS   HT6                 : 1;      // bit 6
      BITS   HT7                 : 1;      // bit 7
    };  // HTRL bitfield

    /// register _ADC1_HTRL reset value
    #define sfr_ADC1_HTRL_RESET_VALUE   ((uint8_t) 0xFF)

  } HTRL;


  /** ADC1 low threshold register high (LTRH at 0x5348) */
  union {

    /// bytewise access to LTRH
    uint8_t  byte;

    /// bitwise access to register LTRH
    struct {
      BITS   LT8                 : 1;      // bit 0
      BITS   LT9                 : 1;      // bit 1
      BITS   LT10                : 1;      // bit 2
      BITS   LT11                : 1;      // bit 3
      BITS                       : 4;      // 4 bits
    };  // LTRH bitfield

    /// register _ADC1_LTRH reset value
    #define sfr_ADC1_LTRH_RESET_VALUE   ((uint8_t) 0x00)

  } LTRH;


  /** ADC1 low threshold register low (LTRL at 0x5349) */
  union {

    /// bytewise access to LTRL
    uint8_t  byte;

    /// bitwise access to register LTRL
    struct {
      BITS   LT0                 : 1;      // bit 0
      BITS   LT1                 : 1;      // bit 1
      BITS   LT2                 : 1;      // bit 2
      BITS   LT3                 : 1;      // bit 3
      BITS   LT4                 : 1;      // bit 4
      BITS   LT5                 : 1;      // bit 5
      BITS   LT6                 : 1;      // bit 6
      BITS   LT7                 : 1;      // bit 7
    };  // LTRL bitfield

    /// register _ADC1_LTRL reset value
    #define sfr_ADC1_LTRL_RESET_VALUE   ((uint8_t) 0x00)

  } LTRL;


  /** ADC1 channel sequence 1 register (SQR1 at 0x534a) */
  union {

    /// bytewise access to SQR1
    uint8_t  byte;

    /// bitwise access to register SQR1
    struct {
      BITS   CHSEL_S24           : 1;      // bit 0
      BITS   CHSEL_S25           : 1;      // bit 1
      BITS   CHSEL_S26           : 1;      // bit 2
      BITS   CHSEL_S27           : 1;      // bit 3
      BITS   CHSEL_S28           : 1;      // bit 4
      BITS   CHSEL_S29           : 1;      // bit 5
      BITS                       : 1;      // 1 bit
      BITS   DMAOFF              : 1;      // bit 7
    };  // SQR1 bitfield

    /// register _ADC1_SQR1 reset value
    #define sfr_ADC1_SQR1_RESET_VALUE   ((uint8_t) 0x00)

  } SQR1;


  /** ADC1 channel sequence 2 register (SQR2 at 0x534b) */
  union {

    /// bytewise access to SQR2
    uint8_t  byte;

    /// bitwise access to register SQR2
    struct {
      BITS   CHSEL_S16           : 1;      // bit 0
      BITS   CHSEL_S17           : 1;      // bit 1
      BITS   CHSEL_S18           : 1;      // bit 2
      BITS   CHSEL_S19           : 1;      // bit 3
      BITS   CHSEL_S20           : 1;      // bit 4
      BITS   CHSEL_S21           : 1;      // bit 5
      BITS   CHSEL_S22           : 1;      // bit 6
      BITS   CHSEL_S23           : 1;      // bit 7
    };  // SQR2 bitfield

    /// register _ADC1_SQR2 reset value
    #define sfr_ADC1_SQR2_RESET_VALUE   ((uint8_t) 0x00)

  } SQR2;


  /** ADC1 channel sequence 3 register (SQR3 at 0x534c) */
  union {

    /// bytewise access to SQR3
    uint8_t  byte;

    /// bitwise access to register SQR3
    struct {
      BITS   CHSEL_S8            : 1;      // bit 0
      BITS   CHSEL_S9            : 1;      // bit 1
      BITS   CHSEL_S10           : 1;      // bit 2
      BITS   CHSEL_S11           : 1;      // bit 3
      BITS   CHSEL_S12           : 1;      // bit 4
      BITS   CHSEL_S13           : 1;      // bit 5
      BITS   CHSEL_S14           : 1;      // bit 6
      BITS   CHSEL_S15           : 1;      // bit 7
    };  // SQR3 bitfield

    /// register _ADC1_SQR3 reset value
    #define sfr_ADC1_SQR3_RESET_VALUE   ((uint8_t) 0x00)

  } SQR3;


  /** ADC1 channel sequence 4 register (SQR4 at 0x534d) */
  union {

    /// bytewise access to SQR4
    uint8_t  byte;

    /// bitwise access to register SQR4
    struct {
      BITS   CHSEL_S0            : 1;      // bit 0
      BITS   CHSEL_S1            : 1;      // bit 1
      BITS   CHSEL_S2            : 1;      // bit 2
      BITS   CHSEL_S3            : 1;      // bit 3
      BITS   CHSEL_S4            : 1;      // bit 4
      BITS   CHSEL_S5            : 1;      // bit 5
      BITS   CHSEL_S6            : 1;      // bit 6
      BITS   CHSEL_S7            : 1;      // bit 7
    };  // SQR4 bitfield

    /// register _ADC1_SQR4 reset value
    #define sfr_ADC1_SQR4_RESET_VALUE   ((uint8_t) 0x00)

  } SQR4;


  /** ADC1 trigger disable 1 (TRIGR1 at 0x534e) */
  union {

    /// bytewise access to TRIGR1
    uint8_t  byte;

    /// bitwise access to register TRIGR1
    struct {
      BITS   TRIG24              : 1;      // bit 0
      BITS   TRIG25              : 1;      // bit 1
      BITS   TRIG26              : 1;      // bit 2
      BITS   TRIG27              : 1;      // bit 3
      BITS   VREFINTON           : 1;      // bit 4
      BITS   TSON                : 1;      // bit 5
      BITS                       : 2;      // 2 bits
    };  // TRIGR1 bitfield

    /// register _ADC1_TRIGR1 reset value
    #define sfr_ADC1_TRIGR1_RESET_VALUE   ((uint8_t) 0x00)

  } TRIGR1;


  /** ADC1 trigger disable 2 (TRIGR2 at 0x534f) */
  union {

    /// bytewise access to TRIGR2
    uint8_t  byte;

    /// bitwise access to register TRIGR2
    struct {
      BITS   TRIG16              : 1;      // bit 0
      BITS   TRIG17              : 1;      // bit 1
      BITS   TRIG18              : 1;      // bit 2
      BITS   TRIG19              : 1;      // bit 3
      BITS   TRIG20              : 1;      // bit 4
      BITS   TRIG21              : 1;      // bit 5
      BITS   TRIG22              : 1;      // bit 6
      BITS   TRIG23              : 1;      // bit 7
    };  // TRIGR2 bitfield

    /// register _ADC1_TRIGR2 reset value
    #define sfr_ADC1_TRIGR2_RESET_VALUE   ((uint8_t) 0x00)

  } TRIGR2;


  /** ADC1 trigger disable 3 (TRIGR3 at 0x5350) */
  union {

    /// bytewise access to TRIGR3
    uint8_t  byte;

    /// bitwise access to register TRIGR3
    struct {
      BITS   TRIG8               : 1;      // bit 0
      BITS   TRIG9               : 1;      // bit 1
      BITS   TRIG10              : 1;      // bit 2
      BITS   TRIG11              : 1;      // bit 3
      BITS   TRIG12              : 1;      // bit 4
      BITS   TRIG13              : 1;      // bit 5
      BITS   TRIG14              : 1;      // bit 6
      BITS   TRIG15              : 1;      // bit 7
    };  // TRIGR3 bitfield

    /// register _ADC1_TRIGR3 reset value
    #define sfr_ADC1_TRIGR3_RESET_VALUE   ((uint8_t) 0x00)

  } TRIGR3;


  /** ADC1 trigger disable 4 (TRIGR4 at 0x5351) */
  union {

    /// bytewise access to TRIGR4
    uint8_t  byte;

    /// bitwise access to register TRIGR4
    struct {
      BITS   TRIG0               : 1;      // bit 0
      BITS   TRIG1               : 1;      // bit 1
      BITS   TRIG2               : 1;      // bit 2
      BITS   TRIG3               : 1;      // bit 3
      BITS   TRIG4               : 1;      // bit 4
      BITS   TRIG5               : 1;      // bit 5
      BITS   TRIG6               : 1;      // bit 6
      BITS   TRIG7               : 1;      // bit 7
    };  // TRIGR4 bitfield

    /// register _ADC1_TRIGR4 reset value
    #define sfr_ADC1_TRIGR4_RESET_VALUE   ((uint8_t) 0x00)

  } TRIGR4;

} ADC1_t;

/// access to ADC1 SFR registers
#define sfr_ADC1   (*((ADC1_t*) 0x5340))


//------------------------
// Module BEEP
//------------------------

/** struct containing BEEP module registers */
typedef struct {

  /** BEEP control/status register 1 (CSR1 at 0x50f0) */
  union {

    /// bytewise access to CSR1
    uint8_t  byte;

    /// bitwise access to register CSR1
    struct {
      BITS   MSR                 : 1;      // bit 0
      BITS                       : 7;      // 7 bits
    };  // CSR1 bitfield

    /// register _BEEP_CSR1 reset value
    #define sfr_BEEP_CSR1_RESET_VALUE   ((uint8_t) 0x00)

  } CSR1;


  /// Reserved register (2B)
  uint8_t     Reserved_1[2];


  /** BEEP control/status register 2 (CSR2 at 0x50f3) */
  union {

    /// bytewise access to CSR2
    uint8_t  byte;

    /// bitwise access to register CSR2
    struct {
      BITS   BEEPDIV             : 5;      // bits 0-4
      BITS   BEEPEN              : 1;      // bit 5
      BITS   BEEPSEL             : 2;      // bits 6-7
    };  // CSR2 bitfield

    /// register _BEEP_CSR2 reset value
    #define sfr_BEEP_CSR2_RESET_VALUE   ((uint8_t) 0x1F)

  } CSR2;

} BEEP_t;

/// access to BEEP SFR registers
#define sfr_BEEP   (*((BEEP_t*) 0x50f0))


//------------------------
// Module CLK
//------------------------

/** struct containing CLK module registers */
typedef struct {

  /** Clock master divider register (CKDIVR at 0x50c0) */
  union {

    /// bytewise access to CKDIVR
    uint8_t  byte;

    /// bitwise access to register CKDIVR
    struct {
      BITS   CKM                 : 3;      // bits 0-2
      BITS                       : 5;      // 5 bits
    };  // CKDIVR bitfield

    /// register _CLK_CKDIVR reset value
    #define sfr_CLK_CKDIVR_RESET_VALUE   ((uint8_t) 0x03)

  } CKDIVR;


  /** Clock RTC register (CRTCR at 0x50c1) */
  union {

    /// bytewise access to CRTCR
    uint8_t  byte;

    /// bitwise access to register CRTCR
    struct {
      BITS   RTCSWBSY            : 1;      // bit 0
      BITS   RTCSEL0             : 1;      // bit 1
      BITS   RTCSEL1             : 1;      // bit 2
      BITS   RTCSEL2             : 1;      // bit 3
      BITS   RTCSEL3             : 1;      // bit 4
      BITS   RTCDIV0             : 1;      // bit 5
      BITS   RTCDIV1             : 1;      // bit 6
      BITS   RTCDIV2             : 1;      // bit 7
    };  // CRTCR bitfield

    /// register _CLK_CRTCR reset value
    #define sfr_CLK_CRTCR_RESET_VALUE   ((uint8_t) 0x00)

  } CRTCR;


  /** Internal clock control register (ICKCR at 0x50c2) */
  union {

    /// bytewise access to ICKCR
    uint8_t  byte;

    /// bitwise access to register ICKCR
    struct {
      BITS   HSION               : 1;      // bit 0
      BITS   HSIRDY              : 1;      // bit 1
      BITS   LSION               : 1;      // bit 2
      BITS   LSIRDY              : 1;      // bit 3
      BITS   SAHALT              : 1;      // bit 4
      BITS   FHWU                : 1;      // bit 5
      BITS   BEEPAHALT           : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // ICKCR bitfield

    /// register _CLK_ICKCR reset value
    #define sfr_CLK_ICKCR_RESET_VALUE   ((uint8_t) 0x11)

  } ICKCR;


  /** Peripheral clock gating register 1 (PCKENR1 at 0x50c3) */
  union {

    /// bytewise access to PCKENR1
    uint8_t  byte;

    /// bitwise access to register PCKENR1
    struct {
      BITS   PCKEN10             : 1;      // bit 0
      BITS   PCKEN11             : 1;      // bit 1
      BITS   PCKEN12             : 1;      // bit 2
      BITS   PCKEN13             : 1;      // bit 3
      BITS   PCKEN14             : 1;      // bit 4
      BITS   PCKEN15             : 1;      // bit 5
      BITS   PCKEN16             : 1;      // bit 6
      BITS   PCKEN17             : 1;      // bit 7
    };  // PCKENR1 bitfield

    /// register _CLK_PCKENR1 reset value
    #define sfr_CLK_PCKENR1_RESET_VALUE   ((uint8_t) 0x00)

  } PCKENR1;


  /** Peripheral clock gating register 2 (PCKENR2 at 0x50c4) */
  union {

    /// bytewise access to PCKENR2
    uint8_t  byte;

    /// bitwise access to register PCKENR2
    struct {
      BITS   PCKEN20             : 1;      // bit 0
      BITS   PCKEN21             : 1;      // bit 1
      BITS   PCKEN22             : 1;      // bit 2
      BITS   PCKEN23             : 1;      // bit 3
      BITS   PCKEN24             : 1;      // bit 4
      BITS   PCKEN25             : 1;      // bit 5
      BITS                       : 1;      // 1 bit
      BITS   PCKEN27             : 1;      // bit 7
    };  // PCKENR2 bitfield

    /// register _CLK_PCKENR2 reset value
    #define sfr_CLK_PCKENR2_RESET_VALUE   ((uint8_t) 0x80)

  } PCKENR2;


  /** Configurable clock control register (CCOR at 0x50c5) */
  union {

    /// bytewise access to CCOR
    uint8_t  byte;

    /// bitwise access to register CCOR
    struct {
      BITS   CCOSWBSY            : 1;      // bit 0
      BITS   CCOSEL              : 4;      // bits 1-4
      BITS   CCODIV              : 3;      // bits 5-7
    };  // CCOR bitfield

    /// register _CLK_CCOR reset value
    #define sfr_CLK_CCOR_RESET_VALUE   ((uint8_t) 0x00)

  } CCOR;


  /** External clock control register (ECKR at 0x50c6) */
  union {

    /// bytewise access to ECKR
    uint8_t  byte;

    /// bitwise access to register ECKR
    struct {
      BITS   HSEON               : 1;      // bit 0
      BITS   HSERDY              : 1;      // bit 1
      BITS   LSEON               : 1;      // bit 2
      BITS   LSERDY              : 1;      // bit 3
      BITS   HSEBYP              : 1;      // bit 4
      BITS   LSEBYP              : 1;      // bit 5
      BITS                       : 2;      // 2 bits
    };  // ECKR bitfield

    /// register _CLK_ECKR reset value
    #define sfr_CLK_ECKR_RESET_VALUE   ((uint8_t) 0x00)

  } ECKR;


  /** Clock master status register (SCSR at 0x50c7) */
  union {

    /// bytewise access to SCSR
    uint8_t  byte;

    /// bitwise access to register SCSR
    struct {
      BITS   CKM                 : 8;      // bits 0-7
    };  // SCSR bitfield

    /// register _CLK_SCSR reset value
    #define sfr_CLK_SCSR_RESET_VALUE   ((uint8_t) 0x01)

  } SCSR;


  /** Clock master switch register (SWR at 0x50c8) */
  union {

    /// bytewise access to SWR
    uint8_t  byte;

    /// bitwise access to register SWR
    struct {
      BITS   SWI                 : 8;      // bits 0-7
    };  // SWR bitfield

    /// register _CLK_SWR reset value
    #define sfr_CLK_SWR_RESET_VALUE   ((uint8_t) 0x01)

  } SWR;


  /** Clock switch control register (SWCR at 0x50c9) */
  union {

    /// bytewise access to SWCR
    uint8_t  byte;

    /// bitwise access to register SWCR
    struct {
      BITS   SWBSY               : 1;      // bit 0
      BITS   SWEN                : 1;      // bit 1
      BITS   SWIEN               : 1;      // bit 2
      BITS   SWIF                : 1;      // bit 3
      BITS                       : 4;      // 4 bits
    };  // SWCR bitfield

    /// register _CLK_SWCR reset value
    #define sfr_CLK_SWCR_RESET_VALUE   ((uint8_t) 0x00)

  } SWCR;


  /** Clock security system register (CSSR at 0x50ca) */
  union {

    /// bytewise access to CSSR
    uint8_t  byte;

    /// bitwise access to register CSSR
    struct {
      BITS   CSSEN               : 1;      // bit 0
      BITS   AUX                 : 1;      // bit 1
      BITS   CSSDIE              : 1;      // bit 2
      BITS   CSSD                : 1;      // bit 3
      BITS   CSSDGON             : 1;      // bit 4
      BITS                       : 3;      // 3 bits
    };  // CSSR bitfield

    /// register _CLK_CSSR reset value
    #define sfr_CLK_CSSR_RESET_VALUE   ((uint8_t) 0x00)

  } CSSR;


  /** Clock BEEP register (CBEEPR at 0x50cb) */
  union {

    /// bytewise access to CBEEPR
    uint8_t  byte;

    /// bitwise access to register CBEEPR
    struct {
      BITS   BEEPSWBSY           : 1;      // bit 0
      BITS   CLKBEEPSEL0         : 1;      // bit 1
      BITS   CLKBEEPSEL1         : 1;      // bit 2
      BITS                       : 5;      // 5 bits
    };  // CBEEPR bitfield

    /// register _CLK_CBEEPR reset value
    #define sfr_CLK_CBEEPR_RESET_VALUE   ((uint8_t) 0x00)

  } CBEEPR;


  /** HSI calibration register (HSICALR at 0x50cc) */
  union {

    /// bytewise access to HSICALR
    uint8_t  byte;

    /// bitwise access to register HSICALR
    struct {
      BITS   HSICAL              : 8;      // bits 0-7
    };  // HSICALR bitfield

    /// register _CLK_HSICALR reset value
    #define sfr_CLK_HSICALR_RESET_VALUE   ((uint8_t) 0x00)

  } HSICALR;


  /** HSI clock calibration trimming register (HSITRIMR at 0x50cd) */
  union {

    /// bytewise access to HSITRIMR
    uint8_t  byte;

    /// bitwise access to register HSITRIMR
    struct {
      BITS   HSITRIM             : 8;      // bits 0-7
    };  // HSITRIMR bitfield

    /// register _CLK_HSITRIMR reset value
    #define sfr_CLK_HSITRIMR_RESET_VALUE   ((uint8_t) 0x00)

  } HSITRIMR;


  /** HSI unlock register (HSIUNLCKR at 0x50ce) */
  union {

    /// bytewise access to HSIUNLCKR
    uint8_t  byte;

    /// bitwise access to register HSIUNLCKR
    struct {
      BITS   HSIUNLCK            : 8;      // bits 0-7
    };  // HSIUNLCKR bitfield

    /// register _CLK_HSIUNLCKR reset value
    #define sfr_CLK_HSIUNLCKR_RESET_VALUE   ((uint8_t) 0x00)

  } HSIUNLCKR;


  /** Main regulator control status register (REGCSR at 0x50cf) */
  union {

    /// bytewise access to REGCSR
    uint8_t  byte;

    /// bitwise access to register REGCSR
    struct {
      BITS   REGREADY            : 1;      // bit 0
      BITS   REGOFF              : 1;      // bit 1
      BITS   HSIPD               : 1;      // bit 2
      BITS   LSIPD               : 1;      // bit 3
      BITS   HSEPD               : 1;      // bit 4
      BITS   LSEPD               : 1;      // bit 5
      BITS   EEBUSY              : 1;      // bit 6
      BITS   EEREADY             : 1;      // bit 7
    };  // REGCSR bitfield

    /// register _CLK_REGCSR reset value
    #define sfr_CLK_REGCSR_RESET_VALUE   ((uint8_t) 0x38)

  } REGCSR;

} CLK_t;

/// access to CLK SFR registers
#define sfr_CLK   (*((CLK_t*) 0x50c0))


//------------------------
// Module COMP
//------------------------

/** struct containing COMP module registers */
typedef struct {

  /** Comparator control and status register 1 (CSR1 at 0x5440) */
  union {

    /// bytewise access to CSR1
    uint8_t  byte;

    /// bitwise access to register CSR1
    struct {
      BITS   CMP1                : 2;      // bits 0-1
      BITS   STE                 : 1;      // bit 2
      BITS   CMP1OUT             : 1;      // bit 3
      BITS   EF1                 : 1;      // bit 4
      BITS   IE1                 : 1;      // bit 5
      BITS                       : 2;      // 2 bits
    };  // CSR1 bitfield

    /// register _COMP_CSR1 reset value
    #define sfr_COMP_CSR1_RESET_VALUE   ((uint8_t) 0x00)

  } CSR1;


  /** Comparator control and status register 2 (CSR2 at 0x5441) */
  union {

    /// bytewise access to CSR2
    uint8_t  byte;

    /// bitwise access to register CSR2
    struct {
      BITS   CMP2                : 2;      // bits 0-1
      BITS   SPEED               : 1;      // bit 2
      BITS   CMP2OUT             : 1;      // bit 3
      BITS   EF2                 : 1;      // bit 4
      BITS   IE2                 : 1;      // bit 5
      BITS                       : 2;      // 2 bits
    };  // CSR2 bitfield

    /// register _COMP_CSR2 reset value
    #define sfr_COMP_CSR2_RESET_VALUE   ((uint8_t) 0x00)

  } CSR2;


  /** Comparator control and status register 3 (CSR3 at 0x5442) */
  union {

    /// bytewise access to CSR3
    uint8_t  byte;

    /// bitwise access to register CSR3
    struct {
      BITS   VREFOUTEN           : 1;      // bit 0
      BITS   WNDWE               : 1;      // bit 1
      BITS   VREFEN              : 1;      // bit 2
      BITS   INSEL               : 3;      // bits 3-5
      BITS   OUTSEL              : 2;      // bits 6-7
    };  // CSR3 bitfield

    /// register _COMP_CSR3 reset value
    #define sfr_COMP_CSR3_RESET_VALUE   ((uint8_t) 0x00)

  } CSR3;


  /** Comparator control and status register 4 (CSR4 at 0x5443) */
  union {

    /// bytewise access to CSR4
    uint8_t  byte;

    /// bitwise access to register CSR4
    struct {
      BITS   INVTRIG             : 3;      // bits 0-2
      BITS   NINVTRIG            : 3;      // bits 3-5
      BITS                       : 2;      // 2 bits
    };  // CSR4 bitfield

    /// register _COMP_CSR4 reset value
    #define sfr_COMP_CSR4_RESET_VALUE   ((uint8_t) 0x00)

  } CSR4;


  /** Comparator control and status register 5 (CSR5 at 0x5444) */
  union {

    /// bytewise access to CSR5
    uint8_t  byte;

    /// bitwise access to register CSR5
    struct {
      BITS   VREFTRIG            : 3;      // bits 0-2
      BITS   DACTRIG             : 3;      // bits 3-5
      BITS                       : 2;      // 2 bits
    };  // CSR5 bitfield

    /// register _COMP_CSR5 reset value
    #define sfr_COMP_CSR5_RESET_VALUE   ((uint8_t) 0x00)

  } CSR5;

} COMP_t;

/// access to COMP SFR registers
#define sfr_COMP   (*((COMP_t*) 0x5440))


//------------------------
// Module CPU
//------------------------

/** struct containing CPU module registers */
typedef struct {

  /** Accumulator (A at 0x7f00) */
  union {

    /// bytewise access to A
    uint8_t  byte;

    /// skip bitwise access to register A

    /// register _CPU_A reset value
    #define sfr_CPU_A_RESET_VALUE   ((uint8_t) 0x00)

  } A;


  /** Program counter extended (PCE at 0x7f01) */
  union {

    /// bytewise access to PCE
    uint8_t  byte;

    /// skip bitwise access to register PCE

    /// register _CPU_PCE reset value
    #define sfr_CPU_PCE_RESET_VALUE   ((uint8_t) 0x00)

  } PCE;


  /** Program counter high (PCH at 0x7f02) */
  union {

    /// bytewise access to PCH
    uint8_t  byte;

    /// skip bitwise access to register PCH

    /// register _CPU_PCH reset value
    #define sfr_CPU_PCH_RESET_VALUE   ((uint8_t) 0x00)

  } PCH;


  /** Program counter low (PCL at 0x7f03) */
  union {

    /// bytewise access to PCL
    uint8_t  byte;

    /// skip bitwise access to register PCL

    /// register _CPU_PCL reset value
    #define sfr_CPU_PCL_RESET_VALUE   ((uint8_t) 0x00)

  } PCL;


  /** X index register high (XH at 0x7f04) */
  union {

    /// bytewise access to XH
    uint8_t  byte;

    /// skip bitwise access to register XH

    /// register _CPU_XH reset value
    #define sfr_CPU_XH_RESET_VALUE   ((uint8_t) 0x00)

  } XH;


  /** X index register low (XL at 0x7f05) */
  union {

    /// bytewise access to XL
    uint8_t  byte;

    /// skip bitwise access to register XL

    /// register _CPU_XL reset value
    #define sfr_CPU_XL_RESET_VALUE   ((uint8_t) 0x00)

  } XL;


  /** Y index register high (YH at 0x7f06) */
  union {

    /// bytewise access to YH
    uint8_t  byte;

    /// skip bitwise access to register YH

    /// register _CPU_YH reset value
    #define sfr_CPU_YH_RESET_VALUE   ((uint8_t) 0x00)

  } YH;


  /** Y index register low (YL at 0x7f07) */
  union {

    /// bytewise access to YL
    uint8_t  byte;

    /// skip bitwise access to register YL

    /// register _CPU_YL reset value
    #define sfr_CPU_YL_RESET_VALUE   ((uint8_t) 0x00)

  } YL;


  /** Stack pointer high (SPH at 0x7f08) */
  union {

    /// bytewise access to SPH
    uint8_t  byte;

    /// skip bitwise access to register SPH

    /// register _CPU_SPH reset value
    #define sfr_CPU_SPH_RESET_VALUE   ((uint8_t) 0x03)

  } SPH;


  /** Stack pointer low (SPL at 0x7f09) */
  union {

    /// bytewise access to SPL
    uint8_t  byte;

    /// skip bitwise access to register SPL

    /// register _CPU_SPL reset value
    #define sfr_CPU_SPL_RESET_VALUE   ((uint8_t) 0xFF)

  } SPL;


  /** Condition code register (CCR at 0x7f0a) */
  union {

    /// bytewise access to CCR
    uint8_t  byte;

    /// bitwise access to register CCR
    struct {
      BITS   C                   : 1;      // bit 0
      BITS   Z                   : 1;      // bit 1
      BITS   N                   : 1;      // bit 2
      BITS   I0                  : 1;      // bit 3
      BITS   H                   : 1;      // bit 4
      BITS   I1                  : 1;      // bit 5
      BITS                       : 1;      // 1 bit
      BITS   V                   : 1;      // bit 7
    };  // CCR bitfield

    /// register _CPU_CCR reset value
    #define sfr_CPU_CCR_RESET_VALUE   ((uint8_t) 0x28)

  } CCR;


  /// Reserved register (85B)
  uint8_t     Reserved_1[85];


  /** Global configuration register (CFG_GCR at 0x7f60) */
  union {

    /// bytewise access to CFG_GCR
    uint8_t  byte;

    /// bitwise access to register CFG_GCR
    struct {
      BITS   SWD                 : 1;      // bit 0
      BITS   AL                  : 1;      // bit 1
      BITS                       : 6;      // 6 bits
    };  // CFG_GCR bitfield

    /// register _CPU_CFG_GCR reset value
    #define sfr_CPU_CFG_GCR_RESET_VALUE   ((uint8_t) 0x00)

  } CFG_GCR;

} CPU_t;

/// access to CPU SFR registers
#define sfr_CPU   (*((CPU_t*) 0x7f00))


//------------------------
// Module DAC
//------------------------

/** struct containing DAC module registers */
typedef struct {

  /** DAC control register 1 (CR1 at 0x5380) */
  union {

    /// bytewise access to CR1
    uint8_t  byte;

    /// bitwise access to register CR1
    struct {
      BITS   EN                  : 1;      // bit 0
      BITS   BOFF                : 1;      // bit 1
      BITS   TEN                 : 1;      // bit 2
      BITS   TSEL                : 3;      // bits 3-5
      BITS                       : 2;      // 2 bits
    };  // CR1 bitfield

    /// register _DAC_CR1 reset value
    #define sfr_DAC_CR1_RESET_VALUE   ((uint8_t) 0x00)

  } CR1;


  /** DAC control register 2 (CR2 at 0x5381) */
  union {

    /// bytewise access to CR2
    uint8_t  byte;

    /// bitwise access to register CR2
    struct {
      BITS                       : 4;      // 4 bits
      BITS   DMAEN               : 1;      // bit 4
      BITS   DMAUDRIE            : 1;      // bit 5
      BITS                       : 2;      // 2 bits
    };  // CR2 bitfield

    /// register _DAC_CR2 reset value
    #define sfr_DAC_CR2_RESET_VALUE   ((uint8_t) 0x00)

  } CR2;


  /// Reserved register (2B)
  uint8_t     Reserved_1[2];


  /** DAC software trigger register (SWTRIGR at 0x5384) */
  union {

    /// bytewise access to SWTRIGR
    uint8_t  byte;

    /// bitwise access to register SWTRIGR
    struct {
      BITS   SWTRIG              : 1;      // bit 0
      BITS                       : 7;      // 7 bits
    };  // SWTRIGR bitfield

    /// register _DAC_SWTRIGR reset value
    #define sfr_DAC_SWTRIGR_RESET_VALUE   ((uint8_t) 0x00)

  } SWTRIGR;


  /** DAC status register (SR at 0x5385) */
  union {

    /// bytewise access to SR
    uint8_t  byte;

    /// bitwise access to register SR
    struct {
      BITS   DMAUDR              : 1;      // bit 0
      BITS                       : 7;      // 7 bits
    };  // SR bitfield

    /// register _DAC_SR reset value
    #define sfr_DAC_SR_RESET_VALUE   ((uint8_t) 0x00)

  } SR;


  /// Reserved register (2B)
  uint8_t     Reserved_2[2];


  /** DAC right aligned data holding register high (RDHRH at 0x5388) */
  union {

    /// bytewise access to RDHRH
    uint8_t  byte;

    /// bitwise access to register RDHRH
    struct {
      BITS   RDHRH               : 4;      // bits 0-3
      BITS                       : 4;      // 4 bits
    };  // RDHRH bitfield

    /// register _DAC_RDHRH reset value
    #define sfr_DAC_RDHRH_RESET_VALUE   ((uint8_t) 0x00)

  } RDHRH;


  /** DAC right aligned data holding register low (RDHRL at 0x5389) */
  union {

    /// bytewise access to RDHRL
    uint8_t  byte;

    /// bitwise access to register RDHRL
    struct {
      BITS   RDHRL               : 8;      // bits 0-7
    };  // RDHRL bitfield

    /// register _DAC_RDHRL reset value
    #define sfr_DAC_RDHRL_RESET_VALUE   ((uint8_t) 0x00)

  } RDHRL;


  /// Reserved register (2B)
  uint8_t     Reserved_3[2];


  /** DAC left aligned data holding register high (LDHRH at 0x538c) */
  union {

    /// bytewise access to LDHRH
    uint8_t  byte;

    /// bitwise access to register LDHRH
    struct {
      BITS   LDHRH               : 8;      // bits 0-7
    };  // LDHRH bitfield

    /// register _DAC_LDHRH reset value
    #define sfr_DAC_LDHRH_RESET_VALUE   ((uint8_t) 0x00)

  } LDHRH;


  /** DAC left aligned data holding register low (LDHRL at 0x538d) */
  union {

    /// bytewise access to LDHRL
    uint8_t  byte;

    /// bitwise access to register LDHRL
    struct {
      BITS                       : 4;      // 4 bits
      BITS   LDHRL               : 4;      // bits 4-7
    };  // LDHRL bitfield

    /// register _DAC_LDHRL reset value
    #define sfr_DAC_LDHRL_RESET_VALUE   ((uint8_t) 0x00)

  } LDHRL;


  /// Reserved register (2B)
  uint8_t     Reserved_4[2];


  /** DAC 8-bit data holding register (DHR8 at 0x5390) */
  union {

    /// bytewise access to DHR8
    uint8_t  byte;

    /// bitwise access to register DHR8
    struct {
      BITS   DHR8                : 8;      // bits 0-7
    };  // DHR8 bitfield

    /// register _DAC_DHR8 reset value
    #define sfr_DAC_DHR8_RESET_VALUE   ((uint8_t) 0x00)

  } DHR8;


  /// Reserved register (27B)
  uint8_t     Reserved_5[27];


  /** DAC data output register high (DORH at 0x53ac) */
  union {

    /// bytewise access to DORH
    uint8_t  byte;

    /// bitwise access to register DORH
    struct {
      BITS   DORH                : 4;      // bits 0-3
      BITS                       : 4;      // 4 bits
    };  // DORH bitfield

    /// register _DAC_DORH reset value
    #define sfr_DAC_DORH_RESET_VALUE   ((uint8_t) 0x00)

  } DORH;


  /** DAC data output register low (DORL at 0x53ad) */
  union {

    /// bytewise access to DORL
    uint8_t  byte;

    /// bitwise access to register DORL
    struct {
      BITS   DORL                : 8;      // bits 0-7
    };  // DORL bitfield

    /// register _DAC_DORL reset value
    #define sfr_DAC_DORL_RESET_VALUE   ((uint8_t) 0x00)

  } DORL;

} DAC_t;

/// access to DAC SFR registers
#define sfr_DAC   (*((DAC_t*) 0x5380))


//------------------------
// Module DM
//------------------------

/** struct containing DM module registers */
typedef struct {

  /** DM breakpoint 1 register extended byte (BK1RE at 0x7f90) */
  union {

    /// bytewise access to BK1RE
    uint8_t  byte;

    /// skip bitwise access to register BK1RE

    /// register _DM_BK1RE reset value
    #define sfr_DM_BK1RE_RESET_VALUE   ((uint8_t) 0xFF)

  } BK1RE;


  /** DM breakpoint 1 register high byte (BK1RH at 0x7f91) */
  union {

    /// bytewise access to BK1RH
    uint8_t  byte;

    /// skip bitwise access to register BK1RH

    /// register _DM_BK1RH reset value
    #define sfr_DM_BK1RH_RESET_VALUE   ((uint8_t) 0xFF)

  } BK1RH;


  /** DM breakpoint 1 register low byte (BK1RL at 0x7f92) */
  union {

    /// bytewise access to BK1RL
    uint8_t  byte;

    /// skip bitwise access to register BK1RL

    /// register _DM_BK1RL reset value
    #define sfr_DM_BK1RL_RESET_VALUE   ((uint8_t) 0xFF)

  } BK1RL;


  /** DM breakpoint 2 register extended byte (BK2RE at 0x7f93) */
  union {

    /// bytewise access to BK2RE
    uint8_t  byte;

    /// skip bitwise access to register BK2RE

    /// register _DM_BK2RE reset value
    #define sfr_DM_BK2RE_RESET_VALUE   ((uint8_t) 0xFF)

  } BK2RE;


  /** DM breakpoint 2 register high byte (BK2RH at 0x7f94) */
  union {

    /// bytewise access to BK2RH
    uint8_t  byte;

    /// skip bitwise access to register BK2RH

    /// register _DM_BK2RH reset value
    #define sfr_DM_BK2RH_RESET_VALUE   ((uint8_t) 0xFF)

  } BK2RH;


  /** DM breakpoint 2 register low byte (BK2RL at 0x7f95) */
  union {

    /// bytewise access to BK2RL
    uint8_t  byte;

    /// skip bitwise access to register BK2RL

    /// register _DM_BK2RL reset value
    #define sfr_DM_BK2RL_RESET_VALUE   ((uint8_t) 0xFF)

  } BK2RL;


  /** DM Debug module control register 1 (CR1 at 0x7f96) */
  union {

    /// bytewise access to CR1
    uint8_t  byte;

    /// skip bitwise access to register CR1

    /// register _DM_CR1 reset value
    #define sfr_DM_CR1_RESET_VALUE   ((uint8_t) 0x00)

  } CR1;


  /** DM Debug module control register 2 (CR2 at 0x7f97) */
  union {

    /// bytewise access to CR2
    uint8_t  byte;

    /// skip bitwise access to register CR2

    /// register _DM_CR2 reset value
    #define sfr_DM_CR2_RESET_VALUE   ((uint8_t) 0x00)

  } CR2;


  /** DM Debug module control/status register 1 (CSR1 at 0x7f98) */
  union {

    /// bytewise access to CSR1
    uint8_t  byte;

    /// skip bitwise access to register CSR1

    /// register _DM_CSR1 reset value
    #define sfr_DM_CSR1_RESET_VALUE   ((uint8_t) 0x10)

  } CSR1;


  /** DM Debug module control/status register 2 (CSR2 at 0x7f99) */
  union {

    /// bytewise access to CSR2
    uint8_t  byte;

    /// skip bitwise access to register CSR2

    /// register _DM_CSR2 reset value
    #define sfr_DM_CSR2_RESET_VALUE   ((uint8_t) 0x00)

  } CSR2;


  /** DM enable function register (ENFCTR at 0x7f9a) */
  union {

    /// bytewise access to ENFCTR
    uint8_t  byte;

    /// skip bitwise access to register ENFCTR

    /// register _DM_ENFCTR reset value
    #define sfr_DM_ENFCTR_RESET_VALUE   ((uint8_t) 0xFF)

  } ENFCTR;

} DM_t;

/// access to DM SFR registers
#define sfr_DM   (*((DM_t*) 0x7f90))


//------------------------
// Module DMA1
//------------------------

/** struct containing DMA1 module registers */
typedef struct {

  /** DMA1 global configuration & status register (GCSR at 0x5070) */
  union {

    /// bytewise access to GCSR
    uint8_t  byte;

    /// bitwise access to register GCSR
    struct {
      BITS   GEN                 : 1;      // bit 0
      BITS   GB                  : 1;      // bit 1
      BITS   TO                  : 6;      // bits 2-7
    };  // GCSR bitfield

    /// register _DMA1_GCSR reset value
    #define sfr_DMA1_GCSR_RESET_VALUE   ((uint8_t) 0xFC)

  } GCSR;


  /** DMA1 global interrupt register 1 (GIR1 at 0x5071) */
  union {

    /// bytewise access to GIR1
    uint8_t  byte;

    /// bitwise access to register GIR1
    struct {
      BITS   IFC0                : 1;      // bit 0
      BITS   IFC1                : 1;      // bit 1
      BITS   IFC2                : 1;      // bit 2
      BITS   IFC3                : 1;      // bit 3
      BITS                       : 4;      // 4 bits
    };  // GIR1 bitfield

    /// register _DMA1_GIR1 reset value
    #define sfr_DMA1_GIR1_RESET_VALUE   ((uint8_t) 0x00)

  } GIR1;


  /// Reserved register (3B)
  uint8_t     Reserved_1[3];


  /** DMA1 channel 0 configuration register (C0CR at 0x5075) */
  union {

    /// bytewise access to C0CR
    uint8_t  byte;

    /// bitwise access to register C0CR
    struct {
      BITS   EN                  : 1;      // bit 0
      BITS   TCIE                : 1;      // bit 1
      BITS   HTIE                : 1;      // bit 2
      BITS   DIR                 : 1;      // bit 3
      BITS   CIRC                : 1;      // bit 4
      BITS   MINCDEC             : 1;      // bit 5
      BITS                       : 2;      // 2 bits
    };  // C0CR bitfield

    /// register _DMA1_C0CR reset value
    #define sfr_DMA1_C0CR_RESET_VALUE   ((uint8_t) 0x00)

  } C0CR;


  /** DMA1 channel 0 status (C0SPR at 0x5076) */
  union {

    /// bytewise access to C0SPR
    uint8_t  byte;

    /// bitwise access to register C0SPR
    struct {
      BITS                       : 1;      // 1 bit
      BITS   TCIF                : 1;      // bit 1
      BITS   HTIF                : 1;      // bit 2
      BITS   TSIZE               : 1;      // bit 3
      BITS   PL0                 : 1;      // bit 4
      BITS   PL1                 : 1;      // bit 5
      BITS   PEND                : 1;      // bit 6
      BITS   BUSY                : 1;      // bit 7
    };  // C0SPR bitfield

    /// register _DMA1_C0SPR reset value
    #define sfr_DMA1_C0SPR_RESET_VALUE   ((uint8_t) 0x00)

  } C0SPR;


  /** DMA1 number of data to transfer register (channel 0) (C0NDTR at 0x5077) */
  union {

    /// bytewise access to C0NDTR
    uint8_t  byte;

    /// bitwise access to register C0NDTR
    struct {
      BITS   NDT0                : 1;      // bit 0
      BITS   NDT1                : 1;      // bit 1
      BITS   NDT2                : 1;      // bit 2
      BITS   NDT3                : 1;      // bit 3
      BITS   NDT4                : 1;      // bit 4
      BITS   NDT5                : 1;      // bit 5
      BITS   NDT6                : 1;      // bit 6
      BITS   NDT7                : 1;      // bit 7
    };  // C0NDTR bitfield

    /// register _DMA1_C0NDTR reset value
    #define sfr_DMA1_C0NDTR_RESET_VALUE   ((uint8_t) 0x00)

  } C0NDTR;


  /** DMA1 peripheral address high register (channel 0) (C0PARH at 0x5078) */
  union {

    /// bytewise access to C0PARH
    uint8_t  byte;

    /// bitwise access to register C0PARH
    struct {
      BITS   PA8                 : 1;      // bit 0
      BITS   PA9                 : 1;      // bit 1
      BITS   PA10                : 1;      // bit 2
      BITS   PA11                : 1;      // bit 3
      BITS   PA12                : 1;      // bit 4
      BITS   PA13                : 1;      // bit 5
      BITS   PA14                : 1;      // bit 6
      BITS   PA15                : 1;      // bit 7
    };  // C0PARH bitfield

    /// register _DMA1_C0PARH reset value
    #define sfr_DMA1_C0PARH_RESET_VALUE   ((uint8_t) 0x52)

  } C0PARH;


  /** DMA1 peripheral address low register (channel 0) (C0PARL at 0x5079) */
  union {

    /// bytewise access to C0PARL
    uint8_t  byte;

    /// bitwise access to register C0PARL
    struct {
      BITS   PA0                 : 1;      // bit 0
      BITS   PA1                 : 1;      // bit 1
      BITS   PA2                 : 1;      // bit 2
      BITS   PA3                 : 1;      // bit 3
      BITS   PA4                 : 1;      // bit 4
      BITS   PA5                 : 1;      // bit 5
      BITS   PA6                 : 1;      // bit 6
      BITS   PA7                 : 1;      // bit 7
    };  // C0PARL bitfield

    /// register _DMA1_C0PARL reset value
    #define sfr_DMA1_C0PARL_RESET_VALUE   ((uint8_t) 0x00)

  } C0PARL;


  /// Reserved register (1B)
  uint8_t     Reserved_2[1];


  /** DMA1 memory 0 address high register (channel 0) (C0M0ARH at 0x507b) */
  union {

    /// bytewise access to C0M0ARH
    uint8_t  byte;

    /// bitwise access to register C0M0ARH
    struct {
      BITS   M0A8                : 1;      // bit 0
      BITS   M0A9                : 1;      // bit 1
      BITS   M0A10               : 1;      // bit 2
      BITS   M0A11               : 1;      // bit 3
      BITS   M0A12               : 1;      // bit 4
      BITS   M0A13               : 1;      // bit 5
      BITS   M0A14               : 1;      // bit 6
      BITS   M0A15               : 1;      // bit 7
    };  // C0M0ARH bitfield

    /// register _DMA1_C0M0ARH reset value
    #define sfr_DMA1_C0M0ARH_RESET_VALUE   ((uint8_t) 0x00)

  } C0M0ARH;


  /** DMA1 memory 0 address low register (channel 0) (C0M0ARL at 0x507c) */
  union {

    /// bytewise access to C0M0ARL
    uint8_t  byte;

    /// bitwise access to register C0M0ARL
    struct {
      BITS   M0A0                : 1;      // bit 0
      BITS   M0A1                : 1;      // bit 1
      BITS   M0A2                : 1;      // bit 2
      BITS   M0A3                : 1;      // bit 3
      BITS   M0A4                : 1;      // bit 4
      BITS   M0A5                : 1;      // bit 5
      BITS   M0A6                : 1;      // bit 6
      BITS   M0A7                : 1;      // bit 7
    };  // C0M0ARL bitfield

    /// register _DMA1_C0M0ARL reset value
    #define sfr_DMA1_C0M0ARL_RESET_VALUE   ((uint8_t) 0x00)

  } C0M0ARL;


  /// Reserved register (2B)
  uint8_t     Reserved_3[2];


  /** DMA1 channel 1 configuration register (C1CR at 0x507f) */
  union {

    /// bytewise access to C1CR
    uint8_t  byte;

    /// bitwise access to register C1CR
    struct {
      BITS   EN                  : 1;      // bit 0
      BITS   TCIE                : 1;      // bit 1
      BITS   HTIE                : 1;      // bit 2
      BITS   DIR                 : 1;      // bit 3
      BITS   CIRC                : 1;      // bit 4
      BITS   MINCDEC             : 1;      // bit 5
      BITS                       : 2;      // 2 bits
    };  // C1CR bitfield

    /// register _DMA1_C1CR reset value
    #define sfr_DMA1_C1CR_RESET_VALUE   ((uint8_t) 0x00)

  } C1CR;


  /** DMA1 channel 1 status (C1SPR at 0x5080) */
  union {

    /// bytewise access to C1SPR
    uint8_t  byte;

    /// bitwise access to register C1SPR
    struct {
      BITS                       : 1;      // 1 bit
      BITS   TCIF                : 1;      // bit 1
      BITS   HTIF                : 1;      // bit 2
      BITS   TSIZE               : 1;      // bit 3
      BITS   PL0                 : 1;      // bit 4
      BITS   PL1                 : 1;      // bit 5
      BITS   PEND                : 1;      // bit 6
      BITS   BUSY                : 1;      // bit 7
    };  // C1SPR bitfield

    /// register _DMA1_C1SPR reset value
    #define sfr_DMA1_C1SPR_RESET_VALUE   ((uint8_t) 0x00)

  } C1SPR;


  /** DMA1 number of data to transfer register (channel 1) (C1NDTR at 0x5081) */
  union {

    /// bytewise access to C1NDTR
    uint8_t  byte;

    /// bitwise access to register C1NDTR
    struct {
      BITS   NDT0                : 1;      // bit 0
      BITS   NDT1                : 1;      // bit 1
      BITS   NDT2                : 1;      // bit 2
      BITS   NDT3                : 1;      // bit 3
      BITS   NDT4                : 1;      // bit 4
      BITS   NDT5                : 1;      // bit 5
      BITS   NDT6                : 1;      // bit 6
      BITS   NDT7                : 1;      // bit 7
    };  // C1NDTR bitfield

    /// register _DMA1_C1NDTR reset value
    #define sfr_DMA1_C1NDTR_RESET_VALUE   ((uint8_t) 0x00)

  } C1NDTR;


  /** DMA1 peripheral address high register (channel 1) (C1PARH at 0x5082) */
  union {

    /// bytewise access to C1PARH
    uint8_t  byte;

    /// bitwise access to register C1PARH
    struct {
      BITS   PA8                 : 1;      // bit 0
      BITS   PA9                 : 1;      // bit 1
      BITS   PA10                : 1;      // bit 2
      BITS   PA11                : 1;      // bit 3
      BITS   PA12                : 1;      // bit 4
      BITS   PA13                : 1;      // bit 5
      BITS   PA14                : 1;      // bit 6
      BITS   PA15                : 1;      // bit 7
    };  // C1PARH bitfield

    /// register _DMA1_C1PARH reset value
    #define sfr_DMA1_C1PARH_RESET_VALUE   ((uint8_t) 0x52)

  } C1PARH;


  /** DMA1 peripheral address low register (channel 1) (C1PARL at 0x5083) */
  union {

    /// bytewise access to C1PARL
    uint8_t  byte;

    /// bitwise access to register C1PARL
    struct {
      BITS   PA0                 : 1;      // bit 0
      BITS   PA1                 : 1;      // bit 1
      BITS   PA2                 : 1;      // bit 2
      BITS   PA3                 : 1;      // bit 3
      BITS   PA4                 : 1;      // bit 4
      BITS   PA5                 : 1;      // bit 5
      BITS   PA6                 : 1;      // bit 6
      BITS   PA7                 : 1;      // bit 7
    };  // C1PARL bitfield

    /// register _DMA1_C1PARL reset value
    #define sfr_DMA1_C1PARL_RESET_VALUE   ((uint8_t) 0x00)

  } C1PARL;


  /// Reserved register (1B)
  uint8_t     Reserved_4[1];


  /** DMA1 memory 0 address high register (channel 1) (C1M0ARH at 0x5085) */
  union {

    /// bytewise access to C1M0ARH
    uint8_t  byte;

    /// bitwise access to register C1M0ARH
    struct {
      BITS   M0A8                : 1;      // bit 0
      BITS   M0A9                : 1;      // bit 1
      BITS   M0A10               : 1;      // bit 2
      BITS   M0A11               : 1;      // bit 3
      BITS   M0A12               : 1;      // bit 4
      BITS   M0A13               : 1;      // bit 5
      BITS   M0A14               : 1;      // bit 6
      BITS   M0A15               : 1;      // bit 7
    };  // C1M0ARH bitfield

    /// register _DMA1_C1M0ARH reset value
    #define sfr_DMA1_C1M0ARH_RESET_VALUE   ((uint8_t) 0x00)

  } C1M0ARH;


  /** DMA1 memory 0 address low register (channel 1) (C1M0ARL at 0x5086) */
  union {

    /// bytewise access to C1M0ARL
    uint8_t  byte;

    /// bitwise access to register C1M0ARL
    struct {
      BITS   M0A0                : 1;      // bit 0
      BITS   M0A1                : 1;      // bit 1
      BITS   M0A2                : 1;      // bit 2
      BITS   M0A3                : 1;      // bit 3
      BITS   M0A4                : 1;      // bit 4
      BITS   M0A5                : 1;      // bit 5
      BITS   M0A6                : 1;      // bit 6
      BITS   M0A7                : 1;      // bit 7
    };  // C1M0ARL bitfield

    /// register _DMA1_C1M0ARL reset value
    #define sfr_DMA1_C1M0ARL_RESET_VALUE   ((uint8_t) 0x00)

  } C1M0ARL;


  /// Reserved register (2B)
  uint8_t     Reserved_5[2];


  /** DMA1 channel 2 configuration register (C2CR at 0x5089) */
  union {

    /// bytewise access to C2CR
    uint8_t  byte;

    /// bitwise access to register C2CR
    struct {
      BITS   EN                  : 1;      // bit 0
      BITS   TCIE                : 1;      // bit 1
      BITS   HTIE                : 1;      // bit 2
      BITS   DIR                 : 1;      // bit 3
      BITS   CIRC                : 1;      // bit 4
      BITS   MINCDEC             : 1;      // bit 5
      BITS                       : 2;      // 2 bits
    };  // C2CR bitfield

    /// register _DMA1_C2CR reset value
    #define sfr_DMA1_C2CR_RESET_VALUE   ((uint8_t) 0x00)

  } C2CR;


  /** DMA1 channel 2 status (C2SPR at 0x508a) */
  union {

    /// bytewise access to C2SPR
    uint8_t  byte;

    /// bitwise access to register C2SPR
    struct {
      BITS                       : 1;      // 1 bit
      BITS   TCIF                : 1;      // bit 1
      BITS   HTIF                : 1;      // bit 2
      BITS   TSIZE               : 1;      // bit 3
      BITS   PL0                 : 1;      // bit 4
      BITS   PL1                 : 1;      // bit 5
      BITS   PEND                : 1;      // bit 6
      BITS   BUSY                : 1;      // bit 7
    };  // C2SPR bitfield

    /// register _DMA1_C2SPR reset value
    #define sfr_DMA1_C2SPR_RESET_VALUE   ((uint8_t) 0x00)

  } C2SPR;


  /** DMA1 number of data to transfer register (channel 2) (C2NDTR at 0x508b) */
  union {

    /// bytewise access to C2NDTR
    uint8_t  byte;

    /// bitwise access to register C2NDTR
    struct {
      BITS   NDT0                : 1;      // bit 0
      BITS   NDT1                : 1;      // bit 1
      BITS   NDT2                : 1;      // bit 2
      BITS   NDT3                : 1;      // bit 3
      BITS   NDT4                : 1;      // bit 4
      BITS   NDT5                : 1;      // bit 5
      BITS   NDT6                : 1;      // bit 6
      BITS   NDT7                : 1;      // bit 7
    };  // C2NDTR bitfield

    /// register _DMA1_C2NDTR reset value
    #define sfr_DMA1_C2NDTR_RESET_VALUE   ((uint8_t) 0x00)

  } C2NDTR;


  /** DMA1 peripheral address high register (channel 2) (C2PARH at 0x508c) */
  union {

    /// bytewise access to C2PARH
    uint8_t  byte;

    /// bitwise access to register C2PARH
    struct {
      BITS   PA8                 : 1;      // bit 0
      BITS   PA9                 : 1;      // bit 1
      BITS   PA10                : 1;      // bit 2
      BITS   PA11                : 1;      // bit 3
      BITS   PA12                : 1;      // bit 4
      BITS   PA13                : 1;      // bit 5
      BITS   PA14                : 1;      // bit 6
      BITS   PA15                : 1;      // bit 7
    };  // C2PARH bitfield

    /// register _DMA1_C2PARH reset value
    #define sfr_DMA1_C2PARH_RESET_VALUE   ((uint8_t) 0x52)

  } C2PARH;


  /** DMA1 peripheral address low register (channel 2) (C2PARL at 0x508d) */
  union {

    /// bytewise access to C2PARL
    uint8_t  byte;

    /// bitwise access to register C2PARL
    struct {
      BITS   PA0                 : 1;      // bit 0
      BITS   PA1                 : 1;      // bit 1
      BITS   PA2                 : 1;      // bit 2
      BITS   PA3                 : 1;      // bit 3
      BITS   PA4                 : 1;      // bit 4
      BITS   PA5                 : 1;      // bit 5
      BITS   PA6                 : 1;      // bit 6
      BITS   PA7                 : 1;      // bit 7
    };  // C2PARL bitfield

    /// register _DMA1_C2PARL reset value
    #define sfr_DMA1_C2PARL_RESET_VALUE   ((uint8_t) 0x00)

  } C2PARL;


  /// Reserved register (1B)
  uint8_t     Reserved_6[1];


  /** DMA1 memory 0 address high register (channel 2) (C2M0ARH at 0x508f) */
  union {

    /// bytewise access to C2M0ARH
    uint8_t  byte;

    /// bitwise access to register C2M0ARH
    struct {
      BITS   M0A8                : 1;      // bit 0
      BITS   M0A9                : 1;      // bit 1
      BITS   M0A10               : 1;      // bit 2
      BITS   M0A11               : 1;      // bit 3
      BITS   M0A12               : 1;      // bit 4
      BITS   M0A13               : 1;      // bit 5
      BITS   M0A14               : 1;      // bit 6
      BITS   M0A15               : 1;      // bit 7
    };  // C2M0ARH bitfield

    /// register _DMA1_C2M0ARH reset value
    #define sfr_DMA1_C2M0ARH_RESET_VALUE   ((uint8_t) 0x00)

  } C2M0ARH;


  /** DMA1 memory 0 address low register (channel 2) (C2M0ARL at 0x5090) */
  union {

    /// bytewise access to C2M0ARL
    uint8_t  byte;

    /// bitwise access to register C2M0ARL
    struct {
      BITS   M0A0                : 1;      // bit 0
      BITS   M0A1                : 1;      // bit 1
      BITS   M0A2                : 1;      // bit 2
      BITS   M0A3                : 1;      // bit 3
      BITS   M0A4                : 1;      // bit 4
      BITS   M0A5                : 1;      // bit 5
      BITS   M0A6                : 1;      // bit 6
      BITS   M0A7                : 1;      // bit 7
    };  // C2M0ARL bitfield

    /// register _DMA1_C2M0ARL reset value
    #define sfr_DMA1_C2M0ARL_RESET_VALUE   ((uint8_t) 0x00)

  } C2M0ARL;


  /// Reserved register (2B)
  uint8_t     Reserved_7[2];


  /** DMA1 channel 3 configuration register (C3CR at 0x5093) */
  union {

    /// bytewise access to C3CR
    uint8_t  byte;

    /// bitwise access to register C3CR
    struct {
      BITS   EN                  : 1;      // bit 0
      BITS   TCIE                : 1;      // bit 1
      BITS   HTIE                : 1;      // bit 2
      BITS   DIR                 : 1;      // bit 3
      BITS   CIRC                : 1;      // bit 4
      BITS   MINCDEC             : 1;      // bit 5
      BITS   MEM                 : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // C3CR bitfield

    /// register _DMA1_C3CR reset value
    #define sfr_DMA1_C3CR_RESET_VALUE   ((uint8_t) 0x00)

  } C3CR;


  /** DMA1 channel 3 status (C3SPR at 0x5094) */
  union {

    /// bytewise access to C3SPR
    uint8_t  byte;

    /// bitwise access to register C3SPR
    struct {
      BITS                       : 1;      // 1 bit
      BITS   TCIF                : 1;      // bit 1
      BITS   HTIF                : 1;      // bit 2
      BITS   TSIZE               : 1;      // bit 3
      BITS   PL0                 : 1;      // bit 4
      BITS   PL1                 : 1;      // bit 5
      BITS   PEND                : 1;      // bit 6
      BITS   BUSY                : 1;      // bit 7
    };  // C3SPR bitfield

    /// register _DMA1_C3SPR reset value
    #define sfr_DMA1_C3SPR_RESET_VALUE   ((uint8_t) 0x00)

  } C3SPR;


  /** DMA1 number of data to transfer register (channel 3) (C3NDTR at 0x5095) */
  union {

    /// bytewise access to C3NDTR
    uint8_t  byte;

    /// bitwise access to register C3NDTR
    struct {
      BITS   NDT0                : 1;      // bit 0
      BITS   NDT1                : 1;      // bit 1
      BITS   NDT2                : 1;      // bit 2
      BITS   NDT3                : 1;      // bit 3
      BITS   NDT4                : 1;      // bit 4
      BITS   NDT5                : 1;      // bit 5
      BITS   NDT6                : 1;      // bit 6
      BITS   NDT7                : 1;      // bit 7
    };  // C3NDTR bitfield

    /// register _DMA1_C3NDTR reset value
    #define sfr_DMA1_C3NDTR_RESET_VALUE   ((uint8_t) 0x00)

  } C3NDTR;


  /** DMA1 peripheral address high register (channel 3) (C3PARH_C3M1ARH at 0x5096) */
  union {

    /// bytewise access to C3PARH_C3M1ARH
    uint8_t  byte;

    /// bitwise access to register C3PARH
    struct {
      BITS   PA8                 : 1;      // bit 0
      BITS   PA9                 : 1;      // bit 1
      BITS   PA10                : 1;      // bit 2
      BITS   PA11                : 1;      // bit 3
      BITS   PA12                : 1;      // bit 4
      BITS   PA13                : 1;      // bit 5
      BITS   PA14                : 1;      // bit 6
      BITS   PA15                : 1;      // bit 7
    };  // C3PARH bitfield

    /// register _DMA1_C3PARH reset value
    #define sfr_DMA1_C3PARH_RESET_VALUE   ((uint8_t) 0x40)


    /// bitwise access to register C3M1ARH
    struct {
      BITS   M1A8                : 1;      // bit 0
      BITS   M1A9                : 1;      // bit 1
      BITS   M1A10               : 1;      // bit 2
      BITS   M1A11               : 1;      // bit 3
      BITS   M1A12               : 1;      // bit 4
      BITS   M1A13               : 1;      // bit 5
      BITS   M1A14               : 1;      // bit 6
      BITS   M1A15               : 1;      // bit 7
    };  // C3M1ARH bitfield

    /// register _DMA1_C3M1ARH reset value
    #define sfr_DMA1_C3M1ARH_RESET_VALUE   ((uint8_t) 0x40)

  } C3PARH_C3M1ARH;


  /** DMA1 peripheral address low register (channel 3) (C3PARL_C3M1ARL at 0x5097) */
  union {

    /// bytewise access to C3PARL_C3M1ARL
    uint8_t  byte;

    /// bitwise access to register C3PARL
    struct {
      BITS   PA0                 : 1;      // bit 0
      BITS   PA1                 : 1;      // bit 1
      BITS   PA2                 : 1;      // bit 2
      BITS   PA3                 : 1;      // bit 3
      BITS   PA4                 : 1;      // bit 4
      BITS   PA5                 : 1;      // bit 5
      BITS   PA6                 : 1;      // bit 6
      BITS   PA7                 : 1;      // bit 7
    };  // C3PARL bitfield

    /// register _DMA1_C3PARL reset value
    #define sfr_DMA1_C3PARL_RESET_VALUE   ((uint8_t) 0x00)


    /// bitwise access to register C3M1ARL
    struct {
      BITS   M1A0                : 1;      // bit 0
      BITS   M1A1                : 1;      // bit 1
      BITS   M1A2                : 1;      // bit 2
      BITS   M1A3                : 1;      // bit 3
      BITS   M1A4                : 1;      // bit 4
      BITS   M1A5                : 1;      // bit 5
      BITS   M1A6                : 1;      // bit 6
      BITS   M1A7                : 1;      // bit 7
    };  // C3M1ARL bitfield

    /// register _DMA1_C3M1ARL reset value
    #define sfr_DMA1_C3M1ARL_RESET_VALUE   ((uint8_t) 0x00)

  } C3PARL_C3M1ARL;


  /// Reserved register (1B)
  uint8_t     Reserved_8[1];


  /** DMA1 memory 0 address high register (channel 3) (C3M0ARH at 0x5099) */
  union {

    /// bytewise access to C3M0ARH
    uint8_t  byte;

    /// bitwise access to register C3M0ARH
    struct {
      BITS   M0A8                : 1;      // bit 0
      BITS   M0A9                : 1;      // bit 1
      BITS   M0A10               : 1;      // bit 2
      BITS   M0A11               : 1;      // bit 3
      BITS   M0A12               : 1;      // bit 4
      BITS   M0A13               : 1;      // bit 5
      BITS   M0A14               : 1;      // bit 6
      BITS   M0A15               : 1;      // bit 7
    };  // C3M0ARH bitfield

    /// register _DMA1_C3M0ARH reset value
    #define sfr_DMA1_C3M0ARH_RESET_VALUE   ((uint8_t) 0x00)

  } C3M0ARH;


  /** DMA1 memory 0 address low register (channel 3) (C3M0ARL at 0x509a) */
  union {

    /// bytewise access to C3M0ARL
    uint8_t  byte;

    /// bitwise access to register C3M0ARL
    struct {
      BITS   M0A0                : 1;      // bit 0
      BITS   M0A1                : 1;      // bit 1
      BITS   M0A2                : 1;      // bit 2
      BITS   M0A3                : 1;      // bit 3
      BITS   M0A4                : 1;      // bit 4
      BITS   M0A5                : 1;      // bit 5
      BITS   M0A6                : 1;      // bit 6
      BITS   M0A7                : 1;      // bit 7
    };  // C3M0ARL bitfield

    /// register _DMA1_C3M0ARL reset value
    #define sfr_DMA1_C3M0ARL_RESET_VALUE   ((uint8_t) 0x00)

  } C3M0ARL;

} DMA1_t;

/// access to DMA1 SFR registers
#define sfr_DMA1   (*((DMA1_t*) 0x5070))


//------------------------
// Module FLASH
//------------------------

/** struct containing FLASH module registers */
typedef struct {

  /** Flash control register 1 (CR1 at 0x5050) */
  union {

    /// bytewise access to CR1
    uint8_t  byte;

    /// bitwise access to register CR1
    struct {
      BITS   FIX                 : 1;      // bit 0
      BITS   IE                  : 1;      // bit 1
      BITS   WAITM               : 1;      // bit 2
      BITS   EEPM                : 1;      // bit 3
      BITS                       : 4;      // 4 bits
    };  // CR1 bitfield

    /// register _FLASH_CR1 reset value
    #define sfr_FLASH_CR1_RESET_VALUE   ((uint8_t) 0x00)

  } CR1;


  /** Flash control register 2 (CR2 at 0x5051) */
  union {

    /// bytewise access to CR2
    uint8_t  byte;

    /// bitwise access to register CR2
    struct {
      BITS   PRG                 : 1;      // bit 0
      BITS                       : 3;      // 3 bits
      BITS   FPRG                : 1;      // bit 4
      BITS   ERASE               : 1;      // bit 5
      BITS   WPRG                : 1;      // bit 6
      BITS   OPT                 : 1;      // bit 7
    };  // CR2 bitfield

    /// register _FLASH_CR2 reset value
    #define sfr_FLASH_CR2_RESET_VALUE   ((uint8_t) 0x00)

  } CR2;


  /** Flash program memory unprotection key register (PUKR at 0x5052) */
  union {

    /// bytewise access to PUKR
    uint8_t  byte;

    /// bitwise access to register PUKR
    struct {
      BITS   MASS_PRG            : 8;      // bits 0-7
    };  // PUKR bitfield

    /// register _FLASH_PUKR reset value
    #define sfr_FLASH_PUKR_RESET_VALUE   ((uint8_t) 0x00)

  } PUKR;


  /** Data EEPROM unprotection key register (DUKR at 0x5053) */
  union {

    /// bytewise access to DUKR
    uint8_t  byte;

    /// bitwise access to register DUKR
    struct {
      BITS   MASS_DATA           : 8;      // bits 0-7
    };  // DUKR bitfield

    /// register _FLASH_DUKR reset value
    #define sfr_FLASH_DUKR_RESET_VALUE   ((uint8_t) 0x00)

  } DUKR;


  /** Flash in-application programming status register (IAPSR at 0x5054) */
  union {

    /// bytewise access to IAPSR
    uint8_t  byte;

    /// bitwise access to register IAPSR
    struct {
      BITS   WR_PG_DIS           : 1;      // bit 0
      BITS   PUL                 : 1;      // bit 1
      BITS   EOP                 : 1;      // bit 2
      BITS   DUL                 : 1;      // bit 3
      BITS                       : 2;      // 2 bits
      BITS   HVOFF               : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // IAPSR bitfield

    /// register _FLASH_IAPSR reset value
    #define sfr_FLASH_IAPSR_RESET_VALUE   ((uint8_t) 0x00)

  } IAPSR;

} FLASH_t;

/// access to FLASH SFR registers
#define sfr_FLASH   (*((FLASH_t*) 0x5050))


//------------------------
// Module I2C1
//------------------------

/** struct containing I2C1 module registers */
typedef struct {

  /** I2C1 control register 1 (CR1 at 0x5210) */
  union {

    /// bytewise access to CR1
    uint8_t  byte;

    /// bitwise access to register CR1
    struct {
      BITS   PE                  : 1;      // bit 0
      BITS   SMBUS               : 1;      // bit 1
      BITS                       : 1;      // 1 bit
      BITS   SMBTYPE             : 1;      // bit 3
      BITS   ENARP               : 1;      // bit 4
      BITS   ENPEC               : 1;      // bit 5
      BITS   ENGC                : 1;      // bit 6
      BITS   NOSTRETCH           : 1;      // bit 7
    };  // CR1 bitfield

    /// register _I2C1_CR1 reset value
    #define sfr_I2C1_CR1_RESET_VALUE   ((uint8_t) 0x00)

  } CR1;


  /** I2C1 control register 2 (CR2 at 0x5211) */
  union {

    /// bytewise access to CR2
    uint8_t  byte;

    /// bitwise access to register CR2
    struct {
      BITS   START               : 1;      // bit 0
      BITS   STOP                : 1;      // bit 1
      BITS   ACK                 : 1;      // bit 2
      BITS   POS                 : 1;      // bit 3
      BITS   PEC                 : 1;      // bit 4
      BITS   ALERT               : 1;      // bit 5
      BITS                       : 1;      // 1 bit
      BITS   SWRST               : 1;      // bit 7
    };  // CR2 bitfield

    /// register _I2C1_CR2 reset value
    #define sfr_I2C1_CR2_RESET_VALUE   ((uint8_t) 0x00)

  } CR2;


  /** I2C1 frequency register (FREQR at 0x5212) */
  union {

    /// bytewise access to FREQR
    uint8_t  byte;

    /// bitwise access to register FREQR
    struct {
      BITS   FREQ                : 6;      // bits 0-5
      BITS                       : 2;      // 2 bits
    };  // FREQR bitfield

    /// register _I2C1_FREQR reset value
    #define sfr_I2C1_FREQR_RESET_VALUE   ((uint8_t) 0x00)

  } FREQR;


  /** I2C1 own address register low (OARL at 0x5213) */
  union {

    /// bytewise access to OARL
    uint8_t  byte;

    /// bitwise access to register OARL
    struct {
      BITS   ADD0                : 1;      // bit 0
      BITS   ADD1                : 1;      // bit 1
      BITS   ADD2                : 1;      // bit 2
      BITS   ADD3                : 1;      // bit 3
      BITS   ADD4                : 1;      // bit 4
      BITS   ADD5                : 1;      // bit 5
      BITS   ADD6                : 1;      // bit 6
      BITS   ADD7                : 1;      // bit 7
    };  // OARL bitfield

    /// register _I2C1_OARL reset value
    #define sfr_I2C1_OARL_RESET_VALUE   ((uint8_t) 0x00)

  } OARL;


  /** I2C1 own address register high (OARH at 0x5214) */
  union {

    /// bytewise access to OARH
    uint8_t  byte;

    /// bitwise access to register OARH
    struct {
      BITS                       : 1;      // 1 bit
      BITS   ADD8                : 1;      // bit 1
      BITS   ADD9                : 1;      // bit 2
      BITS                       : 3;      // 3 bits
      BITS   ADDCONF             : 1;      // bit 6
      BITS   ADDMODE             : 1;      // bit 7
    };  // OARH bitfield

    /// register _I2C1_OARH reset value
    #define sfr_I2C1_OARH_RESET_VALUE   ((uint8_t) 0x00)

  } OARH;


  /// Reserved register (1B)
  uint8_t     Reserved_1[1];


  /** I2C1 data register (DR at 0x5216) */
  union {

    /// bytewise access to DR
    uint8_t  byte;

    /// bitwise access to register DR
    struct {
      BITS   DR                  : 8;      // bits 0-7
    };  // DR bitfield

    /// register _I2C1_DR reset value
    #define sfr_I2C1_DR_RESET_VALUE   ((uint8_t) 0x00)

  } DR;


  /** I2C1 status register 1 (SR1 at 0x5217) */
  union {

    /// bytewise access to SR1
    uint8_t  byte;

    /// bitwise access to register SR1
    struct {
      BITS   SB                  : 1;      // bit 0
      BITS   ADDR                : 1;      // bit 1
      BITS   BTF                 : 1;      // bit 2
      BITS   ADD10               : 1;      // bit 3
      BITS   STOPF               : 1;      // bit 4
      BITS                       : 1;      // 1 bit
      BITS   RXNE                : 1;      // bit 6
      BITS   TXE                 : 1;      // bit 7
    };  // SR1 bitfield

    /// register _I2C1_SR1 reset value
    #define sfr_I2C1_SR1_RESET_VALUE   ((uint8_t) 0x00)

  } SR1;


  /** I2C1 status register 2 (SR2 at 0x5218) */
  union {

    /// bytewise access to SR2
    uint8_t  byte;

    /// bitwise access to register SR2
    struct {
      BITS   BERR                : 1;      // bit 0
      BITS   ARLO                : 1;      // bit 1
      BITS   AF                  : 1;      // bit 2
      BITS   OVR                 : 1;      // bit 3
      BITS   PECERR              : 1;      // bit 4
      BITS   WUFH                : 1;      // bit 5
      BITS   TIMEOUT             : 1;      // bit 6
      BITS   SMBALERT            : 1;      // bit 7
    };  // SR2 bitfield

    /// register _I2C1_SR2 reset value
    #define sfr_I2C1_SR2_RESET_VALUE   ((uint8_t) 0x00)

  } SR2;


  /** I2C1 status register 3 (SR3 at 0x5219) */
  union {

    /// bytewise access to SR3
    uint8_t  byte;

    /// bitwise access to register SR3
    struct {
      BITS   MSL                 : 1;      // bit 0
      BITS   BUSY                : 1;      // bit 1
      BITS   TRA                 : 1;      // bit 2
      BITS                       : 1;      // 1 bit
      BITS   GENCALL             : 1;      // bit 4
      BITS   SMBDEFAULT          : 1;      // bit 5
      BITS   SMBHOST             : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // SR3 bitfield

    /// register _I2C1_SR3 reset value
    #define sfr_I2C1_SR3_RESET_VALUE   ((uint8_t) 0x00)

  } SR3;


  /** I2C1 interrupt control register (ITR at 0x521a) */
  union {

    /// bytewise access to ITR
    uint8_t  byte;

    /// bitwise access to register ITR
    struct {
      BITS   ITERREN             : 1;      // bit 0
      BITS   ITEVTEN             : 1;      // bit 1
      BITS   ITBUFEN             : 1;      // bit 2
      BITS   DMAEN               : 1;      // bit 3
      BITS   LAST                : 1;      // bit 4
      BITS                       : 3;      // 3 bits
    };  // ITR bitfield

    /// register _I2C1_ITR reset value
    #define sfr_I2C1_ITR_RESET_VALUE   ((uint8_t) 0x00)

  } ITR;


  /** I2C1 clock control register low (CCRL at 0x521b) */
  union {

    /// bytewise access to CCRL
    uint8_t  byte;

    /// bitwise access to register CCRL
    struct {
      BITS   CCR0                : 1;      // bit 0
      BITS   CCR1                : 1;      // bit 1
      BITS   CCR2                : 1;      // bit 2
      BITS   CCR3                : 1;      // bit 3
      BITS   CCR4                : 1;      // bit 4
      BITS   CCR5                : 1;      // bit 5
      BITS   CCR6                : 1;      // bit 6
      BITS   CCR7                : 1;      // bit 7
    };  // CCRL bitfield

    /// register _I2C1_CCRL reset value
    #define sfr_I2C1_CCRL_RESET_VALUE   ((uint8_t) 0x00)

  } CCRL;


  /** I2C1 clock control register high (CCRH at 0x521c) */
  union {

    /// bytewise access to CCRH
    uint8_t  byte;

    /// bitwise access to register CCRH
    struct {
      BITS   CCR8                : 1;      // bit 0
      BITS   CCR9                : 1;      // bit 1
      BITS   CCR10               : 1;      // bit 2
      BITS   CCR11               : 1;      // bit 3
      BITS                       : 2;      // 2 bits
      BITS   DUTY                : 1;      // bit 6
      BITS   F_S                 : 1;      // bit 7
    };  // CCRH bitfield

    /// register _I2C1_CCRH reset value
    #define sfr_I2C1_CCRH_RESET_VALUE   ((uint8_t) 0x00)

  } CCRH;


  /** I2C1 TRISE register (TRISER at 0x521d) */
  union {

    /// bytewise access to TRISER
    uint8_t  byte;

    /// bitwise access to register TRISER
    struct {
      BITS   TRISE               : 6;      // bits 0-5
      BITS                       : 2;      // 2 bits
    };  // TRISER bitfield

    /// register _I2C1_TRISER reset value
    #define sfr_I2C1_TRISER_RESET_VALUE   ((uint8_t) 0x02)

  } TRISER;


  /** I2C1 packet error checking register (PECR at 0x521e) */
  union {

    /// bytewise access to PECR
    uint8_t  byte;

    /// bitwise access to register PECR
    struct {
      BITS   PEC                 : 8;      // bits 0-7
    };  // PECR bitfield

    /// register _I2C1_PECR reset value
    #define sfr_I2C1_PECR_RESET_VALUE   ((uint8_t) 0x00)

  } PECR;

} I2C1_t;

/// access to I2C1 SFR registers
#define sfr_I2C1   (*((I2C1_t*) 0x5210))


//------------------------
// Module IRTIM
//------------------------

/** struct containing IRTIM module registers */
typedef struct {

  /** Infra-red control register (CR at 0x52ff) */
  union {

    /// bytewise access to CR
    uint8_t  byte;

    /// bitwise access to register CR
    struct {
      BITS   IR_EN               : 1;      // bit 0
      BITS   HS_EN               : 1;      // bit 1
      BITS                       : 6;      // 6 bits
    };  // CR bitfield

    /// register _IRTIM_CR reset value
    #define sfr_IRTIM_CR_RESET_VALUE   ((uint8_t) 0x00)

  } CR;

} IRTIM_t;

/// access to IRTIM SFR registers
#define sfr_IRTIM   (*((IRTIM_t*) 0x52ff))


//------------------------
// Module ITC_EXTI
//------------------------

/** struct containing ITC_EXTI module registers */
typedef struct {

  /** External interrupt control register 1 (CR1 at 0x50a0) */
  union {

    /// bytewise access to CR1
    uint8_t  byte;

    /// bitwise access to register CR1
    struct {
      BITS   P0IS                : 2;      // bits 0-1
      BITS   P1IS                : 2;      // bits 2-3
      BITS   P2IS                : 2;      // bits 4-5
      BITS   P3IS                : 2;      // bits 6-7
    };  // CR1 bitfield

    /// register _ITC_EXTI_CR1 reset value
    #define sfr_ITC_EXTI_CR1_RESET_VALUE   ((uint8_t) 0x00)

  } CR1;


  /** External interrupt control register 2 (CR2 at 0x50a1) */
  union {

    /// bytewise access to CR2
    uint8_t  byte;

    /// bitwise access to register CR2
    struct {
      BITS   P4IS                : 2;      // bits 0-1
      BITS   P5IS                : 2;      // bits 2-3
      BITS   P6IS                : 2;      // bits 4-5
      BITS   P7IS                : 2;      // bits 6-7
    };  // CR2 bitfield

    /// register _ITC_EXTI_CR2 reset value
    #define sfr_ITC_EXTI_CR2_RESET_VALUE   ((uint8_t) 0x00)

  } CR2;


  /** External interrupt control register 3 (CR3 at 0x50a2) */
  union {

    /// bytewise access to CR3
    uint8_t  byte;

    /// bitwise access to register CR3
    struct {
      BITS   PBIS                : 2;      // bits 0-1
      BITS   PDIS                : 2;      // bits 2-3
      BITS   PEIS                : 2;      // bits 4-5
      BITS   PFIS                : 2;      // bits 6-7
    };  // CR3 bitfield

    /// register _ITC_EXTI_CR3 reset value
    #define sfr_ITC_EXTI_CR3_RESET_VALUE   ((uint8_t) 0x00)

  } CR3;


  /** External interrupt status register 1 (SR1 at 0x50a3) */
  union {

    /// bytewise access to SR1
    uint8_t  byte;

    /// bitwise access to register SR1
    struct {
      BITS   P0F                 : 1;      // bit 0
      BITS   P1F                 : 1;      // bit 1
      BITS   P2F                 : 1;      // bit 2
      BITS   P3F                 : 1;      // bit 3
      BITS   P4F                 : 1;      // bit 4
      BITS   P5F                 : 1;      // bit 5
      BITS   P6F                 : 1;      // bit 6
      BITS   P7F                 : 1;      // bit 7
    };  // SR1 bitfield

    /// register _ITC_EXTI_SR1 reset value
    #define sfr_ITC_EXTI_SR1_RESET_VALUE   ((uint8_t) 0x00)

  } SR1;


  /** External interrupt status register 2 (SR2 at 0x50a4) */
  union {

    /// bytewise access to SR2
    uint8_t  byte;

    /// bitwise access to register SR2
    struct {
      BITS   PBF                 : 1;      // bit 0
      BITS   PDF                 : 1;      // bit 1
      BITS   PEF                 : 1;      // bit 2
      BITS   PFF                 : 1;      // bit 3
      BITS                       : 4;      // 4 bits
    };  // SR2 bitfield

    /// register _ITC_EXTI_SR2 reset value
    #define sfr_ITC_EXTI_SR2_RESET_VALUE   ((uint8_t) 0x00)

  } SR2;


  /** External interrupt port select register (CONF at 0x50a5) */
  union {

    /// bytewise access to CONF
    uint8_t  byte;

    /// bitwise access to register CONF
    struct {
      BITS   PBLIS               : 1;      // bit 0
      BITS   PBHIS               : 1;      // bit 1
      BITS   PDLIS               : 1;      // bit 2
      BITS   PDHIS               : 1;      // bit 3
      BITS   PELIS               : 1;      // bit 4
      BITS   PEHIS               : 1;      // bit 5
      BITS   PFLIS               : 1;      // bit 6
      BITS   PFES                : 1;      // bit 7
    };  // CONF bitfield

    /// register _ITC_EXTI_CONF reset value
    #define sfr_ITC_EXTI_CONF_RESET_VALUE   ((uint8_t) 0x00)

  } CONF;

} ITC_EXTI_t;

/// access to ITC_EXTI SFR registers
#define sfr_ITC_EXTI   (*((ITC_EXTI_t*) 0x50a0))


//------------------------
// Module ITC_SPR
//------------------------

/** struct containing ITC_SPR module registers */
typedef struct {

  /** Interrupt Software priority register 1 (SPR1 at 0x7f70) */
  union {

    /// bytewise access to SPR1
    uint8_t  byte;

    /// bitwise access to register SPR1
    struct {
      BITS   VECT0SPR            : 2;      // bits 0-1
      BITS   VECT1SPR            : 2;      // bits 2-3
      BITS   VECT2SPR            : 2;      // bits 4-5
      BITS   VECT3SPR            : 2;      // bits 6-7
    };  // SPR1 bitfield

    /// register _ITC_SPR_SPR1 reset value
    #define sfr_ITC_SPR_SPR1_RESET_VALUE   ((uint8_t) 0xFF)

  } SPR1;


  /** Interrupt Software priority register 2 (SPR2 at 0x7f71) */
  union {

    /// bytewise access to SPR2
    uint8_t  byte;

    /// bitwise access to register SPR2
    struct {
      BITS   VECT4SPR            : 2;      // bits 0-1
      BITS   VECT5SPR            : 2;      // bits 2-3
      BITS   VECT6SPR            : 2;      // bits 4-5
      BITS   VECT7SPR            : 2;      // bits 6-7
    };  // SPR2 bitfield

    /// register _ITC_SPR_SPR2 reset value
    #define sfr_ITC_SPR_SPR2_RESET_VALUE   ((uint8_t) 0xFF)

  } SPR2;


  /** Interrupt Software priority register 3 (SPR3 at 0x7f72) */
  union {

    /// bytewise access to SPR3
    uint8_t  byte;

    /// bitwise access to register SPR3
    struct {
      BITS   VECT8SPR            : 2;      // bits 0-1
      BITS   VECT9SPR            : 2;      // bits 2-3
      BITS   VECT10SPR           : 2;      // bits 4-5
      BITS   VECT11SPR           : 2;      // bits 6-7
    };  // SPR3 bitfield

    /// register _ITC_SPR_SPR3 reset value
    #define sfr_ITC_SPR_SPR3_RESET_VALUE   ((uint8_t) 0xFF)

  } SPR3;


  /** Interrupt Software priority register 4 (SPR4 at 0x7f73) */
  union {

    /// bytewise access to SPR4
    uint8_t  byte;

    /// bitwise access to register SPR4
    struct {
      BITS   VECT12SPR           : 2;      // bits 0-1
      BITS   VECT13SPR           : 2;      // bits 2-3
      BITS   VECT14SPR           : 2;      // bits 4-5
      BITS   VECT15SPR           : 2;      // bits 6-7
    };  // SPR4 bitfield

    /// register _ITC_SPR_SPR4 reset value
    #define sfr_ITC_SPR_SPR4_RESET_VALUE   ((uint8_t) 0xFF)

  } SPR4;


  /** Interrupt Software priority register 5 (SPR5 at 0x7f74) */
  union {

    /// bytewise access to SPR5
    uint8_t  byte;

    /// bitwise access to register SPR5
    struct {
      BITS   VECT16SPR           : 2;      // bits 0-1
      BITS   VECT17SPR           : 2;      // bits 2-3
      BITS   VECT18SPR           : 2;      // bits 4-5
      BITS   VECT19SPR           : 2;      // bits 6-7
    };  // SPR5 bitfield

    /// register _ITC_SPR_SPR5 reset value
    #define sfr_ITC_SPR_SPR5_RESET_VALUE   ((uint8_t) 0xFF)

  } SPR5;


  /** Interrupt Software priority register 6 (SPR6 at 0x7f75) */
  union {

    /// bytewise access to SPR6
    uint8_t  byte;

    /// bitwise access to register SPR6
    struct {
      BITS   VECT20SPR           : 2;      // bits 0-1
      BITS   VECT21SPR           : 2;      // bits 2-3
      BITS   VECT22SPR           : 2;      // bits 4-5
      BITS   VECT23SPR           : 2;      // bits 6-7
    };  // SPR6 bitfield

    /// register _ITC_SPR_SPR6 reset value
    #define sfr_ITC_SPR_SPR6_RESET_VALUE   ((uint8_t) 0xFF)

  } SPR6;


  /** Interrupt Software priority register 7 (SPR7 at 0x7f76) */
  union {

    /// bytewise access to SPR7
    uint8_t  byte;

    /// bitwise access to register SPR7
    struct {
      BITS   VECT24SPR           : 2;      // bits 0-1
      BITS   VECT25SPR           : 2;      // bits 2-3
      BITS   VECT26SPR           : 2;      // bits 4-5
      BITS   VECT27SPR           : 2;      // bits 6-7
    };  // SPR7 bitfield

    /// register _ITC_SPR_SPR7 reset value
    #define sfr_ITC_SPR_SPR7_RESET_VALUE   ((uint8_t) 0xFF)

  } SPR7;


  /** Interrupt Software priority register 8 (SPR8 at 0x7f77) */
  union {

    /// bytewise access to SPR8
    uint8_t  byte;

    /// bitwise access to register SPR8
    struct {
      BITS   VECT28SPR           : 2;      // bits 0-1
      BITS   VECT29SPR           : 2;      // bits 2-3
      BITS                       : 4;      // 4 bits
    };  // SPR8 bitfield

    /// register _ITC_SPR_SPR8 reset value
    #define sfr_ITC_SPR_SPR8_RESET_VALUE   ((uint8_t) 0xFF)

  } SPR8;

} ITC_SPR_t;

/// access to ITC_SPR SFR registers
#define sfr_ITC_SPR   (*((ITC_SPR_t*) 0x7f70))


//------------------------
// Module IWDG
//------------------------

/** struct containing IWDG module registers */
typedef struct {

  /** IWDG key register (KR at 0x50e0) */
  union {

    /// bytewise access to KR
    uint8_t  byte;

    /// bitwise access to register KR
    struct {
      BITS   KEY0                : 1;      // bit 0
      BITS   KEY1                : 1;      // bit 1
      BITS   KEY2                : 1;      // bit 2
      BITS   KEY3                : 1;      // bit 3
      BITS   KEY4                : 1;      // bit 4
      BITS   KEY5                : 1;      // bit 5
      BITS   KEY6                : 1;      // bit 6
      BITS   KEY7                : 1;      // bit 7
    };  // KR bitfield

    /// register _IWDG_KR reset value
    #define sfr_IWDG_KR_RESET_VALUE   ((uint8_t) 0x00)

  } KR;


  /** IWDG prescaler register (PR at 0x50e1) */
  union {

    /// bytewise access to PR
    uint8_t  byte;

    /// bitwise access to register PR
    struct {
      BITS   PR                  : 3;      // bits 0-2
      BITS                       : 5;      // 5 bits
    };  // PR bitfield

    /// register _IWDG_PR reset value
    #define sfr_IWDG_PR_RESET_VALUE   ((uint8_t) 0x00)

  } PR;


  /** IWDG reload register (RLR at 0x50e2) */
  union {

    /// bytewise access to RLR
    uint8_t  byte;

    /// bitwise access to register RLR
    struct {
      BITS   RL                  : 8;      // bits 0-7
    };  // RLR bitfield

    /// register _IWDG_RLR reset value
    #define sfr_IWDG_RLR_RESET_VALUE   ((uint8_t) 0xFF)

  } RLR;

} IWDG_t;

/// access to IWDG SFR registers
#define sfr_IWDG   (*((IWDG_t*) 0x50e0))


//------------------------
// Module LCD
//------------------------

/** struct containing LCD module registers */
typedef struct {

  /** LCD control register 1 (CR1 at 0x5400) */
  union {

    /// bytewise access to CR1
    uint8_t  byte;

    /// bitwise access to register CR1
    struct {
      BITS   B2                  : 1;      // bit 0
      BITS   DUTY0               : 1;      // bit 1
      BITS   DUTY1               : 1;      // bit 2
      BITS   BLINKF0             : 1;      // bit 3
      BITS   BLINKF1             : 1;      // bit 4
      BITS   BLINKF2             : 1;      // bit 5
      BITS   BLINK0              : 1;      // bit 6
      BITS   BLINK1              : 1;      // bit 7
    };  // CR1 bitfield

    /// register _LCD_CR1 reset value
    #define sfr_LCD_CR1_RESET_VALUE   ((uint8_t) 0x00)

  } CR1;


  /** LCD control register 2 (CR2 at 0x5401) */
  union {

    /// bytewise access to CR2
    uint8_t  byte;

    /// bitwise access to register CR2
    struct {
      BITS   VSEL                : 1;      // bit 0
      BITS   CC0                 : 1;      // bit 1
      BITS   CC1                 : 1;      // bit 2
      BITS   CC2                 : 1;      // bit 3
      BITS   HD                  : 1;      // bit 4
      BITS   PON0                : 1;      // bit 5
      BITS   PON1                : 1;      // bit 6
      BITS   PON2                : 1;      // bit 7
    };  // CR2 bitfield

    /// register _LCD_CR2 reset value
    #define sfr_LCD_CR2_RESET_VALUE   ((uint8_t) 0x00)

  } CR2;


  /** LCD control register 3 (CR3 at 0x5402) */
  union {

    /// bytewise access to CR3
    uint8_t  byte;

    /// bitwise access to register CR3
    struct {
      BITS   DEAD0               : 1;      // bit 0
      BITS   DEAD1               : 1;      // bit 1
      BITS   DEAD2               : 1;      // bit 2
      BITS   SOFC                : 1;      // bit 3
      BITS   SOF                 : 1;      // bit 4
      BITS   SOFIE               : 1;      // bit 5
      BITS   LCDEN               : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // CR3 bitfield

    /// register _LCD_CR3 reset value
    #define sfr_LCD_CR3_RESET_VALUE   ((uint8_t) 0x00)

  } CR3;


  /** LCD frequency selection register (FRQ at 0x5403) */
  union {

    /// bytewise access to FRQ
    uint8_t  byte;

    /// bitwise access to register FRQ
    struct {
      BITS   DIV0                : 1;      // bit 0
      BITS   DIV1                : 1;      // bit 1
      BITS   DIV2                : 1;      // bit 2
      BITS   DIV3                : 1;      // bit 3
      BITS   PS0                 : 1;      // bit 4
      BITS   PS1                 : 1;      // bit 5
      BITS   PS2                 : 1;      // bit 6
      BITS   PS3                 : 1;      // bit 7
    };  // FRQ bitfield

    /// register _LCD_FRQ reset value
    #define sfr_LCD_FRQ_RESET_VALUE   ((uint8_t) 0x00)

  } FRQ;


  /** LCD Port mask register 0 (PM0 at 0x5404) */
  union {

    /// bytewise access to PM0
    uint8_t  byte;

    /// bitwise access to register PM0
    struct {
      BITS   SEG00               : 1;      // bit 0
      BITS   SEG01               : 1;      // bit 1
      BITS   SEG02               : 1;      // bit 2
      BITS   SEG03               : 1;      // bit 3
      BITS   SEG04               : 1;      // bit 4
      BITS   SEG05               : 1;      // bit 5
      BITS   SEG06               : 1;      // bit 6
      BITS   SEG07               : 1;      // bit 7
    };  // PM0 bitfield

    /// register _LCD_PM0 reset value
    #define sfr_LCD_PM0_RESET_VALUE   ((uint8_t) 0x00)

  } PM0;


  /** LCD Port mask register 1 (PM1 at 0x5405) */
  union {

    /// bytewise access to PM1
    uint8_t  byte;

    /// bitwise access to register PM1
    struct {
      BITS   SEG08               : 1;      // bit 0
      BITS   SEG09               : 1;      // bit 1
      BITS   SEG10               : 1;      // bit 2
      BITS   SEG11               : 1;      // bit 3
      BITS   SEG12               : 1;      // bit 4
      BITS   SEG13               : 1;      // bit 5
      BITS   SEG14               : 1;      // bit 6
      BITS   SEG15               : 1;      // bit 7
    };  // PM1 bitfield

    /// register _LCD_PM1 reset value
    #define sfr_LCD_PM1_RESET_VALUE   ((uint8_t) 0x00)

  } PM1;


  /** LCD Port mask register 2 (PM2 at 0x5406) */
  union {

    /// bytewise access to PM2
    uint8_t  byte;

    /// bitwise access to register PM2
    struct {
      BITS   SEG16               : 1;      // bit 0
      BITS   SEG17               : 1;      // bit 1
      BITS   SEG18               : 1;      // bit 2
      BITS   SEG19               : 1;      // bit 3
      BITS   SEG20               : 1;      // bit 4
      BITS   SEG21               : 1;      // bit 5
      BITS   SEG22               : 1;      // bit 6
      BITS   SEG23               : 1;      // bit 7
    };  // PM2 bitfield

    /// register _LCD_PM2 reset value
    #define sfr_LCD_PM2_RESET_VALUE   ((uint8_t) 0x00)

  } PM2;


  /** LCD Port mask register 3 (PM3 at 0x5407) */
  union {

    /// bytewise access to PM3
    uint8_t  byte;

    /// bitwise access to register PM3
    struct {
      BITS   SEG24               : 1;      // bit 0
      BITS   SEG25               : 1;      // bit 1
      BITS   SEG26               : 1;      // bit 2
      BITS   SEG27               : 1;      // bit 3
      BITS                       : 4;      // 4 bits
    };  // PM3 bitfield

    /// register _LCD_PM3 reset value
    #define sfr_LCD_PM3_RESET_VALUE   ((uint8_t) 0x00)

  } PM3;


  /// Reserved register (4B)
  uint8_t     Reserved_1[4];


  /** LCD display memory 0 (RAM0 at 0x540c) */
  union {

    /// bytewise access to RAM0
    uint8_t  byte;

    /// bitwise access to register RAM0
    struct {
      BITS   S000                : 1;      // bit 0
      BITS   S001                : 1;      // bit 1
      BITS   S002                : 1;      // bit 2
      BITS   S003                : 1;      // bit 3
      BITS   S004                : 1;      // bit 4
      BITS   S005                : 1;      // bit 5
      BITS   S006                : 1;      // bit 6
      BITS   S007                : 1;      // bit 7
    };  // RAM0 bitfield

    /// register _LCD_RAM0 reset value
    #define sfr_LCD_RAM0_RESET_VALUE   ((uint8_t) 0x00)

  } RAM0;


  /** LCD display memory 1 (RAM1 at 0x540d) */
  union {

    /// bytewise access to RAM1
    uint8_t  byte;

    /// bitwise access to register RAM1
    struct {
      BITS   S008                : 1;      // bit 0
      BITS   S009                : 1;      // bit 1
      BITS   S010                : 1;      // bit 2
      BITS   S011                : 1;      // bit 3
      BITS   S012                : 1;      // bit 4
      BITS   S013                : 1;      // bit 5
      BITS   S014                : 1;      // bit 6
      BITS   S015                : 1;      // bit 7
    };  // RAM1 bitfield

    /// register _LCD_RAM1 reset value
    #define sfr_LCD_RAM1_RESET_VALUE   ((uint8_t) 0x00)

  } RAM1;


  /** LCD display memory 2 (RAM2 at 0x540e) */
  union {

    /// bytewise access to RAM2
    uint8_t  byte;

    /// bitwise access to register RAM2
    struct {
      BITS   S016                : 1;      // bit 0
      BITS   S017                : 1;      // bit 1
      BITS   S018                : 1;      // bit 2
      BITS   S019                : 1;      // bit 3
      BITS   S020                : 1;      // bit 4
      BITS   S021                : 1;      // bit 5
      BITS   S022                : 1;      // bit 6
      BITS   S023                : 1;      // bit 7
    };  // RAM2 bitfield

    /// register _LCD_RAM2 reset value
    #define sfr_LCD_RAM2_RESET_VALUE   ((uint8_t) 0x00)

  } RAM2;


  /** LCD display memory 3 (RAM3 at 0x540f) */
  union {

    /// bytewise access to RAM3
    uint8_t  byte;

    /// bitwise access to register RAM3
    struct {
      BITS   S024                : 1;      // bit 0
      BITS   S025                : 1;      // bit 1
      BITS   S026                : 1;      // bit 2
      BITS   S027                : 1;      // bit 3
      BITS   S100                : 1;      // bit 4
      BITS   S101                : 1;      // bit 5
      BITS   S102                : 1;      // bit 6
      BITS   S103                : 1;      // bit 7
    };  // RAM3 bitfield

    /// register _LCD_RAM3 reset value
    #define sfr_LCD_RAM3_RESET_VALUE   ((uint8_t) 0x00)

  } RAM3;


  /** LCD display memory 4 (RAM4 at 0x5410) */
  union {

    /// bytewise access to RAM4
    uint8_t  byte;

    /// bitwise access to register RAM4
    struct {
      BITS   S104                : 1;      // bit 0
      BITS   S105                : 1;      // bit 1
      BITS   S106                : 1;      // bit 2
      BITS   S107                : 1;      // bit 3
      BITS   S108                : 1;      // bit 4
      BITS   S109                : 1;      // bit 5
      BITS   S110                : 1;      // bit 6
      BITS   S111                : 1;      // bit 7
    };  // RAM4 bitfield

    /// register _LCD_RAM4 reset value
    #define sfr_LCD_RAM4_RESET_VALUE   ((uint8_t) 0x00)

  } RAM4;


  /** LCD display memory 5 (RAM5 at 0x5411) */
  union {

    /// bytewise access to RAM5
    uint8_t  byte;

    /// bitwise access to register RAM5
    struct {
      BITS   S112                : 1;      // bit 0
      BITS   S113                : 1;      // bit 1
      BITS   S114                : 1;      // bit 2
      BITS   S115                : 1;      // bit 3
      BITS   S116                : 1;      // bit 4
      BITS   S117                : 1;      // bit 5
      BITS   S118                : 1;      // bit 6
      BITS   S119                : 1;      // bit 7
    };  // RAM5 bitfield

    /// register _LCD_RAM5 reset value
    #define sfr_LCD_RAM5_RESET_VALUE   ((uint8_t) 0x00)

  } RAM5;


  /** LCD display memory 6 (RAM6 at 0x5412) */
  union {

    /// bytewise access to RAM6
    uint8_t  byte;

    /// bitwise access to register RAM6
    struct {
      BITS   S120                : 1;      // bit 0
      BITS   S121                : 1;      // bit 1
      BITS   S122                : 1;      // bit 2
      BITS   S123                : 1;      // bit 3
      BITS   S124                : 1;      // bit 4
      BITS   S125                : 1;      // bit 5
      BITS   S126                : 1;      // bit 6
      BITS   S127                : 1;      // bit 7
    };  // RAM6 bitfield

    /// register _LCD_RAM6 reset value
    #define sfr_LCD_RAM6_RESET_VALUE   ((uint8_t) 0x00)

  } RAM6;


  /** LCD display memory 7 (RAM7 at 0x5413) */
  union {

    /// bytewise access to RAM7
    uint8_t  byte;

    /// bitwise access to register RAM7
    struct {
      BITS   S200                : 1;      // bit 0
      BITS   S201                : 1;      // bit 1
      BITS   S202                : 1;      // bit 2
      BITS   S203                : 1;      // bit 3
      BITS   S204                : 1;      // bit 4
      BITS   S205                : 1;      // bit 5
      BITS   S206                : 1;      // bit 6
      BITS   S207                : 1;      // bit 7
    };  // RAM7 bitfield

    /// register _LCD_RAM7 reset value
    #define sfr_LCD_RAM7_RESET_VALUE   ((uint8_t) 0x00)

  } RAM7;


  /** LCD display memory 8 (RAM8 at 0x5414) */
  union {

    /// bytewise access to RAM8
    uint8_t  byte;

    /// bitwise access to register RAM8
    struct {
      BITS   S208                : 1;      // bit 0
      BITS   S209                : 1;      // bit 1
      BITS   S210                : 1;      // bit 2
      BITS   S211                : 1;      // bit 3
      BITS   S212                : 1;      // bit 4
      BITS   S213                : 1;      // bit 5
      BITS   S214                : 1;      // bit 6
      BITS   S215                : 1;      // bit 7
    };  // RAM8 bitfield

    /// register _LCD_RAM8 reset value
    #define sfr_LCD_RAM8_RESET_VALUE   ((uint8_t) 0x00)

  } RAM8;


  /** LCD display memory 9 (RAM9 at 0x5415) */
  union {

    /// bytewise access to RAM9
    uint8_t  byte;

    /// bitwise access to register RAM9
    struct {
      BITS   S216                : 1;      // bit 0
      BITS   S217                : 1;      // bit 1
      BITS   S218                : 1;      // bit 2
      BITS   S219                : 1;      // bit 3
      BITS   S220                : 1;      // bit 4
      BITS   S221                : 1;      // bit 5
      BITS   S222                : 1;      // bit 6
      BITS   S223                : 1;      // bit 7
    };  // RAM9 bitfield

    /// register _LCD_RAM9 reset value
    #define sfr_LCD_RAM9_RESET_VALUE   ((uint8_t) 0x00)

  } RAM9;


  /** LCD display memory 10 (RAM10 at 0x5416) */
  union {

    /// bytewise access to RAM10
    uint8_t  byte;

    /// bitwise access to register RAM10
    struct {
      BITS   S224                : 1;      // bit 0
      BITS   S225                : 1;      // bit 1
      BITS   S226                : 1;      // bit 2
      BITS   S227                : 1;      // bit 3
      BITS   S300                : 1;      // bit 4
      BITS   S301                : 1;      // bit 5
      BITS   S302                : 1;      // bit 6
      BITS   S303                : 1;      // bit 7
    };  // RAM10 bitfield

    /// register _LCD_RAM10 reset value
    #define sfr_LCD_RAM10_RESET_VALUE   ((uint8_t) 0x00)

  } RAM10;


  /** LCD display memory 11 (RAM11 at 0x5417) */
  union {

    /// bytewise access to RAM11
    uint8_t  byte;

    /// bitwise access to register RAM11
    struct {
      BITS   S304                : 1;      // bit 0
      BITS   S305                : 1;      // bit 1
      BITS   S306                : 1;      // bit 2
      BITS   S307                : 1;      // bit 3
      BITS   S308                : 1;      // bit 4
      BITS   S309                : 1;      // bit 5
      BITS   S310                : 1;      // bit 6
      BITS   S311                : 1;      // bit 7
    };  // RAM11 bitfield

    /// register _LCD_RAM11 reset value
    #define sfr_LCD_RAM11_RESET_VALUE   ((uint8_t) 0x00)

  } RAM11;


  /** LCD display memory 12 (RAM12 at 0x5418) */
  union {

    /// bytewise access to RAM12
    uint8_t  byte;

    /// bitwise access to register RAM12
    struct {
      BITS   S312                : 1;      // bit 0
      BITS   S313                : 1;      // bit 1
      BITS   S314                : 1;      // bit 2
      BITS   S315                : 1;      // bit 3
      BITS   S316                : 1;      // bit 4
      BITS   S317                : 1;      // bit 5
      BITS   S318                : 1;      // bit 6
      BITS   S319                : 1;      // bit 7
    };  // RAM12 bitfield

    /// register _LCD_RAM12 reset value
    #define sfr_LCD_RAM12_RESET_VALUE   ((uint8_t) 0x00)

  } RAM12;


  /** LCD display memory 13 (RAM13 at 0x5419) */
  union {

    /// bytewise access to RAM13
    uint8_t  byte;

    /// bitwise access to register RAM13
    struct {
      BITS   S320                : 1;      // bit 0
      BITS   S321                : 1;      // bit 1
      BITS   S322                : 1;      // bit 2
      BITS   S323                : 1;      // bit 3
      BITS   S324                : 1;      // bit 4
      BITS   S325                : 1;      // bit 5
      BITS   S326                : 1;      // bit 6
      BITS   S327                : 1;      // bit 7
    };  // RAM13 bitfield

    /// register _LCD_RAM13 reset value
    #define sfr_LCD_RAM13_RESET_VALUE   ((uint8_t) 0x00)

  } RAM13;

} LCD_t;

/// access to LCD SFR registers
#define sfr_LCD   (*((LCD_t*) 0x5400))


//------------------------
// Module OPT
//------------------------

/** struct containing OPT module registers */
typedef struct {

  /** Read-out protection (ROP) (OPT0 at 0x4800) */
  union {

    /// bytewise access to OPT0
    uint8_t  byte;

    /// skip bitwise access to register OPT0

    /// register _OPT_OPT0 reset value
    #define sfr_OPT_OPT0_RESET_VALUE   ((uint8_t) 0xAA)

  } OPT0;


  /// Reserved register (1B)
  uint8_t     Reserved_1[1];


  /** User boot code (UBC) (OPT1 at 0x4802) */
  union {

    /// bytewise access to OPT1
    uint8_t  byte;

    /// skip bitwise access to register OPT1

    /// register _OPT_OPT1 reset value
    #define sfr_OPT_OPT1_RESET_VALUE   ((uint8_t) 0x00)

  } OPT1;


  /// Reserved register (5B)
  uint8_t     Reserved_2[5];


  /** Watchdog option (OPT3 at 0x4808) */
  union {

    /// bytewise access to OPT3
    uint8_t  byte;

    /// skip bitwise access to register OPT3

    /// register _OPT_OPT3 reset value
    #define sfr_OPT_OPT3_RESET_VALUE   ((uint8_t) 0x00)

  } OPT3;


  /** Clock option (OPT4 at 0x4809) */
  union {

    /// bytewise access to OPT4
    uint8_t  byte;

    /// skip bitwise access to register OPT4

    /// register _OPT_OPT4 reset value
    #define sfr_OPT_OPT4_RESET_VALUE   ((uint8_t) 0x00)

  } OPT4;


  /** Brownout reset (BOR) (OPT5 at 0x480a) */
  union {

    /// bytewise access to OPT5
    uint8_t  byte;

    /// skip bitwise access to register OPT5

    /// register _OPT_OPT5 reset value
    #define sfr_OPT_OPT5_RESET_VALUE   ((uint8_t) 0x00)

  } OPT5;


  /** Bootloader (high byte) (OPTBL_H at 0x480b) */
  union {

    /// bytewise access to OPTBL_H
    uint8_t  byte;

    /// skip bitwise access to register OPTBL_H

    /// register _OPT_OPTBL_H reset value
    #define sfr_OPT_OPTBL_H_RESET_VALUE   ((uint8_t) 0x00)

  } OPTBL_H;


  /** Bootloader (low byte) (OPTBL_L at 0x480c) */
  union {

    /// bytewise access to OPTBL_L
    uint8_t  byte;

    /// skip bitwise access to register OPTBL_L

    /// register _OPT_OPTBL_L reset value
    #define sfr_OPT_OPTBL_L_RESET_VALUE   ((uint8_t) 0x00)

  } OPTBL_L;

} OPT_t;

/// access to OPT SFR registers
#define sfr_OPT   (*((OPT_t*) 0x4800))


//------------------------
// Module PWR
//------------------------

/** struct containing PWR module registers */
typedef struct {

  /** Power control and status register 1 (CSR1 at 0x50b2) */
  union {

    /// bytewise access to CSR1
    uint8_t  byte;

    /// bitwise access to register CSR1
    struct {
      BITS   PVDE                : 1;      // bit 0
      BITS   PLS                 : 3;      // bits 1-3
      BITS   PVDIEN              : 1;      // bit 4
      BITS   PVDIF               : 1;      // bit 5
      BITS   PVDOF               : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // CSR1 bitfield

    /// register _PWR_CSR1 reset value
    #define sfr_PWR_CSR1_RESET_VALUE   ((uint8_t) 0x00)

  } CSR1;


  /** Power control and status register 2 (CSR2 at 0x50b3) */
  union {

    /// bytewise access to CSR2
    uint8_t  byte;

    /// bitwise access to register CSR2
    struct {
      BITS   VREFINTF            : 1;      // bit 0
      BITS   ULP                 : 1;      // bit 1
      BITS   FWU                 : 1;      // bit 2
      BITS                       : 5;      // 5 bits
    };  // CSR2 bitfield

    /// register _PWR_CSR2 reset value
    #define sfr_PWR_CSR2_RESET_VALUE   ((uint8_t) 0x00)

  } CSR2;

} PWR_t;

/// access to PWR SFR registers
#define sfr_PWR   (*((PWR_t*) 0x50b2))


//------------------------
// Module PORT
//------------------------

/** struct containing PORTA module registers */
typedef struct {

  /** Port A data output latch register (ODR at 0x5000) */
  union {

    /// bytewise access to ODR
    uint8_t  byte;

    /// bitwise access to register ODR
    struct {
      BITS   ODR0                : 1;      // bit 0
      BITS   ODR1                : 1;      // bit 1
      BITS   ODR2                : 1;      // bit 2
      BITS   ODR3                : 1;      // bit 3
      BITS   ODR4                : 1;      // bit 4
      BITS   ODR5                : 1;      // bit 5
      BITS   ODR6                : 1;      // bit 6
      BITS   ODR7                : 1;      // bit 7
    };  // ODR bitfield

    /// register _PORT_ODR reset value
    #define sfr_PORT_ODR_RESET_VALUE   ((uint8_t) 0x00)

  } ODR;


  /** Port A input pin value register (IDR at 0x5001) */
  union {

    /// bytewise access to IDR
    uint8_t  byte;

    /// bitwise access to register IDR
    struct {
      BITS   IDR0                : 1;      // bit 0
      BITS   IDR1                : 1;      // bit 1
      BITS   IDR2                : 1;      // bit 2
      BITS   IDR3                : 1;      // bit 3
      BITS   IDR4                : 1;      // bit 4
      BITS   IDR5                : 1;      // bit 5
      BITS   IDR6                : 1;      // bit 6
      BITS   IDR7                : 1;      // bit 7
    };  // IDR bitfield

    /// register _PORT_IDR reset value
    #define sfr_PORT_IDR_RESET_VALUE   ((uint8_t) 0x00)

  } IDR;


  /** Port A data direction register (DDR at 0x5002) */
  union {

    /// bytewise access to DDR
    uint8_t  byte;

    /// bitwise access to register DDR
    struct {
      BITS   DDR0                : 1;      // bit 0
      BITS   DDR1                : 1;      // bit 1
      BITS   DDR2                : 1;      // bit 2
      BITS   DDR3                : 1;      // bit 3
      BITS   DDR4                : 1;      // bit 4
      BITS   DDR5                : 1;      // bit 5
      BITS   DDR6                : 1;      // bit 6
      BITS   DDR7                : 1;      // bit 7
    };  // DDR bitfield

    /// register _PORT_DDR reset value
    #define sfr_PORT_DDR_RESET_VALUE   ((uint8_t) 0x00)

  } DDR;


  /** Port A control register 1 (CR1 at 0x5003) */
  union {

    /// bytewise access to CR1
    uint8_t  byte;

    /// bitwise access to register CR1
    struct {
      BITS   C10                 : 1;      // bit 0
      BITS   C11                 : 1;      // bit 1
      BITS   C12                 : 1;      // bit 2
      BITS   C13                 : 1;      // bit 3
      BITS   C14                 : 1;      // bit 4
      BITS   C15                 : 1;      // bit 5
      BITS   C16                 : 1;      // bit 6
      BITS   C17                 : 1;      // bit 7
    };  // CR1 bitfield

    /// register _PORT_CR1 reset value
    #define sfr_PORT_CR1_RESET_VALUE   ((uint8_t) 0x01)

  } CR1;


  /** Port A control register 2 (CR2 at 0x5004) */
  union {

    /// bytewise access to CR2
    uint8_t  byte;

    /// bitwise access to register CR2
    struct {
      BITS   C20                 : 1;      // bit 0
      BITS   C21                 : 1;      // bit 1
      BITS   C22                 : 1;      // bit 2
      BITS   C23                 : 1;      // bit 3
      BITS   C24                 : 1;      // bit 4
      BITS   C25                 : 1;      // bit 5
      BITS   C26                 : 1;      // bit 6
      BITS   C27                 : 1;      // bit 7
    };  // CR2 bitfield

    /// register _PORT_CR2 reset value
    #define sfr_PORT_CR2_RESET_VALUE   ((uint8_t) 0x00)

  } CR2;

} PORT_t;

/// access to PORTA SFR registers
#define sfr_PORTA   (*((PORT_t*) 0x5000))


/// access to PORTB SFR registers
#define sfr_PORTB   (*((PORT_t*) 0x5005))


/// access to PORTC SFR registers
#define sfr_PORTC   (*((PORT_t*) 0x500a))


/// access to PORTD SFR registers
#define sfr_PORTD   (*((PORT_t*) 0x500f))


/// access to PORTE SFR registers
#define sfr_PORTE   (*((PORT_t*) 0x5014))


/// access to PORTF SFR registers
#define sfr_PORTF   (*((PORT_t*) 0x5019))


//------------------------
// Module REMAP
//------------------------

/** struct containing REMAP module registers */
typedef struct {

  /** Remapping register 1 (SYSCFG_RMPCR1 at 0x509e) */
  union {

    /// bytewise access to SYSCFG_RMPCR1
    uint8_t  byte;

    /// bitwise access to register SYSCFG_RMPCR1
    struct {
      BITS   ADC1DMA_REMAP       : 2;      // bits 0-1
      BITS   TIM4DMA_REMAP       : 2;      // bits 2-3
      BITS   USART1TR_REMAP      : 2;      // bits 4-5
      BITS   USART1CK_REMAP      : 1;      // bit 6
      BITS   SPI1_REMAP          : 1;      // bit 7
    };  // SYSCFG_RMPCR1 bitfield

    /// register _REMAP_SYSCFG_RMPCR1 reset value
    #define sfr_REMAP_SYSCFG_RMPCR1_RESET_VALUE   ((uint8_t) 0x00)

  } SYSCFG_RMPCR1;


  /** Remapping register 2 (SYSCFG_RMPCR2 at 0x509f) */
  union {

    /// bytewise access to SYSCFG_RMPCR2
    uint8_t  byte;

    /// bitwise access to register SYSCFG_RMPCR2
    struct {
      BITS   ADC1TRIG_REMAP      : 1;      // bit 0
      BITS   TIM2TRIG_REMAP      : 1;      // bit 1
      BITS   TIM3TRIG_REMAP      : 1;      // bit 2
      BITS   TIM2TRIGLSE_REMAP   : 1;      // bit 3
      BITS   TIM3TRIGLSE_REMAP   : 1;      // bit 4
      BITS                       : 3;      // 3 bits
    };  // SYSCFG_RMPCR2 bitfield

    /// register _REMAP_SYSCFG_RMPCR2 reset value
    #define sfr_REMAP_SYSCFG_RMPCR2_RESET_VALUE   ((uint8_t) 0x00)

  } SYSCFG_RMPCR2;

} REMAP_t;

/// access to REMAP SFR registers
#define sfr_REMAP   (*((REMAP_t*) 0x509e))


//------------------------
// Module RI
//------------------------

/** struct containing RI module registers */
typedef struct {

  /** Timer input capture routing register 1 (ICR1 at 0x5431) */
  union {

    /// bytewise access to ICR1
    uint8_t  byte;

    /// bitwise access to register ICR1
    struct {
      BITS   IC2CS               : 5;      // bits 0-4
      BITS                       : 3;      // 3 bits
    };  // ICR1 bitfield

    /// register _RI_ICR1 reset value
    #define sfr_RI_ICR1_RESET_VALUE   ((uint8_t) 0x00)

  } ICR1;


  /** Timer input capture routing register 2 (ICR2 at 0x5432) */
  union {

    /// bytewise access to ICR2
    uint8_t  byte;

    /// bitwise access to register ICR2
    struct {
      BITS   IC3CS               : 5;      // bits 0-4
      BITS                       : 3;      // 3 bits
    };  // ICR2 bitfield

    /// register _RI_ICR2 reset value
    #define sfr_RI_ICR2_RESET_VALUE   ((uint8_t) 0x00)

  } ICR2;


  /** I/O input register 1 (IOIR1 at 0x5433) */
  union {

    /// bytewise access to IOIR1
    uint8_t  byte;

    /// bitwise access to register IOIR1
    struct {
      BITS   CH1I                : 1;      // bit 0
      BITS   CH4I                : 1;      // bit 1
      BITS   CH7I                : 1;      // bit 2
      BITS   CH10I               : 1;      // bit 3
      BITS   CH13I               : 1;      // bit 4
      BITS   CH16I               : 1;      // bit 5
      BITS   CH19I               : 1;      // bit 6
      BITS   CH22I               : 1;      // bit 7
    };  // IOIR1 bitfield

    /// register _RI_IOIR1 reset value
    #define sfr_RI_IOIR1_RESET_VALUE   ((uint8_t) 0x00)

  } IOIR1;


  /** I/O input register 2 (IOIR2 at 0x5434) */
  union {

    /// bytewise access to IOIR2
    uint8_t  byte;

    /// bitwise access to register IOIR2
    struct {
      BITS   CH2I                : 1;      // bit 0
      BITS   CH5I                : 1;      // bit 1
      BITS   CH8I                : 1;      // bit 2
      BITS   CH11I               : 1;      // bit 3
      BITS   CH14I               : 1;      // bit 4
      BITS   CH17I               : 1;      // bit 5
      BITS   CH20I               : 1;      // bit 6
      BITS   CH23I               : 1;      // bit 7
    };  // IOIR2 bitfield

    /// register _RI_IOIR2 reset value
    #define sfr_RI_IOIR2_RESET_VALUE   ((uint8_t) 0x00)

  } IOIR2;


  /** I/O input register 3 (IOIR3 at 0x5435) */
  union {

    /// bytewise access to IOIR3
    uint8_t  byte;

    /// bitwise access to register IOIR3
    struct {
      BITS   CH3I                : 1;      // bit 0
      BITS   CH6I                : 1;      // bit 1
      BITS   CH9I                : 1;      // bit 2
      BITS   CH12I               : 1;      // bit 3
      BITS   CH15I               : 1;      // bit 4
      BITS   CH18I               : 1;      // bit 5
      BITS   CH21I               : 1;      // bit 6
      BITS   CH24I               : 1;      // bit 7
    };  // IOIR3 bitfield

    /// register _RI_IOIR3 reset value
    #define sfr_RI_IOIR3_RESET_VALUE   ((uint8_t) 0x00)

  } IOIR3;


  /** I/O control mode register 1 (IOCMR1 at 0x5436) */
  union {

    /// bytewise access to IOCMR1
    uint8_t  byte;

    /// bitwise access to register IOCMR1
    struct {
      BITS   CH1M                : 1;      // bit 0
      BITS   CH4M                : 1;      // bit 1
      BITS   CH7M                : 1;      // bit 2
      BITS   CH10M               : 1;      // bit 3
      BITS   CH13M               : 1;      // bit 4
      BITS   CH16M               : 1;      // bit 5
      BITS   CH19M               : 1;      // bit 6
      BITS   CH22M               : 1;      // bit 7
    };  // IOCMR1 bitfield

    /// register _RI_IOCMR1 reset value
    #define sfr_RI_IOCMR1_RESET_VALUE   ((uint8_t) 0x00)

  } IOCMR1;


  /** I/O control mode register 2 (IOCMR2 at 0x5437) */
  union {

    /// bytewise access to IOCMR2
    uint8_t  byte;

    /// bitwise access to register IOCMR2
    struct {
      BITS   CH2M                : 1;      // bit 0
      BITS   CH5M                : 1;      // bit 1
      BITS   CH8M                : 1;      // bit 2
      BITS   CH11M               : 1;      // bit 3
      BITS   CH14M               : 1;      // bit 4
      BITS   CH17M               : 1;      // bit 5
      BITS   CH20M               : 1;      // bit 6
      BITS   CH23M               : 1;      // bit 7
    };  // IOCMR2 bitfield

    /// register _RI_IOCMR2 reset value
    #define sfr_RI_IOCMR2_RESET_VALUE   ((uint8_t) 0x00)

  } IOCMR2;


  /** I/O control mode register 3 (IOCMR3 at 0x5438) */
  union {

    /// bytewise access to IOCMR3
    uint8_t  byte;

    /// bitwise access to register IOCMR3
    struct {
      BITS   CH3M                : 1;      // bit 0
      BITS   CH6M                : 1;      // bit 1
      BITS   CH9M                : 1;      // bit 2
      BITS   CH12M               : 1;      // bit 3
      BITS   CH53M               : 1;      // bit 4
      BITS   CH18M               : 1;      // bit 5
      BITS   CH21M               : 1;      // bit 6
      BITS   CH24M               : 1;      // bit 7
    };  // IOCMR3 bitfield

    /// register _RI_IOCMR3 reset value
    #define sfr_RI_IOCMR3_RESET_VALUE   ((uint8_t) 0x00)

  } IOCMR3;


  /** I/O switch register 1 (IOSR1 at 0x5439) */
  union {

    /// bytewise access to IOSR1
    uint8_t  byte;

    /// bitwise access to register IOSR1
    struct {
      BITS   CH1E                : 1;      // bit 0
      BITS   CH4E                : 1;      // bit 1
      BITS   CH7E                : 1;      // bit 2
      BITS   CH10E               : 1;      // bit 3
      BITS   CH13E               : 1;      // bit 4
      BITS   CH16E               : 1;      // bit 5
      BITS   CH19E               : 1;      // bit 6
      BITS   CH22E               : 1;      // bit 7
    };  // IOSR1 bitfield

    /// register _RI_IOSR1 reset value
    #define sfr_RI_IOSR1_RESET_VALUE   ((uint8_t) 0x00)

  } IOSR1;


  /** I/O switch register 2 (IOSR2 at 0x543a) */
  union {

    /// bytewise access to IOSR2
    uint8_t  byte;

    /// bitwise access to register IOSR2
    struct {
      BITS   CH2E                : 1;      // bit 0
      BITS   CH5E                : 1;      // bit 1
      BITS   CH8E                : 1;      // bit 2
      BITS   CH11E               : 1;      // bit 3
      BITS   CH14E               : 1;      // bit 4
      BITS   CH17E               : 1;      // bit 5
      BITS   CH20E               : 1;      // bit 6
      BITS   CH23E               : 1;      // bit 7
    };  // IOSR2 bitfield

    /// register _RI_IOSR2 reset value
    #define sfr_RI_IOSR2_RESET_VALUE   ((uint8_t) 0x00)

  } IOSR2;


  /** I/O switch register 3 (IOSR3 at 0x543b) */
  union {

    /// bytewise access to IOSR3
    uint8_t  byte;

    /// bitwise access to register IOSR3
    struct {
      BITS   CH3E                : 1;      // bit 0
      BITS   CH6E                : 1;      // bit 1
      BITS   CH9E                : 1;      // bit 2
      BITS   CH12E               : 1;      // bit 3
      BITS   CH15E               : 1;      // bit 4
      BITS   CH18E               : 1;      // bit 5
      BITS   CH21E               : 1;      // bit 6
      BITS   CH24E               : 1;      // bit 7
    };  // IOSR3 bitfield

    /// register _RI_IOSR3 reset value
    #define sfr_RI_IOSR3_RESET_VALUE   ((uint8_t) 0x00)

  } IOSR3;


  /** I/O group control register (IOGCR at 0x543c) */
  union {

    /// bytewise access to IOGCR
    uint8_t  byte;

    /// bitwise access to register IOGCR
    struct {
      BITS   IOM1                : 2;      // bits 0-1
      BITS   IOM2                : 2;      // bits 2-3
      BITS   IOM3                : 2;      // bits 4-5
      BITS                       : 2;      // 2 bits
    };  // IOGCR bitfield

    /// register _RI_IOGCR reset value
    #define sfr_RI_IOGCR_RESET_VALUE   ((uint8_t) 0x3F)

  } IOGCR;


  /** Analog switch register 1 (ASCR1 at 0x543d) */
  union {

    /// bytewise access to ASCR1
    uint8_t  byte;

    /// bitwise access to register ASCR1
    struct {
      BITS   AS0                 : 1;      // bit 0
      BITS   AS1                 : 1;      // bit 1
      BITS   AS2                 : 1;      // bit 2
      BITS   AS3                 : 1;      // bit 3
      BITS   AS4                 : 1;      // bit 4
      BITS   AS5                 : 1;      // bit 5
      BITS   AS6                 : 1;      // bit 6
      BITS   AS7                 : 1;      // bit 7
    };  // ASCR1 bitfield

    /// register _RI_ASCR1 reset value
    #define sfr_RI_ASCR1_RESET_VALUE   ((uint8_t) 0x00)

  } ASCR1;


  /** Analog switch register 2 (ASCR2 at 0x543e) */
  union {

    /// bytewise access to ASCR2
    uint8_t  byte;

    /// bitwise access to register ASCR2
    struct {
      BITS   AS8                 : 1;      // bit 0
      BITS                       : 5;      // 5 bits
      BITS   AS14                : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // ASCR2 bitfield

    /// register _RI_ASCR2 reset value
    #define sfr_RI_ASCR2_RESET_VALUE   ((uint8_t) 0x00)

  } ASCR2;


  /** Resistor control register 1 (RCR at 0x543f) */
  union {

    /// bytewise access to RCR
    uint8_t  byte;

    /// bitwise access to register RCR
    struct {
      BITS   KPU10               : 1;      // bit 0
      BITS   KPU400              : 1;      // bit 1
      BITS   KPD10               : 1;      // bit 2
      BITS   KPD400              : 1;      // bit 3
      BITS                       : 4;      // 4 bits
    };  // RCR bitfield

    /// register _RI_RCR reset value
    #define sfr_RI_RCR_RESET_VALUE   ((uint8_t) 0x00)

  } RCR;

} RI_t;

/// access to RI SFR registers
#define sfr_RI   (*((RI_t*) 0x5431))


//------------------------
// Module RST
//------------------------

/** struct containing RST module registers */
typedef struct {

  /** Reset control register (CR at 0x50b0) */
  union {

    /// bytewise access to CR
    uint8_t  byte;

    /// bitwise access to register CR
    struct {
      BITS   RSTPIN_KEY          : 8;      // bits 0-7
    };  // CR bitfield

    /// register _RST_CR reset value
    #define sfr_RST_CR_RESET_VALUE   ((uint8_t) 0x00)

  } CR;


  /** Reset status register (SR at 0x50b1) */
  union {

    /// bytewise access to SR
    uint8_t  byte;

    /// bitwise access to register SR
    struct {
      BITS   PORF                : 1;      // bit 0
      BITS   IWDGF               : 1;      // bit 1
      BITS   ILLOPF              : 1;      // bit 2
      BITS   SWIMF               : 1;      // bit 3
      BITS   WWDGF               : 1;      // bit 4
      BITS   BORF                : 1;      // bit 5
      BITS                       : 2;      // 2 bits
    };  // SR bitfield

    /// register _RST_SR reset value
    #define sfr_RST_SR_RESET_VALUE   ((uint8_t) 0x01)

  } SR;

} RST_t;

/// access to RST SFR registers
#define sfr_RST   (*((RST_t*) 0x50b0))


//------------------------
// Module RTC
//------------------------

/** struct containing RTC module registers */
typedef struct {

  /** Time register 1 (TR1 at 0x5140) */
  union {

    /// bytewise access to TR1
    uint8_t  byte;

    /// bitwise access to register TR1
    struct {
      BITS   SU                  : 4;      // bits 0-3
      BITS   ST                  : 4;      // bits 4-7
    };  // TR1 bitfield

    /// register _RTC_TR1 reset value
    #define sfr_RTC_TR1_RESET_VALUE   ((uint8_t) 0x00)

  } TR1;


  /** Time register 2 (TR2 at 0x5141) */
  union {

    /// bytewise access to TR2
    uint8_t  byte;

    /// bitwise access to register TR2
    struct {
      BITS   MNU                 : 4;      // bits 0-3
      BITS   MNT                 : 4;      // bits 4-7
    };  // TR2 bitfield

    /// register _RTC_TR2 reset value
    #define sfr_RTC_TR2_RESET_VALUE   ((uint8_t) 0x00)

  } TR2;


  /** Time register 3 (TR3 at 0x5142) */
  union {

    /// bytewise access to TR3
    uint8_t  byte;

    /// bitwise access to register TR3
    struct {
      BITS   HU                  : 4;      // bits 0-3
      BITS   HT                  : 2;      // bits 4-5
      BITS   PM                  : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // TR3 bitfield

    /// register _RTC_TR3 reset value
    #define sfr_RTC_TR3_RESET_VALUE   ((uint8_t) 0x00)

  } TR3;


  /// Reserved register (1B)
  uint8_t     Reserved_1[1];


  /** Date register 1 (DR1 at 0x5144) */
  union {

    /// bytewise access to DR1
    uint8_t  byte;

    /// bitwise access to register DR1
    struct {
      BITS   DU                  : 4;      // bits 0-3
      BITS   DT                  : 2;      // bits 4-5
      BITS                       : 2;      // 2 bits
    };  // DR1 bitfield

    /// register _RTC_DR1 reset value
    #define sfr_RTC_DR1_RESET_VALUE   ((uint8_t) 0x01)

  } DR1;


  /** Date register 2 (DR2 at 0x5145) */
  union {

    /// bytewise access to DR2
    uint8_t  byte;

    /// bitwise access to register DR2
    struct {
      BITS   MU                  : 4;      // bits 0-3
      BITS   MT                  : 1;      // bit 4
      BITS   WDU                 : 3;      // bits 5-7
    };  // DR2 bitfield

    /// register _RTC_DR2 reset value
    #define sfr_RTC_DR2_RESET_VALUE   ((uint8_t) 0x21)

  } DR2;


  /** Date register 3 (DR3 at 0x5146) */
  union {

    /// bytewise access to DR3
    uint8_t  byte;

    /// bitwise access to register DR3
    struct {
      BITS   YU                  : 4;      // bits 0-3
      BITS   YT                  : 4;      // bits 4-7
    };  // DR3 bitfield

    /// register _RTC_DR3 reset value
    #define sfr_RTC_DR3_RESET_VALUE   ((uint8_t) 0x00)

  } DR3;


  /// Reserved register (1B)
  uint8_t     Reserved_2[1];


  /** Control register 1 (CR1 at 0x5148) */
  union {

    /// bytewise access to CR1
    uint8_t  byte;

    /// bitwise access to register CR1
    struct {
      BITS   WUCKSEL             : 4;      // bits 0-3
      BITS                       : 1;      // 1 bit
      BITS   RATIO               : 1;      // bit 5
      BITS   FMT                 : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // CR1 bitfield

    /// register _RTC_CR1 reset value
    #define sfr_RTC_CR1_RESET_VALUE   ((uint8_t) 0x00)

  } CR1;


  /** Control register 2 (CR2 at 0x5149) */
  union {

    /// bytewise access to CR2
    uint8_t  byte;

    /// bitwise access to register CR2
    struct {
      BITS   ALRAE               : 1;      // bit 0
      BITS                       : 1;      // 1 bit
      BITS   WUTE                : 1;      // bit 2
      BITS                       : 1;      // 1 bit
      BITS   ALRAIE              : 1;      // bit 4
      BITS                       : 1;      // 1 bit
      BITS   WUTIE               : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // CR2 bitfield

    /// register _RTC_CR2 reset value
    #define sfr_RTC_CR2_RESET_VALUE   ((uint8_t) 0x00)

  } CR2;


  /** Control register 3 (CR3 at 0x514a) */
  union {

    /// bytewise access to CR3
    uint8_t  byte;

    /// bitwise access to register CR3
    struct {
      BITS   ADD1H               : 1;      // bit 0
      BITS   SUB1H               : 1;      // bit 1
      BITS   BCK                 : 1;      // bit 2
      BITS                       : 1;      // 1 bit
      BITS   POL                 : 1;      // bit 4
      BITS   OSEL                : 2;      // bits 5-6
      BITS   COE                 : 1;      // bit 7
    };  // CR3 bitfield

    /// register _RTC_CR3 reset value
    #define sfr_RTC_CR3_RESET_VALUE   ((uint8_t) 0x00)

  } CR3;


  /// Reserved register (1B)
  uint8_t     Reserved_3[1];


  /** Initialization and status register 1 (ISR1 at 0x514c) */
  union {

    /// bytewise access to ISR1
    uint8_t  byte;

    /// bitwise access to register ISR1
    struct {
      BITS   ALRAWF              : 1;      // bit 0
      BITS   RECALPF             : 1;      // bit 1
      BITS   WUTWF               : 1;      // bit 2
      BITS   SHPF                : 1;      // bit 3
      BITS   INITS               : 1;      // bit 4
      BITS   RSF                 : 1;      // bit 5
      BITS   INITF               : 1;      // bit 6
      BITS   INIT                : 1;      // bit 7
    };  // ISR1 bitfield

    /// register _RTC_ISR1 reset value
    #define sfr_RTC_ISR1_RESET_VALUE   ((uint8_t) 0x00)

  } ISR1;


  /** Initialization and Status register 2 (ISR2 at 0x514d) */
  union {

    /// bytewise access to ISR2
    uint8_t  byte;

    /// bitwise access to register ISR2
    struct {
      BITS   ALRAF               : 1;      // bit 0
      BITS                       : 1;      // 1 bit
      BITS   WUTF                : 1;      // bit 2
      BITS                       : 2;      // 2 bits
      BITS   TAMP1F              : 1;      // bit 5
      BITS   TAMP2F              : 1;      // bit 6
      BITS   TAMP3F              : 1;      // bit 7
    };  // ISR2 bitfield

    /// register _RTC_ISR2 reset value
    #define sfr_RTC_ISR2_RESET_VALUE   ((uint8_t) 0x00)

  } ISR2;


  /// Reserved register (2B)
  uint8_t     Reserved_4[2];


  /** Synchronous prescaler register high (SPRERH at 0x5150) */
  union {

    /// bytewise access to SPRERH
    uint8_t  byte;

    /// bitwise access to register SPRERH
    struct {
      BITS   PREDIV_S8           : 1;      // bit 0
      BITS   PREDIV_S9           : 1;      // bit 1
      BITS   PREDIV_S10          : 1;      // bit 2
      BITS   PREDIV_S11          : 1;      // bit 3
      BITS   PREDIV_S12          : 1;      // bit 4
      BITS                       : 3;      // 3 bits
    };  // SPRERH bitfield

    /// register _RTC_SPRERH reset value
    #define sfr_RTC_SPRERH_RESET_VALUE   ((uint8_t) 0x00)

  } SPRERH;


  /** Synchronous prescaler register low (SPRERL at 0x5151) */
  union {

    /// bytewise access to SPRERL
    uint8_t  byte;

    /// bitwise access to register SPRERL
    struct {
      BITS   PREDIV_S0           : 1;      // bit 0
      BITS   PREDIV_S1           : 1;      // bit 1
      BITS   PREDIV_S2           : 1;      // bit 2
      BITS   PREDIV_S3           : 1;      // bit 3
      BITS   PREDIV_S4           : 1;      // bit 4
      BITS   PREDIV_S5           : 1;      // bit 5
      BITS   PREDIV_S6           : 1;      // bit 6
      BITS   PREDIV_S7           : 1;      // bit 7
    };  // SPRERL bitfield

    /// register _RTC_SPRERL reset value
    #define sfr_RTC_SPRERL_RESET_VALUE   ((uint8_t) 0xFF)

  } SPRERL;


  /** Asynchronous prescaler register (APRER at 0x5152) */
  union {

    /// bytewise access to APRER
    uint8_t  byte;

    /// bitwise access to register APRER
    struct {
      BITS   PREDIV_A            : 7;      // bits 0-6
      BITS                       : 1;      // 1 bit
    };  // APRER bitfield

    /// register _RTC_APRER reset value
    #define sfr_RTC_APRER_RESET_VALUE   ((uint8_t) 0x7F)

  } APRER;


  /// Reserved register (1B)
  uint8_t     Reserved_5[1];


  /** Wakeup timer register high (WUTRH at 0x5154) */
  union {

    /// bytewise access to WUTRH
    uint8_t  byte;

    /// bitwise access to register WUTRH
    struct {
      BITS   WUT8                : 1;      // bit 0
      BITS   WUT9                : 1;      // bit 1
      BITS   WUT10               : 1;      // bit 2
      BITS   WUT11               : 1;      // bit 3
      BITS   WUT12               : 1;      // bit 4
      BITS   WUT13               : 1;      // bit 5
      BITS   WUT14               : 1;      // bit 6
      BITS   WUT15               : 1;      // bit 7
    };  // WUTRH bitfield

    /// register _RTC_WUTRH reset value
    #define sfr_RTC_WUTRH_RESET_VALUE   ((uint8_t) 0xFF)

  } WUTRH;


  /** Wakeup timer register low (WUTRL at 0x5155) */
  union {

    /// bytewise access to WUTRL
    uint8_t  byte;

    /// bitwise access to register WUTRL
    struct {
      BITS   WUT0                : 1;      // bit 0
      BITS   WUT1                : 1;      // bit 1
      BITS   WUT2                : 1;      // bit 2
      BITS   WUT3                : 1;      // bit 3
      BITS   WUT4                : 1;      // bit 4
      BITS   WUT5                : 1;      // bit 5
      BITS   WUT6                : 1;      // bit 6
      BITS   WUT7                : 1;      // bit 7
    };  // WUTRL bitfield

    /// register _RTC_WUTRL reset value
    #define sfr_RTC_WUTRL_RESET_VALUE   ((uint8_t) 0xFF)

  } WUTRL;


  /// Reserved register (3B)
  uint8_t     Reserved_6[3];


  /** Write protection register (WPR at 0x5159) */
  union {

    /// bytewise access to WPR
    uint8_t  byte;

    /// skip bitwise access to register WPR

    /// register _RTC_WPR reset value
    #define sfr_RTC_WPR_RESET_VALUE   ((uint8_t) 0x00)

  } WPR;


  /// Reserved register (2B)
  uint8_t     Reserved_7[2];


  /** Alarm A register 1 (ALRMAR1 at 0x515c) */
  union {

    /// bytewise access to ALRMAR1
    uint8_t  byte;

    /// bitwise access to register ALRMAR1
    struct {
      BITS   ALSU                : 4;      // bits 0-3
      BITS   ALST                : 3;      // bits 4-6
      BITS   MSK1                : 1;      // bit 7
    };  // ALRMAR1 bitfield

    /// register _RTC_ALRMAR1 reset value
    #define sfr_RTC_ALRMAR1_RESET_VALUE   ((uint8_t) 0x00)

  } ALRMAR1;


  /** Alarm A register 2 (ALRMAR2 at 0x515d) */
  union {

    /// bytewise access to ALRMAR2
    uint8_t  byte;

    /// bitwise access to register ALRMAR2
    struct {
      BITS   ALMNU               : 4;      // bits 0-3
      BITS   ALMNT               : 3;      // bits 4-6
      BITS   MSK2                : 1;      // bit 7
    };  // ALRMAR2 bitfield

    /// register _RTC_ALRMAR2 reset value
    #define sfr_RTC_ALRMAR2_RESET_VALUE   ((uint8_t) 0x00)

  } ALRMAR2;


  /** Alarm A register 3 (ALRMAR3 at 0x515e) */
  union {

    /// bytewise access to ALRMAR3
    uint8_t  byte;

    /// bitwise access to register ALRMAR3
    struct {
      BITS   ALHU                : 4;      // bits 0-3
      BITS   ALHT                : 2;      // bits 4-5
      BITS   PM                  : 1;      // bit 6
      BITS   MSK3                : 1;      // bit 7
    };  // ALRMAR3 bitfield

    /// register _RTC_ALRMAR3 reset value
    #define sfr_RTC_ALRMAR3_RESET_VALUE   ((uint8_t) 0x00)

  } ALRMAR3;


  /** Alarm A register 4 (ALRMAR4 at 0x515f) */
  union {

    /// bytewise access to ALRMAR4
    uint8_t  byte;

    /// bitwise access to register ALRMAR4
    struct {
      BITS   ALDU                : 4;      // bits 0-3
      BITS   ALDT                : 2;      // bits 4-5
      BITS   WDSEL               : 1;      // bit 6
      BITS   MSK4                : 1;      // bit 7
    };  // ALRMAR4 bitfield

    /// register _RTC_ALRMAR4 reset value
    #define sfr_RTC_ALRMAR4_RESET_VALUE   ((uint8_t) 0x00)

  } ALRMAR4;

} RTC_t;

/// access to RTC SFR registers
#define sfr_RTC   (*((RTC_t*) 0x5140))


//------------------------
// Module SPI1
//------------------------

/** struct containing SPI1 module registers */
typedef struct {

  /** SPI1 control register 1 (CR1 at 0x5200) */
  union {

    /// bytewise access to CR1
    uint8_t  byte;

    /// bitwise access to register CR1
    struct {
      BITS   CPHA                : 1;      // bit 0
      BITS   CPOL                : 1;      // bit 1
      BITS   MSTR                : 1;      // bit 2
      BITS   BR                  : 3;      // bits 3-5
      BITS   SPE                 : 1;      // bit 6
      BITS   LSBFIRST            : 1;      // bit 7
    };  // CR1 bitfield

    /// register _SPI1_CR1 reset value
    #define sfr_SPI1_CR1_RESET_VALUE   ((uint8_t) 0x00)

  } CR1;


  /** SPI1 control register 2 (CR2 at 0x5201) */
  union {

    /// bytewise access to CR2
    uint8_t  byte;

    /// bitwise access to register CR2
    struct {
      BITS   SSI                 : 1;      // bit 0
      BITS   SSM                 : 1;      // bit 1
      BITS   RXONLY              : 1;      // bit 2
      BITS                       : 1;      // 1 bit
      BITS   CRCNEXT             : 1;      // bit 4
      BITS   CRCEN               : 1;      // bit 5
      BITS   BDOE                : 1;      // bit 6
      BITS   BDM                 : 1;      // bit 7
    };  // CR2 bitfield

    /// register _SPI1_CR2 reset value
    #define sfr_SPI1_CR2_RESET_VALUE   ((uint8_t) 0x00)

  } CR2;


  /** SPI1 interrupt control register (ICR at 0x5202) */
  union {

    /// bytewise access to ICR
    uint8_t  byte;

    /// bitwise access to register ICR
    struct {
      BITS   RXDMAEN             : 1;      // bit 0
      BITS   TXDMAEN             : 1;      // bit 1
      BITS                       : 2;      // 2 bits
      BITS   WKIE                : 1;      // bit 4
      BITS   ERRIE               : 1;      // bit 5
      BITS   RXIE                : 1;      // bit 6
      BITS   TXIE                : 1;      // bit 7
    };  // ICR bitfield

    /// register _SPI1_ICR reset value
    #define sfr_SPI1_ICR_RESET_VALUE   ((uint8_t) 0x00)

  } ICR;


  /** SPI1 status register (SR at 0x5203) */
  union {

    /// bytewise access to SR
    uint8_t  byte;

    /// bitwise access to register SR
    struct {
      BITS   RXNE                : 1;      // bit 0
      BITS   TXE                 : 1;      // bit 1
      BITS                       : 1;      // 1 bit
      BITS   WKUP                : 1;      // bit 3
      BITS   CRCERR              : 1;      // bit 4
      BITS   MODF                : 1;      // bit 5
      BITS   OVR                 : 1;      // bit 6
      BITS   BSY                 : 1;      // bit 7
    };  // SR bitfield

    /// register _SPI1_SR reset value
    #define sfr_SPI1_SR_RESET_VALUE   ((uint8_t) 0x02)

  } SR;


  /** SPI1 data register (DR at 0x5204) */
  union {

    /// bytewise access to DR
    uint8_t  byte;

    /// bitwise access to register DR
    struct {
      BITS   DR                  : 8;      // bits 0-7
    };  // DR bitfield

    /// register _SPI1_DR reset value
    #define sfr_SPI1_DR_RESET_VALUE   ((uint8_t) 0x00)

  } DR;


  /** SPI1 CRC polynomial register (CRCPR at 0x5205) */
  union {

    /// bytewise access to CRCPR
    uint8_t  byte;

    /// bitwise access to register CRCPR
    struct {
      BITS   CRCPOLY             : 8;      // bits 0-7
    };  // CRCPR bitfield

    /// register _SPI1_CRCPR reset value
    #define sfr_SPI1_CRCPR_RESET_VALUE   ((uint8_t) 0x07)

  } CRCPR;


  /** SPI1 Rx CRC register (RXCRCR at 0x5206) */
  union {

    /// bytewise access to RXCRCR
    uint8_t  byte;

    /// bitwise access to register RXCRCR
    struct {
      BITS   RXCRC               : 8;      // bits 0-7
    };  // RXCRCR bitfield

    /// register _SPI1_RXCRCR reset value
    #define sfr_SPI1_RXCRCR_RESET_VALUE   ((uint8_t) 0x00)

  } RXCRCR;


  /** SPI1 Tx CRC register (TXCRCR at 0x5207) */
  union {

    /// bytewise access to TXCRCR
    uint8_t  byte;

    /// bitwise access to register TXCRCR
    struct {
      BITS   TXCRC               : 7;      // bits 0-6
      BITS                       : 1;      // 1 bit
    };  // TXCRCR bitfield

    /// register _SPI1_TXCRCR reset value
    #define sfr_SPI1_TXCRCR_RESET_VALUE   ((uint8_t) 0x00)

  } TXCRCR;

} SPI1_t;

/// access to SPI1 SFR registers
#define sfr_SPI1   (*((SPI1_t*) 0x5200))


//------------------------
// Module SWIM
//------------------------

/** struct containing SWIM module registers */
typedef struct {

  /** SWIM control status register (CSR at 0x7f80) */
  union {

    /// bytewise access to CSR
    uint8_t  byte;

    /// skip bitwise access to register CSR

    /// register _SWIM_CSR reset value
    #define sfr_SWIM_CSR_RESET_VALUE   ((uint8_t) 0x00)

  } CSR;

} SWIM_t;

/// access to SWIM SFR registers
#define sfr_SWIM   (*((SWIM_t*) 0x7f80))


//------------------------
// Module TIM1
//------------------------

/** struct containing TIM1 module registers */
typedef struct {

  /** TIM1 control register 1 (CR1 at 0x52b0) */
  union {

    /// bytewise access to CR1
    uint8_t  byte;

    /// bitwise access to register CR1
    struct {
      BITS   CEN                 : 1;      // bit 0
      BITS   UDIS                : 1;      // bit 1
      BITS   URS                 : 1;      // bit 2
      BITS   OPM                 : 1;      // bit 3
      BITS   DIR                 : 1;      // bit 4
      BITS   CMS                 : 2;      // bits 5-6
      BITS   ARPE                : 1;      // bit 7
    };  // CR1 bitfield

    /// register _TIM1_CR1 reset value
    #define sfr_TIM1_CR1_RESET_VALUE   ((uint8_t) 0x00)

  } CR1;


  /** TIM1 control register 2 (CR2 at 0x52b1) */
  union {

    /// bytewise access to CR2
    uint8_t  byte;

    /// bitwise access to register CR2
    struct {
      BITS   CCPC                : 1;      // bit 0
      BITS                       : 1;      // 1 bit
      BITS   COMS                : 1;      // bit 2
      BITS   CCDS                : 1;      // bit 3
      BITS   MMS                 : 3;      // bits 4-6
      BITS   TI1S                : 1;      // bit 7
    };  // CR2 bitfield

    /// register _TIM1_CR2 reset value
    #define sfr_TIM1_CR2_RESET_VALUE   ((uint8_t) 0x00)

  } CR2;


  /** TIM1 Slave mode control register (SMCR at 0x52b2) */
  union {

    /// bytewise access to SMCR
    uint8_t  byte;

    /// bitwise access to register SMCR
    struct {
      BITS   SMS                 : 3;      // bits 0-2
      BITS   OCCS                : 1;      // bit 3
      BITS   TS                  : 3;      // bits 4-6
      BITS   MSM                 : 1;      // bit 7
    };  // SMCR bitfield

    /// register _TIM1_SMCR reset value
    #define sfr_TIM1_SMCR_RESET_VALUE   ((uint8_t) 0x00)

  } SMCR;


  /** TIM1 external trigger register (ETR at 0x52b3) */
  union {

    /// bytewise access to ETR
    uint8_t  byte;

    /// bitwise access to register ETR
    struct {
      BITS   ETF                 : 4;      // bits 0-3
      BITS   ETPS                : 2;      // bits 4-5
      BITS   ECE                 : 1;      // bit 6
      BITS   ETP                 : 1;      // bit 7
    };  // ETR bitfield

    /// register _TIM1_ETR reset value
    #define sfr_TIM1_ETR_RESET_VALUE   ((uint8_t) 0x00)

  } ETR;


  /** TIM1 DMA1 request enable register (DER at 0x52b4) */
  union {

    /// bytewise access to DER
    uint8_t  byte;

    /// bitwise access to register DER
    struct {
      BITS   UDE                 : 1;      // bit 0
      BITS   CC1DE               : 1;      // bit 1
      BITS   CC2DE               : 1;      // bit 2
      BITS   CC3DE               : 1;      // bit 3
      BITS   CC4DE               : 1;      // bit 4
      BITS   COMDE               : 1;      // bit 5
      BITS                       : 2;      // 2 bits
    };  // DER bitfield

    /// register _TIM1_DER reset value
    #define sfr_TIM1_DER_RESET_VALUE   ((uint8_t) 0x00)

  } DER;


  /** TIM1 Interrupt enable register (IER at 0x52b5) */
  union {

    /// bytewise access to IER
    uint8_t  byte;

    /// bitwise access to register IER
    struct {
      BITS   UIE                 : 1;      // bit 0
      BITS   CC1IE               : 1;      // bit 1
      BITS   CC2IE               : 1;      // bit 2
      BITS   CC3IE               : 1;      // bit 3
      BITS   CC4IE               : 1;      // bit 4
      BITS   COMIE               : 1;      // bit 5
      BITS   TIE                 : 1;      // bit 6
      BITS   BIE                 : 1;      // bit 7
    };  // IER bitfield

    /// register _TIM1_IER reset value
    #define sfr_TIM1_IER_RESET_VALUE   ((uint8_t) 0x00)

  } IER;


  /** TIM1 status register 1 (SR1 at 0x52b6) */
  union {

    /// bytewise access to SR1
    uint8_t  byte;

    /// bitwise access to register SR1
    struct {
      BITS   UIF                 : 1;      // bit 0
      BITS   CC1IF               : 1;      // bit 1
      BITS   CC2IF               : 1;      // bit 2
      BITS   CC3IF               : 1;      // bit 3
      BITS   CC4IF               : 1;      // bit 4
      BITS   COMIF               : 1;      // bit 5
      BITS   TIF                 : 1;      // bit 6
      BITS   BIF                 : 1;      // bit 7
    };  // SR1 bitfield

    /// register _TIM1_SR1 reset value
    #define sfr_TIM1_SR1_RESET_VALUE   ((uint8_t) 0x00)

  } SR1;


  /** TIM1 status register 2 (SR2 at 0x52b7) */
  union {

    /// bytewise access to SR2
    uint8_t  byte;

    /// bitwise access to register SR2
    struct {
      BITS                       : 1;      // 1 bit
      BITS   CC1OF               : 1;      // bit 1
      BITS   CC2OF               : 1;      // bit 2
      BITS   CC3OF               : 1;      // bit 3
      BITS   CC4OF               : 1;      // bit 4
      BITS                       : 3;      // 3 bits
    };  // SR2 bitfield

    /// register _TIM1_SR2 reset value
    #define sfr_TIM1_SR2_RESET_VALUE   ((uint8_t) 0x00)

  } SR2;


  /** TIM1 event generation register (EGR at 0x52b8) */
  union {

    /// bytewise access to EGR
    uint8_t  byte;

    /// bitwise access to register EGR
    struct {
      BITS   UG                  : 1;      // bit 0
      BITS   CC1G                : 1;      // bit 1
      BITS   CC2G                : 1;      // bit 2
      BITS   CC3G                : 1;      // bit 3
      BITS   CC4G                : 1;      // bit 4
      BITS   COMG                : 1;      // bit 5
      BITS   TG                  : 1;      // bit 6
      BITS   BG                  : 1;      // bit 7
    };  // EGR bitfield

    /// register _TIM1_EGR reset value
    #define sfr_TIM1_EGR_RESET_VALUE   ((uint8_t) 0x00)

  } EGR;


  /** TIM1 Capture/Compare mode register 1 (CCMR1 at 0x52b9) */
  union {

    /// bytewise access to CCMR1
    uint8_t  byte;

    /// bitwise access to register CCMR1
    struct {
      BITS   CC1S                : 2;      // bits 0-1
      BITS   OC1FE               : 1;      // bit 2
      BITS   OC1PE               : 1;      // bit 3
      BITS   OC1M                : 3;      // bits 4-6
      BITS   OC1CE               : 1;      // bit 7
    };  // CCMR1 bitfield

    /// register _TIM1_CCMR1 reset value
    #define sfr_TIM1_CCMR1_RESET_VALUE   ((uint8_t) 0x00)

  } CCMR1;


  /** TIM1 Capture/Compare mode register 2 (CCMR2 at 0x52ba) */
  union {

    /// bytewise access to CCMR2
    uint8_t  byte;

    /// bitwise access to register CCMR2
    struct {
      BITS   CC2S                : 2;      // bits 0-1
      BITS   OC2FE               : 1;      // bit 2
      BITS   OC2PE               : 1;      // bit 3
      BITS   OC2M                : 3;      // bits 4-6
      BITS   OC2CE               : 1;      // bit 7
    };  // CCMR2 bitfield

    /// register _TIM1_CCMR2 reset value
    #define sfr_TIM1_CCMR2_RESET_VALUE   ((uint8_t) 0x00)

  } CCMR2;


  /** TIM1 Capture/Compare mode register 3 (CCMR3 at 0x52bb) */
  union {

    /// bytewise access to CCMR3
    uint8_t  byte;

    /// bitwise access to register CCMR3
    struct {
      BITS   CC3S                : 2;      // bits 0-1
      BITS   OC3FE               : 1;      // bit 2
      BITS   OC3PE               : 1;      // bit 3
      BITS   OC3M                : 3;      // bits 4-6
      BITS   OC3CE               : 1;      // bit 7
    };  // CCMR3 bitfield

    /// register _TIM1_CCMR3 reset value
    #define sfr_TIM1_CCMR3_RESET_VALUE   ((uint8_t) 0x00)

  } CCMR3;


  /** TIM1 Capture/Compare mode register 4 (CCMR4 at 0x52bc) */
  union {

    /// bytewise access to CCMR4
    uint8_t  byte;

    /// bitwise access to register CCMR4
    struct {
      BITS   CC4S                : 2;      // bits 0-1
      BITS                       : 1;      // 1 bit
      BITS   OC4PE               : 1;      // bit 3
      BITS   OC4M                : 3;      // bits 4-6
      BITS   OC4CE               : 1;      // bit 7
    };  // CCMR4 bitfield

    /// register _TIM1_CCMR4 reset value
    #define sfr_TIM1_CCMR4_RESET_VALUE   ((uint8_t) 0x00)

  } CCMR4;


  /** TIM1 Capture/Compare enable register 1 (CCER1 at 0x52bd) */
  union {

    /// bytewise access to CCER1
    uint8_t  byte;

    /// bitwise access to register CCER1
    struct {
      BITS   CC1E                : 1;      // bit 0
      BITS   CC1P                : 1;      // bit 1
      BITS   CC1NE               : 1;      // bit 2
      BITS   CC1NP               : 1;      // bit 3
      BITS   CC2E                : 1;      // bit 4
      BITS   CC2P                : 1;      // bit 5
      BITS   CC2NE               : 1;      // bit 6
      BITS   CC2NP               : 1;      // bit 7
    };  // CCER1 bitfield

    /// register _TIM1_CCER1 reset value
    #define sfr_TIM1_CCER1_RESET_VALUE   ((uint8_t) 0x00)

  } CCER1;


  /** TIM1 Capture/Compare enable register 2 (CCER2 at 0x52be) */
  union {

    /// bytewise access to CCER2
    uint8_t  byte;

    /// bitwise access to register CCER2
    struct {
      BITS   CC3E                : 1;      // bit 0
      BITS   CC3P                : 1;      // bit 1
      BITS   CC3NE               : 1;      // bit 2
      BITS   CC3NP               : 1;      // bit 3
      BITS   CC4E                : 1;      // bit 4
      BITS   CC4P                : 1;      // bit 5
      BITS                       : 2;      // 2 bits
    };  // CCER2 bitfield

    /// register _TIM1_CCER2 reset value
    #define sfr_TIM1_CCER2_RESET_VALUE   ((uint8_t) 0x00)

  } CCER2;


  /** TIM1 counter high (CNTRH at 0x52bf) */
  union {

    /// bytewise access to CNTRH
    uint8_t  byte;

    /// bitwise access to register CNTRH
    struct {
      BITS   CNT8                : 1;      // bit 0
      BITS   CNT9                : 1;      // bit 1
      BITS   CNT10               : 1;      // bit 2
      BITS   CNT11               : 1;      // bit 3
      BITS   CNT12               : 1;      // bit 4
      BITS   CNT13               : 1;      // bit 5
      BITS   CNT14               : 1;      // bit 6
      BITS   CNT15               : 1;      // bit 7
    };  // CNTRH bitfield

    /// register _TIM1_CNTRH reset value
    #define sfr_TIM1_CNTRH_RESET_VALUE   ((uint8_t) 0x00)

  } CNTRH;


  /** TIM1 counter low (CNTRL at 0x52c0) */
  union {

    /// bytewise access to CNTRL
    uint8_t  byte;

    /// bitwise access to register CNTRL
    struct {
      BITS   CNT0                : 1;      // bit 0
      BITS   CNT1                : 1;      // bit 1
      BITS   CNT2                : 1;      // bit 2
      BITS   CNT3                : 1;      // bit 3
      BITS   CNT4                : 1;      // bit 4
      BITS   CNT5                : 1;      // bit 5
      BITS   CNT6                : 1;      // bit 6
      BITS   CNT7                : 1;      // bit 7
    };  // CNTRL bitfield

    /// register _TIM1_CNTRL reset value
    #define sfr_TIM1_CNTRL_RESET_VALUE   ((uint8_t) 0x00)

  } CNTRL;


  /** TIM1 prescaler register high (PSCRH at 0x52c1) */
  union {

    /// bytewise access to PSCRH
    uint8_t  byte;

    /// bitwise access to register PSCRH
    struct {
      BITS   PSC8                : 1;      // bit 0
      BITS   PSC9                : 1;      // bit 1
      BITS   PSC10               : 1;      // bit 2
      BITS   PSC11               : 1;      // bit 3
      BITS   PSC12               : 1;      // bit 4
      BITS   PSC13               : 1;      // bit 5
      BITS   PSC14               : 1;      // bit 6
      BITS   PSC15               : 1;      // bit 7
    };  // PSCRH bitfield

    /// register _TIM1_PSCRH reset value
    #define sfr_TIM1_PSCRH_RESET_VALUE   ((uint8_t) 0x00)

  } PSCRH;


  /** TIM1 prescaler register low (PSCRL at 0x52c2) */
  union {

    /// bytewise access to PSCRL
    uint8_t  byte;

    /// bitwise access to register PSCRL
    struct {
      BITS   PSC0                : 1;      // bit 0
      BITS   PSC1                : 1;      // bit 1
      BITS   PSC2                : 1;      // bit 2
      BITS   PSC3                : 1;      // bit 3
      BITS   PSC4                : 1;      // bit 4
      BITS   PSC5                : 1;      // bit 5
      BITS   PSC6                : 1;      // bit 6
      BITS   PSC7                : 1;      // bit 7
    };  // PSCRL bitfield

    /// register _TIM1_PSCRL reset value
    #define sfr_TIM1_PSCRL_RESET_VALUE   ((uint8_t) 0x00)

  } PSCRL;


  /** TIM1 Auto-reload register high (ARRH at 0x52c3) */
  union {

    /// bytewise access to ARRH
    uint8_t  byte;

    /// bitwise access to register ARRH
    struct {
      BITS   ARR8                : 1;      // bit 0
      BITS   ARR9                : 1;      // bit 1
      BITS   ARR10               : 1;      // bit 2
      BITS   ARR11               : 1;      // bit 3
      BITS   ARR12               : 1;      // bit 4
      BITS   ARR13               : 1;      // bit 5
      BITS   ARR14               : 1;      // bit 6
      BITS   ARR15               : 1;      // bit 7
    };  // ARRH bitfield

    /// register _TIM1_ARRH reset value
    #define sfr_TIM1_ARRH_RESET_VALUE   ((uint8_t) 0xFF)

  } ARRH;


  /** TIM1 Auto-reload register low (ARRL at 0x52c4) */
  union {

    /// bytewise access to ARRL
    uint8_t  byte;

    /// bitwise access to register ARRL
    struct {
      BITS   ARR0                : 1;      // bit 0
      BITS   ARR1                : 1;      // bit 1
      BITS   ARR2                : 1;      // bit 2
      BITS   ARR3                : 1;      // bit 3
      BITS   ARR4                : 1;      // bit 4
      BITS   ARR5                : 1;      // bit 5
      BITS   ARR6                : 1;      // bit 6
      BITS   ARR7                : 1;      // bit 7
    };  // ARRL bitfield

    /// register _TIM1_ARRL reset value
    #define sfr_TIM1_ARRL_RESET_VALUE   ((uint8_t) 0xFF)

  } ARRL;


  /** TIM1 Repetition counter register (RCR at 0x52c5) */
  union {

    /// bytewise access to RCR
    uint8_t  byte;

    /// bitwise access to register RCR
    struct {
      BITS   REP                 : 8;      // bits 0-7
    };  // RCR bitfield

    /// register _TIM1_RCR reset value
    #define sfr_TIM1_RCR_RESET_VALUE   ((uint8_t) 0x00)

  } RCR;


  /** TIM1 Capture/Compare register 1 high (CCR1H at 0x52c6) */
  union {

    /// bytewise access to CCR1H
    uint8_t  byte;

    /// bitwise access to register CCR1H
    struct {
      BITS   CCR18               : 1;      // bit 0
      BITS   CCR19               : 1;      // bit 1
      BITS   CCR110              : 1;      // bit 2
      BITS   CCR111              : 1;      // bit 3
      BITS   CCR112              : 1;      // bit 4
      BITS   CCR113              : 1;      // bit 5
      BITS   CCR114              : 1;      // bit 6
      BITS   CCR115              : 1;      // bit 7
    };  // CCR1H bitfield

    /// register _TIM1_CCR1H reset value
    #define sfr_TIM1_CCR1H_RESET_VALUE   ((uint8_t) 0x00)

  } CCR1H;


  /** TIM1 Capture/Compare register 1 low (CCR1L at 0x52c7) */
  union {

    /// bytewise access to CCR1L
    uint8_t  byte;

    /// bitwise access to register CCR1L
    struct {
      BITS   CCR10               : 1;      // bit 0
      BITS   CCR11               : 1;      // bit 1
      BITS   CCR12               : 1;      // bit 2
      BITS   CCR13               : 1;      // bit 3
      BITS   CCR14               : 1;      // bit 4
      BITS   CCR15               : 1;      // bit 5
      BITS   CCR16               : 1;      // bit 6
      BITS   CCR17               : 1;      // bit 7
    };  // CCR1L bitfield

    /// register _TIM1_CCR1L reset value
    #define sfr_TIM1_CCR1L_RESET_VALUE   ((uint8_t) 0x00)

  } CCR1L;


  /** TIM1 Capture/Compare register 2 high (CCR2H at 0x52c8) */
  union {

    /// bytewise access to CCR2H
    uint8_t  byte;

    /// bitwise access to register CCR2H
    struct {
      BITS   CCR28               : 1;      // bit 0
      BITS   CCR29               : 1;      // bit 1
      BITS   CCR210              : 1;      // bit 2
      BITS   CCR211              : 1;      // bit 3
      BITS   CCR212              : 1;      // bit 4
      BITS   CCR213              : 1;      // bit 5
      BITS   CCR214              : 1;      // bit 6
      BITS   CCR215              : 1;      // bit 7
    };  // CCR2H bitfield

    /// register _TIM1_CCR2H reset value
    #define sfr_TIM1_CCR2H_RESET_VALUE   ((uint8_t) 0x00)

  } CCR2H;


  /** TIM1 Capture/Compare register 2 low (CCR2L at 0x52c9) */
  union {

    /// bytewise access to CCR2L
    uint8_t  byte;

    /// bitwise access to register CCR2L
    struct {
      BITS   CCR20               : 1;      // bit 0
      BITS   CCR21               : 1;      // bit 1
      BITS   CCR22               : 1;      // bit 2
      BITS   CCR23               : 1;      // bit 3
      BITS   CCR24               : 1;      // bit 4
      BITS   CCR25               : 1;      // bit 5
      BITS   CCR26               : 1;      // bit 6
      BITS   CCR27               : 1;      // bit 7
    };  // CCR2L bitfield

    /// register _TIM1_CCR2L reset value
    #define sfr_TIM1_CCR2L_RESET_VALUE   ((uint8_t) 0x00)

  } CCR2L;


  /** TIM1 Capture/Compare register 3 high (CCR3H at 0x52ca) */
  union {

    /// bytewise access to CCR3H
    uint8_t  byte;

    /// bitwise access to register CCR3H
    struct {
      BITS   CCR38               : 1;      // bit 0
      BITS   CCR39               : 1;      // bit 1
      BITS   CCR310              : 1;      // bit 2
      BITS   CCR311              : 1;      // bit 3
      BITS   CCR312              : 1;      // bit 4
      BITS   CCR313              : 1;      // bit 5
      BITS   CCR314              : 1;      // bit 6
      BITS   CCR315              : 1;      // bit 7
    };  // CCR3H bitfield

    /// register _TIM1_CCR3H reset value
    #define sfr_TIM1_CCR3H_RESET_VALUE   ((uint8_t) 0x00)

  } CCR3H;


  /** TIM1 Capture/Compare register 3 low (CCR3L at 0x52cb) */
  union {

    /// bytewise access to CCR3L
    uint8_t  byte;

    /// bitwise access to register CCR3L
    struct {
      BITS   CCR30               : 1;      // bit 0
      BITS   CCR31               : 1;      // bit 1
      BITS   CCR32               : 1;      // bit 2
      BITS   CCR33               : 1;      // bit 3
      BITS   CCR34               : 1;      // bit 4
      BITS   CCR35               : 1;      // bit 5
      BITS   CCR36               : 1;      // bit 6
      BITS   CCR37               : 1;      // bit 7
    };  // CCR3L bitfield

    /// register _TIM1_CCR3L reset value
    #define sfr_TIM1_CCR3L_RESET_VALUE   ((uint8_t) 0x00)

  } CCR3L;


  /** TIM1 Capture/Compare register 4 high (CCR4H at 0x52cc) */
  union {

    /// bytewise access to CCR4H
    uint8_t  byte;

    /// bitwise access to register CCR4H
    struct {
      BITS   CCR48               : 1;      // bit 0
      BITS   CCR49               : 1;      // bit 1
      BITS   CCR410              : 1;      // bit 2
      BITS   CCR411              : 1;      // bit 3
      BITS   CCR412              : 1;      // bit 4
      BITS   CCR413              : 1;      // bit 5
      BITS   CCR414              : 1;      // bit 6
      BITS   CCR415              : 1;      // bit 7
    };  // CCR4H bitfield

    /// register _TIM1_CCR4H reset value
    #define sfr_TIM1_CCR4H_RESET_VALUE   ((uint8_t) 0x00)

  } CCR4H;


  /** TIM1 Capture/Compare register 4 low (CCR4L at 0x52cd) */
  union {

    /// bytewise access to CCR4L
    uint8_t  byte;

    /// bitwise access to register CCR4L
    struct {
      BITS   CCR40               : 1;      // bit 0
      BITS   CCR41               : 1;      // bit 1
      BITS   CCR42               : 1;      // bit 2
      BITS   CCR43               : 1;      // bit 3
      BITS   CCR44               : 1;      // bit 4
      BITS   CCR45               : 1;      // bit 5
      BITS   CCR46               : 1;      // bit 6
      BITS   CCR47               : 1;      // bit 7
    };  // CCR4L bitfield

    /// register _TIM1_CCR4L reset value
    #define sfr_TIM1_CCR4L_RESET_VALUE   ((uint8_t) 0x00)

  } CCR4L;


  /** TIM1 break register (BKR at 0x52ce) */
  union {

    /// bytewise access to BKR
    uint8_t  byte;

    /// bitwise access to register BKR
    struct {
      BITS   LOCK                : 2;      // bits 0-1
      BITS   OSSI                : 1;      // bit 2
      BITS   OSSR                : 1;      // bit 3
      BITS   BKE                 : 1;      // bit 4
      BITS   BKP                 : 1;      // bit 5
      BITS   AOE                 : 1;      // bit 6
      BITS   MOE                 : 1;      // bit 7
    };  // BKR bitfield

    /// register _TIM1_BKR reset value
    #define sfr_TIM1_BKR_RESET_VALUE   ((uint8_t) 0x00)

  } BKR;


  /** TIM1 dead-time register (DTR at 0x52cf) */
  union {

    /// bytewise access to DTR
    uint8_t  byte;

    /// bitwise access to register DTR
    struct {
      BITS   DTG                 : 8;      // bits 0-7
    };  // DTR bitfield

    /// register _TIM1_DTR reset value
    #define sfr_TIM1_DTR_RESET_VALUE   ((uint8_t) 0x00)

  } DTR;


  /** TIM1 output idle state register (OISR at 0x52d0) */
  union {

    /// bytewise access to OISR
    uint8_t  byte;

    /// bitwise access to register OISR
    struct {
      BITS   OIS1                : 1;      // bit 0
      BITS   OIS1N               : 1;      // bit 1
      BITS   OIS2                : 1;      // bit 2
      BITS   OIS2N               : 1;      // bit 3
      BITS   OIS3                : 1;      // bit 4
      BITS   OIS3N               : 1;      // bit 5
      BITS                       : 2;      // 2 bits
    };  // OISR bitfield

    /// register _TIM1_OISR reset value
    #define sfr_TIM1_OISR_RESET_VALUE   ((uint8_t) 0x00)

  } OISR;


  /** DMA1 control register 1 (DCR1 at 0x52d1) */
  union {

    /// bytewise access to DCR1
    uint8_t  byte;

    /// bitwise access to register DCR1
    struct {
      BITS   DBA                 : 5;      // bits 0-4
      BITS                       : 3;      // 3 bits
    };  // DCR1 bitfield

    /// register _TIM1_DCR1 reset value
    #define sfr_TIM1_DCR1_RESET_VALUE   ((uint8_t) 0x00)

  } DCR1;


  /** TIM1 DMA1 control register 2 (DCR2 at 0x52d2) */
  union {

    /// bytewise access to DCR2
    uint8_t  byte;

    /// bitwise access to register DCR2
    struct {
      BITS   DBL                 : 5;      // bits 0-4
      BITS                       : 3;      // 3 bits
    };  // DCR2 bitfield

    /// register _TIM1_DCR2 reset value
    #define sfr_TIM1_DCR2_RESET_VALUE   ((uint8_t) 0x00)

  } DCR2;


  /** TIM1 DMA1 address for burst mode (DMA1R at 0x52d3) */
  union {

    /// bytewise access to DMA1R
    uint8_t  byte;

    /// bitwise access to register DMA1R
    struct {
      BITS   DMAB                : 8;      // bits 0-7
    };  // DMA1R bitfield

    /// register _TIM1_DMA1R reset value
    #define sfr_TIM1_DMA1R_RESET_VALUE   ((uint8_t) 0x00)

  } DMA1R;

} TIM1_t;

/// access to TIM1 SFR registers
#define sfr_TIM1   (*((TIM1_t*) 0x52b0))


//------------------------
// Module TIM2
//------------------------

/** struct containing TIM2 module registers */
typedef struct {

  /** TIM2 control register 1 (CR1 at 0x5250) */
  union {

    /// bytewise access to CR1
    uint8_t  byte;

    /// bitwise access to register CR1
    struct {
      BITS   CEN                 : 1;      // bit 0
      BITS   UDIS                : 1;      // bit 1
      BITS   URS                 : 1;      // bit 2
      BITS   OPM                 : 1;      // bit 3
      BITS   DIR                 : 1;      // bit 4
      BITS   CMS                 : 2;      // bits 5-6
      BITS   ARPE                : 1;      // bit 7
    };  // CR1 bitfield

    /// register _TIM2_CR1 reset value
    #define sfr_TIM2_CR1_RESET_VALUE   ((uint8_t) 0x00)

  } CR1;


  /** TIM2 control register 2 (CR2 at 0x5251) */
  union {

    /// bytewise access to CR2
    uint8_t  byte;

    /// bitwise access to register CR2
    struct {
      BITS                       : 3;      // 3 bits
      BITS   CCDS                : 1;      // bit 3
      BITS   MMS                 : 3;      // bits 4-6
      BITS   TI1S                : 1;      // bit 7
    };  // CR2 bitfield

    /// register _TIM2_CR2 reset value
    #define sfr_TIM2_CR2_RESET_VALUE   ((uint8_t) 0x00)

  } CR2;


  /** TIM2 Slave mode control register (SMCR at 0x5252) */
  union {

    /// bytewise access to SMCR
    uint8_t  byte;

    /// bitwise access to register SMCR
    struct {
      BITS   SMS                 : 3;      // bits 0-2
      BITS                       : 1;      // 1 bit
      BITS   TS                  : 3;      // bits 4-6
      BITS   MSM                 : 1;      // bit 7
    };  // SMCR bitfield

    /// register _TIM2_SMCR reset value
    #define sfr_TIM2_SMCR_RESET_VALUE   ((uint8_t) 0x00)

  } SMCR;


  /** TIM2 external trigger register (ETR at 0x5253) */
  union {

    /// bytewise access to ETR
    uint8_t  byte;

    /// bitwise access to register ETR
    struct {
      BITS   ETF                 : 4;      // bits 0-3
      BITS   ETPS                : 2;      // bits 4-5
      BITS   ECE                 : 1;      // bit 6
      BITS   ETP                 : 1;      // bit 7
    };  // ETR bitfield

    /// register _TIM2_ETR reset value
    #define sfr_TIM2_ETR_RESET_VALUE   ((uint8_t) 0x00)

  } ETR;


  /** TIM2 DMA1 request enable register (DER at 0x5254) */
  union {

    /// bytewise access to DER
    uint8_t  byte;

    /// bitwise access to register DER
    struct {
      BITS   UDE                 : 1;      // bit 0
      BITS   CC1DE               : 1;      // bit 1
      BITS   CC2DE               : 1;      // bit 2
      BITS                       : 5;      // 5 bits
    };  // DER bitfield

    /// register _TIM2_DER reset value
    #define sfr_TIM2_DER_RESET_VALUE   ((uint8_t) 0x00)

  } DER;


  /** TIM2 interrupt enable register (IER at 0x5255) */
  union {

    /// bytewise access to IER
    uint8_t  byte;

    /// bitwise access to register IER
    struct {
      BITS   UIE                 : 1;      // bit 0
      BITS   CC1IE               : 1;      // bit 1
      BITS   CC2IE               : 1;      // bit 2
      BITS                       : 3;      // 3 bits
      BITS   TIE                 : 1;      // bit 6
      BITS   BIE                 : 1;      // bit 7
    };  // IER bitfield

    /// register _TIM2_IER reset value
    #define sfr_TIM2_IER_RESET_VALUE   ((uint8_t) 0x00)

  } IER;


  /** TIM2 status register 1 (SR1 at 0x5256) */
  union {

    /// bytewise access to SR1
    uint8_t  byte;

    /// bitwise access to register SR1
    struct {
      BITS   UIF                 : 1;      // bit 0
      BITS   CC1IF               : 1;      // bit 1
      BITS   CC2IF               : 1;      // bit 2
      BITS                       : 3;      // 3 bits
      BITS   TIF                 : 1;      // bit 6
      BITS   BIF                 : 1;      // bit 7
    };  // SR1 bitfield

    /// register _TIM2_SR1 reset value
    #define sfr_TIM2_SR1_RESET_VALUE   ((uint8_t) 0x00)

  } SR1;


  /** TIM2 status register 2 (SR2 at 0x5257) */
  union {

    /// bytewise access to SR2
    uint8_t  byte;

    /// bitwise access to register SR2
    struct {
      BITS                       : 1;      // 1 bit
      BITS   CC1OF               : 1;      // bit 1
      BITS   CC2OF               : 1;      // bit 2
      BITS                       : 5;      // 5 bits
    };  // SR2 bitfield

    /// register _TIM2_SR2 reset value
    #define sfr_TIM2_SR2_RESET_VALUE   ((uint8_t) 0x00)

  } SR2;


  /** TIM2 event generation register (EGR at 0x5258) */
  union {

    /// bytewise access to EGR
    uint8_t  byte;

    /// bitwise access to register EGR
    struct {
      BITS   UG                  : 1;      // bit 0
      BITS   CC1G                : 1;      // bit 1
      BITS   CC2G                : 1;      // bit 2
      BITS                       : 3;      // 3 bits
      BITS   TG                  : 1;      // bit 6
      BITS   BG                  : 1;      // bit 7
    };  // EGR bitfield

    /// register _TIM2_EGR reset value
    #define sfr_TIM2_EGR_RESET_VALUE   ((uint8_t) 0x00)

  } EGR;


  /** TIM2 capture/compare mode register 1 (CCMR1 at 0x5259) */
  union {

    /// bytewise access to CCMR1
    uint8_t  byte;

    /// bitwise access to register CCMR1
    struct {
      BITS   CC1S                : 2;      // bits 0-1
      BITS   OC1FE               : 1;      // bit 2
      BITS   OC1PE               : 1;      // bit 3
      BITS   OC1M                : 3;      // bits 4-6
      BITS                       : 1;      // 1 bit
    };  // CCMR1 bitfield

    /// register _TIM2_CCMR1 reset value
    #define sfr_TIM2_CCMR1_RESET_VALUE   ((uint8_t) 0x00)

  } CCMR1;


  /** TIM2 capture/compare mode register 2 (CCMR2 at 0x525a) */
  union {

    /// bytewise access to CCMR2
    uint8_t  byte;

    /// bitwise access to register CCMR2
    struct {
      BITS   CC2S                : 2;      // bits 0-1
      BITS   OC2FE               : 1;      // bit 2
      BITS   OC2PE               : 1;      // bit 3
      BITS   OC2M                : 3;      // bits 4-6
      BITS                       : 1;      // 1 bit
    };  // CCMR2 bitfield

    /// register _TIM2_CCMR2 reset value
    #define sfr_TIM2_CCMR2_RESET_VALUE   ((uint8_t) 0x00)

  } CCMR2;


  /** TIM2 capture/compare enable register 1 (CCER1 at 0x525b) */
  union {

    /// bytewise access to CCER1
    uint8_t  byte;

    /// bitwise access to register CCER1
    struct {
      BITS   CC1E                : 1;      // bit 0
      BITS   CC1P                : 1;      // bit 1
      BITS                       : 2;      // 2 bits
      BITS   CC2E                : 1;      // bit 4
      BITS   CC2P                : 1;      // bit 5
      BITS                       : 2;      // 2 bits
    };  // CCER1 bitfield

    /// register _TIM2_CCER1 reset value
    #define sfr_TIM2_CCER1_RESET_VALUE   ((uint8_t) 0x00)

  } CCER1;


  /** TIM2 counter high (CNTRH at 0x525c) */
  union {

    /// bytewise access to CNTRH
    uint8_t  byte;

    /// bitwise access to register CNTRH
    struct {
      BITS   CNT8                : 1;      // bit 0
      BITS   CNT9                : 1;      // bit 1
      BITS   CNT10               : 1;      // bit 2
      BITS   CNT11               : 1;      // bit 3
      BITS   CNT12               : 1;      // bit 4
      BITS   CNT13               : 1;      // bit 5
      BITS   CNT14               : 1;      // bit 6
      BITS   CNT15               : 1;      // bit 7
    };  // CNTRH bitfield

    /// register _TIM2_CNTRH reset value
    #define sfr_TIM2_CNTRH_RESET_VALUE   ((uint8_t) 0x00)

  } CNTRH;


  /** TIM2 counter low (CNTRL at 0x525d) */
  union {

    /// bytewise access to CNTRL
    uint8_t  byte;

    /// bitwise access to register CNTRL
    struct {
      BITS   CNT0                : 1;      // bit 0
      BITS   CNT1                : 1;      // bit 1
      BITS   CNT2                : 1;      // bit 2
      BITS   CNT3                : 1;      // bit 3
      BITS   CNT4                : 1;      // bit 4
      BITS   CNT5                : 1;      // bit 5
      BITS   CNT6                : 1;      // bit 6
      BITS   CNT7                : 1;      // bit 7
    };  // CNTRL bitfield

    /// register _TIM2_CNTRL reset value
    #define sfr_TIM2_CNTRL_RESET_VALUE   ((uint8_t) 0x00)

  } CNTRL;


  /** TIM2 prescaler register (PSCR at 0x525e) */
  union {

    /// bytewise access to PSCR
    uint8_t  byte;

    /// bitwise access to register PSCR
    struct {
      BITS   PSC                 : 3;      // bits 0-2
      BITS                       : 5;      // 5 bits
    };  // PSCR bitfield

    /// register _TIM2_PSCR reset value
    #define sfr_TIM2_PSCR_RESET_VALUE   ((uint8_t) 0x00)

  } PSCR;


  /** TIM2 auto-reload register high (ARRH at 0x525f) */
  union {

    /// bytewise access to ARRH
    uint8_t  byte;

    /// bitwise access to register ARRH
    struct {
      BITS   ARR8                : 1;      // bit 0
      BITS   ARR9                : 1;      // bit 1
      BITS   ARR10               : 1;      // bit 2
      BITS   ARR11               : 1;      // bit 3
      BITS   ARR12               : 1;      // bit 4
      BITS   ARR13               : 1;      // bit 5
      BITS   ARR14               : 1;      // bit 6
      BITS   ARR15               : 1;      // bit 7
    };  // ARRH bitfield

    /// register _TIM2_ARRH reset value
    #define sfr_TIM2_ARRH_RESET_VALUE   ((uint8_t) 0xFF)

  } ARRH;


  /** TIM2 auto-reload register low (ARRL at 0x5260) */
  union {

    /// bytewise access to ARRL
    uint8_t  byte;

    /// bitwise access to register ARRL
    struct {
      BITS   ARR0                : 1;      // bit 0
      BITS   ARR1                : 1;      // bit 1
      BITS   ARR2                : 1;      // bit 2
      BITS   ARR3                : 1;      // bit 3
      BITS   ARR4                : 1;      // bit 4
      BITS   ARR5                : 1;      // bit 5
      BITS   ARR6                : 1;      // bit 6
      BITS   ARR7                : 1;      // bit 7
    };  // ARRL bitfield

    /// register _TIM2_ARRL reset value
    #define sfr_TIM2_ARRL_RESET_VALUE   ((uint8_t) 0xFF)

  } ARRL;


  /** TIM2 capture/compare register 1 high (CCR1H at 0x5261) */
  union {

    /// bytewise access to CCR1H
    uint8_t  byte;

    /// bitwise access to register CCR1H
    struct {
      BITS   CCR18               : 1;      // bit 0
      BITS   CCR19               : 1;      // bit 1
      BITS   CCR110              : 1;      // bit 2
      BITS   CCR111              : 1;      // bit 3
      BITS   CCR112              : 1;      // bit 4
      BITS   CCR113              : 1;      // bit 5
      BITS   CCR114              : 1;      // bit 6
      BITS   CCR115              : 1;      // bit 7
    };  // CCR1H bitfield

    /// register _TIM2_CCR1H reset value
    #define sfr_TIM2_CCR1H_RESET_VALUE   ((uint8_t) 0x00)

  } CCR1H;


  /** TIM2 capture/compare register 1 low (CCR1L at 0x5262) */
  union {

    /// bytewise access to CCR1L
    uint8_t  byte;

    /// bitwise access to register CCR1L
    struct {
      BITS   CCR10               : 1;      // bit 0
      BITS   CCR11               : 1;      // bit 1
      BITS   CCR12               : 1;      // bit 2
      BITS   CCR13               : 1;      // bit 3
      BITS   CCR14               : 1;      // bit 4
      BITS   CCR15               : 1;      // bit 5
      BITS   CCR16               : 1;      // bit 6
      BITS   CCR17               : 1;      // bit 7
    };  // CCR1L bitfield

    /// register _TIM2_CCR1L reset value
    #define sfr_TIM2_CCR1L_RESET_VALUE   ((uint8_t) 0x00)

  } CCR1L;


  /** TIM2 capture/compare register 2 high (CCR2H at 0x5263) */
  union {

    /// bytewise access to CCR2H
    uint8_t  byte;

    /// bitwise access to register CCR2H
    struct {
      BITS   CCR28               : 1;      // bit 0
      BITS   CCR29               : 1;      // bit 1
      BITS   CCR210              : 1;      // bit 2
      BITS   CCR211              : 1;      // bit 3
      BITS   CCR212              : 1;      // bit 4
      BITS   CCR213              : 1;      // bit 5
      BITS   CCR214              : 1;      // bit 6
      BITS   CCR215              : 1;      // bit 7
    };  // CCR2H bitfield

    /// register _TIM2_CCR2H reset value
    #define sfr_TIM2_CCR2H_RESET_VALUE   ((uint8_t) 0x00)

  } CCR2H;


  /** TIM2 capture/compare register 2 low (CCR2L at 0x5264) */
  union {

    /// bytewise access to CCR2L
    uint8_t  byte;

    /// bitwise access to register CCR2L
    struct {
      BITS   CCR10               : 1;      // bit 0
      BITS   CCR11               : 1;      // bit 1
      BITS   CCR12               : 1;      // bit 2
      BITS   CCR13               : 1;      // bit 3
      BITS   CCR14               : 1;      // bit 4
      BITS   CCR15               : 1;      // bit 5
      BITS   CCR16               : 1;      // bit 6
      BITS   CCR17               : 1;      // bit 7
    };  // CCR2L bitfield

    /// register _TIM2_CCR2L reset value
    #define sfr_TIM2_CCR2L_RESET_VALUE   ((uint8_t) 0x00)

  } CCR2L;


  /** TIM2 break register (BKR at 0x5265) */
  union {

    /// bytewise access to BKR
    uint8_t  byte;

    /// bitwise access to register BKR
    struct {
      BITS   LOCK                : 2;      // bits 0-1
      BITS   OSSI                : 1;      // bit 2
      BITS                       : 1;      // 1 bit
      BITS   BKE                 : 1;      // bit 4
      BITS   BKP                 : 1;      // bit 5
      BITS   AOE                 : 1;      // bit 6
      BITS   MOE                 : 1;      // bit 7
    };  // BKR bitfield

    /// register _TIM2_BKR reset value
    #define sfr_TIM2_BKR_RESET_VALUE   ((uint8_t) 0x00)

  } BKR;


  /** TIM2 output idle state register (OISR at 0x5266) */
  union {

    /// bytewise access to OISR
    uint8_t  byte;

    /// bitwise access to register OISR
    struct {
      BITS   OIS1                : 1;      // bit 0
      BITS                       : 1;      // 1 bit
      BITS   OIS2                : 1;      // bit 2
      BITS                       : 5;      // 5 bits
    };  // OISR bitfield

    /// register _TIM2_OISR reset value
    #define sfr_TIM2_OISR_RESET_VALUE   ((uint8_t) 0x00)

  } OISR;

} TIM2_t;

/// access to TIM2 SFR registers
#define sfr_TIM2   (*((TIM2_t*) 0x5250))


//------------------------
// Module TIM3
//------------------------

/** struct containing TIM3 module registers */
typedef struct {

  /** TIM3 control register 1 (CR1 at 0x5280) */
  union {

    /// bytewise access to CR1
    uint8_t  byte;

    /// bitwise access to register CR1
    struct {
      BITS   CEN                 : 1;      // bit 0
      BITS   UDIS                : 1;      // bit 1
      BITS   URS                 : 1;      // bit 2
      BITS   OPM                 : 1;      // bit 3
      BITS   DIR                 : 1;      // bit 4
      BITS   CMS                 : 2;      // bits 5-6
      BITS   ARPE                : 1;      // bit 7
    };  // CR1 bitfield

    /// register _TIM3_CR1 reset value
    #define sfr_TIM3_CR1_RESET_VALUE   ((uint8_t) 0x00)

  } CR1;


  /** TIM3 control register 2 (CR2 at 0x5281) */
  union {

    /// bytewise access to CR2
    uint8_t  byte;

    /// bitwise access to register CR2
    struct {
      BITS                       : 3;      // 3 bits
      BITS   CCDS                : 1;      // bit 3
      BITS   MMS                 : 3;      // bits 4-6
      BITS   TI1S                : 1;      // bit 7
    };  // CR2 bitfield

    /// register _TIM3_CR2 reset value
    #define sfr_TIM3_CR2_RESET_VALUE   ((uint8_t) 0x00)

  } CR2;


  /** TIM3 Slave mode control register (SMCR at 0x5282) */
  union {

    /// bytewise access to SMCR
    uint8_t  byte;

    /// bitwise access to register SMCR
    struct {
      BITS   SMS                 : 3;      // bits 0-2
      BITS                       : 1;      // 1 bit
      BITS   TS                  : 3;      // bits 4-6
      BITS   MSM                 : 1;      // bit 7
    };  // SMCR bitfield

    /// register _TIM3_SMCR reset value
    #define sfr_TIM3_SMCR_RESET_VALUE   ((uint8_t) 0x00)

  } SMCR;


  /** TIM3 external trigger register (ETR at 0x5283) */
  union {

    /// bytewise access to ETR
    uint8_t  byte;

    /// bitwise access to register ETR
    struct {
      BITS   ETF                 : 4;      // bits 0-3
      BITS   ETPS                : 2;      // bits 4-5
      BITS   ECE                 : 1;      // bit 6
      BITS   ETP                 : 1;      // bit 7
    };  // ETR bitfield

    /// register _TIM3_ETR reset value
    #define sfr_TIM3_ETR_RESET_VALUE   ((uint8_t) 0x00)

  } ETR;


  /** TIM3 DMA1 request enable register (DER at 0x5284) */
  union {

    /// bytewise access to DER
    uint8_t  byte;

    /// bitwise access to register DER
    struct {
      BITS   UDE                 : 1;      // bit 0
      BITS   CC1DE               : 1;      // bit 1
      BITS   CC2DE               : 1;      // bit 2
      BITS                       : 5;      // 5 bits
    };  // DER bitfield

    /// register _TIM3_DER reset value
    #define sfr_TIM3_DER_RESET_VALUE   ((uint8_t) 0x00)

  } DER;


  /** TIM3 interrupt enable register (IER at 0x5285) */
  union {

    /// bytewise access to IER
    uint8_t  byte;

    /// bitwise access to register IER
    struct {
      BITS   UIE                 : 1;      // bit 0
      BITS   CC1IE               : 1;      // bit 1
      BITS   CC2IE               : 1;      // bit 2
      BITS                       : 3;      // 3 bits
      BITS   TIE                 : 1;      // bit 6
      BITS   BIE                 : 1;      // bit 7
    };  // IER bitfield

    /// register _TIM3_IER reset value
    #define sfr_TIM3_IER_RESET_VALUE   ((uint8_t) 0x00)

  } IER;


  /** TIM3 status register 1 (SR1 at 0x5286) */
  union {

    /// bytewise access to SR1
    uint8_t  byte;

    /// bitwise access to register SR1
    struct {
      BITS   UIF                 : 1;      // bit 0
      BITS   CC1IF               : 1;      // bit 1
      BITS   CC2IF               : 1;      // bit 2
      BITS                       : 3;      // 3 bits
      BITS   TIF                 : 1;      // bit 6
      BITS   BIF                 : 1;      // bit 7
    };  // SR1 bitfield

    /// register _TIM3_SR1 reset value
    #define sfr_TIM3_SR1_RESET_VALUE   ((uint8_t) 0x00)

  } SR1;


  /** TIM3 status register 2 (SR2 at 0x5287) */
  union {

    /// bytewise access to SR2
    uint8_t  byte;

    /// bitwise access to register SR2
    struct {
      BITS                       : 1;      // 1 bit
      BITS   CC1OF               : 1;      // bit 1
      BITS   CC2OF               : 1;      // bit 2
      BITS                       : 5;      // 5 bits
    };  // SR2 bitfield

    /// register _TIM3_SR2 reset value
    #define sfr_TIM3_SR2_RESET_VALUE   ((uint8_t) 0x00)

  } SR2;


  /** TIM3 event generation register (EGR at 0x5288) */
  union {

    /// bytewise access to EGR
    uint8_t  byte;

    /// bitwise access to register EGR
    struct {
      BITS   UG                  : 1;      // bit 0
      BITS   CC1G                : 1;      // bit 1
      BITS   CC2G                : 1;      // bit 2
      BITS                       : 3;      // 3 bits
      BITS   TG                  : 1;      // bit 6
      BITS   BG                  : 1;      // bit 7
    };  // EGR bitfield

    /// register _TIM3_EGR reset value
    #define sfr_TIM3_EGR_RESET_VALUE   ((uint8_t) 0x00)

  } EGR;


  /** TIM3 Capture/Compare mode register 1 (CCMR1 at 0x5289) */
  union {

    /// bytewise access to CCMR1
    uint8_t  byte;

    /// bitwise access to register CCMR1
    struct {
      BITS   CC1S                : 2;      // bits 0-1
      BITS   OC1FE               : 1;      // bit 2
      BITS   OC1PE               : 1;      // bit 3
      BITS   OC1M                : 3;      // bits 4-6
      BITS                       : 1;      // 1 bit
    };  // CCMR1 bitfield

    /// register _TIM3_CCMR1 reset value
    #define sfr_TIM3_CCMR1_RESET_VALUE   ((uint8_t) 0x00)

  } CCMR1;


  /** TIM3 Capture/Compare mode register 2 (CCMR2 at 0x528a) */
  union {

    /// bytewise access to CCMR2
    uint8_t  byte;

    /// bitwise access to register CCMR2
    struct {
      BITS   CC2S                : 2;      // bits 0-1
      BITS   OC2FE               : 1;      // bit 2
      BITS   OC2PE               : 1;      // bit 3
      BITS   OC2M                : 3;      // bits 4-6
      BITS                       : 1;      // 1 bit
    };  // CCMR2 bitfield

    /// register _TIM3_CCMR2 reset value
    #define sfr_TIM3_CCMR2_RESET_VALUE   ((uint8_t) 0x00)

  } CCMR2;


  /** TIM3 Capture/Compare enable register 1 (CCER1 at 0x528b) */
  union {

    /// bytewise access to CCER1
    uint8_t  byte;

    /// bitwise access to register CCER1
    struct {
      BITS   CC1E                : 1;      // bit 0
      BITS   CC1P                : 1;      // bit 1
      BITS                       : 2;      // 2 bits
      BITS   CC2E                : 1;      // bit 4
      BITS   CC2P                : 1;      // bit 5
      BITS                       : 2;      // 2 bits
    };  // CCER1 bitfield

    /// register _TIM3_CCER1 reset value
    #define sfr_TIM3_CCER1_RESET_VALUE   ((uint8_t) 0x00)

  } CCER1;


  /** TIM3 counter high (CNTRH at 0x528c) */
  union {

    /// bytewise access to CNTRH
    uint8_t  byte;

    /// bitwise access to register CNTRH
    struct {
      BITS   CNT8                : 1;      // bit 0
      BITS   CNT9                : 1;      // bit 1
      BITS   CNT10               : 1;      // bit 2
      BITS   CNT11               : 1;      // bit 3
      BITS   CNT12               : 1;      // bit 4
      BITS   CNT13               : 1;      // bit 5
      BITS   CNT14               : 1;      // bit 6
      BITS   CNT15               : 1;      // bit 7
    };  // CNTRH bitfield

    /// register _TIM3_CNTRH reset value
    #define sfr_TIM3_CNTRH_RESET_VALUE   ((uint8_t) 0x00)

  } CNTRH;


  /** TIM3 counter low (CNTRL at 0x528d) */
  union {

    /// bytewise access to CNTRL
    uint8_t  byte;

    /// bitwise access to register CNTRL
    struct {
      BITS   CNT0                : 1;      // bit 0
      BITS   CNT1                : 1;      // bit 1
      BITS   CNT2                : 1;      // bit 2
      BITS   CNT3                : 1;      // bit 3
      BITS   CNT4                : 1;      // bit 4
      BITS   CNT5                : 1;      // bit 5
      BITS   CNT6                : 1;      // bit 6
      BITS   CNT7                : 1;      // bit 7
    };  // CNTRL bitfield

    /// register _TIM3_CNTRL reset value
    #define sfr_TIM3_CNTRL_RESET_VALUE   ((uint8_t) 0x00)

  } CNTRL;


  /** TIM3 prescaler register (PSCR at 0x528e) */
  union {

    /// bytewise access to PSCR
    uint8_t  byte;

    /// bitwise access to register PSCR
    struct {
      BITS   PSC                 : 3;      // bits 0-2
      BITS                       : 5;      // 5 bits
    };  // PSCR bitfield

    /// register _TIM3_PSCR reset value
    #define sfr_TIM3_PSCR_RESET_VALUE   ((uint8_t) 0x00)

  } PSCR;


  /** TIM3 Auto-reload register high (ARRH at 0x528f) */
  union {

    /// bytewise access to ARRH
    uint8_t  byte;

    /// bitwise access to register ARRH
    struct {
      BITS   ARR8                : 1;      // bit 0
      BITS   ARR9                : 1;      // bit 1
      BITS   ARR10               : 1;      // bit 2
      BITS   ARR11               : 1;      // bit 3
      BITS   ARR12               : 1;      // bit 4
      BITS   ARR13               : 1;      // bit 5
      BITS   ARR14               : 1;      // bit 6
      BITS   ARR15               : 1;      // bit 7
    };  // ARRH bitfield

    /// register _TIM3_ARRH reset value
    #define sfr_TIM3_ARRH_RESET_VALUE   ((uint8_t) 0xFF)

  } ARRH;


  /** TIM3 Auto-reload register low (ARRL at 0x5290) */
  union {

    /// bytewise access to ARRL
    uint8_t  byte;

    /// bitwise access to register ARRL
    struct {
      BITS   ARR0                : 1;      // bit 0
      BITS   ARR1                : 1;      // bit 1
      BITS   ARR2                : 1;      // bit 2
      BITS   ARR3                : 1;      // bit 3
      BITS   ARR4                : 1;      // bit 4
      BITS   ARR5                : 1;      // bit 5
      BITS   ARR6                : 1;      // bit 6
      BITS   ARR7                : 1;      // bit 7
    };  // ARRL bitfield

    /// register _TIM3_ARRL reset value
    #define sfr_TIM3_ARRL_RESET_VALUE   ((uint8_t) 0xFF)

  } ARRL;


  /** TIM3 Capture/Compare register 1 high (CCR1H at 0x5291) */
  union {

    /// bytewise access to CCR1H
    uint8_t  byte;

    /// bitwise access to register CCR1H
    struct {
      BITS   CCR18               : 1;      // bit 0
      BITS   CCR19               : 1;      // bit 1
      BITS   CCR110              : 1;      // bit 2
      BITS   CCR111              : 1;      // bit 3
      BITS   CCR112              : 1;      // bit 4
      BITS   CCR113              : 1;      // bit 5
      BITS   CCR114              : 1;      // bit 6
      BITS   CCR115              : 1;      // bit 7
    };  // CCR1H bitfield

    /// register _TIM3_CCR1H reset value
    #define sfr_TIM3_CCR1H_RESET_VALUE   ((uint8_t) 0x00)

  } CCR1H;


  /** TIM3 Capture/Compare register 1 low (CCR1L at 0x5292) */
  union {

    /// bytewise access to CCR1L
    uint8_t  byte;

    /// bitwise access to register CCR1L
    struct {
      BITS   CCR10               : 1;      // bit 0
      BITS   CCR11               : 1;      // bit 1
      BITS   CCR12               : 1;      // bit 2
      BITS   CCR13               : 1;      // bit 3
      BITS   CCR14               : 1;      // bit 4
      BITS   CCR15               : 1;      // bit 5
      BITS   CCR16               : 1;      // bit 6
      BITS   CCR17               : 1;      // bit 7
    };  // CCR1L bitfield

    /// register _TIM3_CCR1L reset value
    #define sfr_TIM3_CCR1L_RESET_VALUE   ((uint8_t) 0x00)

  } CCR1L;


  /** TIM3 Capture/Compare register 2 high (CCR2H at 0x5293) */
  union {

    /// bytewise access to CCR2H
    uint8_t  byte;

    /// bitwise access to register CCR2H
    struct {
      BITS   CCR28               : 1;      // bit 0
      BITS   CCR29               : 1;      // bit 1
      BITS   CCR210              : 1;      // bit 2
      BITS   CCR211              : 1;      // bit 3
      BITS   CCR212              : 1;      // bit 4
      BITS   CCR213              : 1;      // bit 5
      BITS   CCR214              : 1;      // bit 6
      BITS   CCR215              : 1;      // bit 7
    };  // CCR2H bitfield

    /// register _TIM3_CCR2H reset value
    #define sfr_TIM3_CCR2H_RESET_VALUE   ((uint8_t) 0x00)

  } CCR2H;


  /** TIM3 Capture/Compare register 2 low (CCR2L at 0x5294) */
  union {

    /// bytewise access to CCR2L
    uint8_t  byte;

    /// bitwise access to register CCR2L
    struct {
      BITS   CCR10               : 1;      // bit 0
      BITS   CCR11               : 1;      // bit 1
      BITS   CCR12               : 1;      // bit 2
      BITS   CCR13               : 1;      // bit 3
      BITS   CCR14               : 1;      // bit 4
      BITS   CCR15               : 1;      // bit 5
      BITS   CCR16               : 1;      // bit 6
      BITS   CCR17               : 1;      // bit 7
    };  // CCR2L bitfield

    /// register _TIM3_CCR2L reset value
    #define sfr_TIM3_CCR2L_RESET_VALUE   ((uint8_t) 0x00)

  } CCR2L;


  /** TIM3 break register (BKR at 0x5295) */
  union {

    /// bytewise access to BKR
    uint8_t  byte;

    /// bitwise access to register BKR
    struct {
      BITS   LOCK                : 2;      // bits 0-1
      BITS   OSSI                : 1;      // bit 2
      BITS                       : 1;      // 1 bit
      BITS   BKE                 : 1;      // bit 4
      BITS   BKP                 : 1;      // bit 5
      BITS   AOE                 : 1;      // bit 6
      BITS   MOE                 : 1;      // bit 7
    };  // BKR bitfield

    /// register _TIM3_BKR reset value
    #define sfr_TIM3_BKR_RESET_VALUE   ((uint8_t) 0x00)

  } BKR;


  /** TIM3 output idle state register (OISR at 0x5296) */
  union {

    /// bytewise access to OISR
    uint8_t  byte;

    /// bitwise access to register OISR
    struct {
      BITS   OIS1                : 1;      // bit 0
      BITS                       : 1;      // 1 bit
      BITS   OIS2                : 1;      // bit 2
      BITS                       : 5;      // 5 bits
    };  // OISR bitfield

    /// register _TIM3_OISR reset value
    #define sfr_TIM3_OISR_RESET_VALUE   ((uint8_t) 0x00)

  } OISR;

} TIM3_t;

/// access to TIM3 SFR registers
#define sfr_TIM3   (*((TIM3_t*) 0x5280))


//------------------------
// Module TIM4
//------------------------

/** struct containing TIM4 module registers */
typedef struct {

  /** TIM4 control register 1 (CR1 at 0x52e0) */
  union {

    /// bytewise access to CR1
    uint8_t  byte;

    /// bitwise access to register CR1
    struct {
      BITS   CEN                 : 1;      // bit 0
      BITS   UDIS                : 1;      // bit 1
      BITS   URS                 : 1;      // bit 2
      BITS   OPM                 : 1;      // bit 3
      BITS                       : 3;      // 3 bits
      BITS   ARPE                : 1;      // bit 7
    };  // CR1 bitfield

    /// register _TIM4_CR1 reset value
    #define sfr_TIM4_CR1_RESET_VALUE   ((uint8_t) 0x00)

  } CR1;


  /** TIM4 control register 2 (CR2 at 0x52e1) */
  union {

    /// bytewise access to CR2
    uint8_t  byte;

    /// bitwise access to register CR2
    struct {
      BITS                       : 4;      // 4 bits
      BITS   MMS                 : 3;      // bits 4-6
      BITS                       : 1;      // 1 bit
    };  // CR2 bitfield

    /// register _TIM4_CR2 reset value
    #define sfr_TIM4_CR2_RESET_VALUE   ((uint8_t) 0x00)

  } CR2;


  /** TIM4 Slave mode control register (SMCR at 0x52e2) */
  union {

    /// bytewise access to SMCR
    uint8_t  byte;

    /// bitwise access to register SMCR
    struct {
      BITS   SMS                 : 3;      // bits 0-2
      BITS                       : 1;      // 1 bit
      BITS   TS                  : 3;      // bits 4-6
      BITS   MSM                 : 1;      // bit 7
    };  // SMCR bitfield

    /// register _TIM4_SMCR reset value
    #define sfr_TIM4_SMCR_RESET_VALUE   ((uint8_t) 0x00)

  } SMCR;


  /** TIM4 DMA1 request enable register (DER at 0x52e3) */
  union {

    /// bytewise access to DER
    uint8_t  byte;

    /// bitwise access to register DER
    struct {
      BITS   UDE                 : 1;      // bit 0
      BITS                       : 7;      // 7 bits
    };  // DER bitfield

    /// register _TIM4_DER reset value
    #define sfr_TIM4_DER_RESET_VALUE   ((uint8_t) 0x00)

  } DER;


  /** TIM4 Interrupt enable register (IER at 0x52e4) */
  union {

    /// bytewise access to IER
    uint8_t  byte;

    /// bitwise access to register IER
    struct {
      BITS   UIE                 : 1;      // bit 0
      BITS                       : 5;      // 5 bits
      BITS   TIE                 : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // IER bitfield

    /// register _TIM4_IER reset value
    #define sfr_TIM4_IER_RESET_VALUE   ((uint8_t) 0x00)

  } IER;


  /** TIM4 status register 1 (SR1 at 0x52e5) */
  union {

    /// bytewise access to SR1
    uint8_t  byte;

    /// bitwise access to register SR1
    struct {
      BITS   UIF                 : 1;      // bit 0
      BITS                       : 5;      // 5 bits
      BITS   TIF                 : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // SR1 bitfield

    /// register _TIM4_SR1 reset value
    #define sfr_TIM4_SR1_RESET_VALUE   ((uint8_t) 0x00)

  } SR1;


  /** TIM4 Event generation register (EGR at 0x52e6) */
  union {

    /// bytewise access to EGR
    uint8_t  byte;

    /// bitwise access to register EGR
    struct {
      BITS   UG                  : 1;      // bit 0
      BITS                       : 5;      // 5 bits
      BITS   TG                  : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // EGR bitfield

    /// register _TIM4_EGR reset value
    #define sfr_TIM4_EGR_RESET_VALUE   ((uint8_t) 0x00)

  } EGR;


  /** TIM4 counter (CNTR at 0x52e7) */
  union {

    /// bytewise access to CNTR
    uint8_t  byte;

    /// bitwise access to register CNTR
    struct {
      BITS   CNT0                : 1;      // bit 0
      BITS   CNT1                : 1;      // bit 1
      BITS   CNT2                : 1;      // bit 2
      BITS   CNT3                : 1;      // bit 3
      BITS   CNT4                : 1;      // bit 4
      BITS   CNT5                : 1;      // bit 5
      BITS   CNT6                : 1;      // bit 6
      BITS   CNT7                : 1;      // bit 7
    };  // CNTR bitfield

    /// register _TIM4_CNTR reset value
    #define sfr_TIM4_CNTR_RESET_VALUE   ((uint8_t) 0x00)

  } CNTR;


  /** TIM4 prescaler register (PSCR at 0x52e8) */
  union {

    /// bytewise access to PSCR
    uint8_t  byte;

    /// bitwise access to register PSCR
    struct {
      BITS   PSC                 : 4;      // bits 0-3
      BITS                       : 4;      // 4 bits
    };  // PSCR bitfield

    /// register _TIM4_PSCR reset value
    #define sfr_TIM4_PSCR_RESET_VALUE   ((uint8_t) 0x00)

  } PSCR;


  /** TIM4 Auto-reload register (ARR at 0x52e9) */
  union {

    /// bytewise access to ARR
    uint8_t  byte;

    /// bitwise access to register ARR
    struct {
      BITS   ARR0                : 1;      // bit 0
      BITS   ARR1                : 1;      // bit 1
      BITS   ARR2                : 1;      // bit 2
      BITS   ARR3                : 1;      // bit 3
      BITS   ARR4                : 1;      // bit 4
      BITS   ARR5                : 1;      // bit 5
      BITS   ARR6                : 1;      // bit 6
      BITS   ARR7                : 1;      // bit 7
    };  // ARR bitfield

    /// register _TIM4_ARR reset value
    #define sfr_TIM4_ARR_RESET_VALUE   ((uint8_t) 0x00)

  } ARR;

} TIM4_t;

/// access to TIM4 SFR registers
#define sfr_TIM4   (*((TIM4_t*) 0x52e0))


//------------------------
// Module USART1
//------------------------

/** struct containing USART1 module registers */
typedef struct {

  /** USART1 status register (SR at 0x5230) */
  union {

    /// bytewise access to SR
    uint8_t  byte;

    /// bitwise access to register SR
    struct {
      BITS   PE                  : 1;      // bit 0
      BITS   FE                  : 1;      // bit 1
      BITS   NF                  : 1;      // bit 2
      BITS   OR                  : 1;      // bit 3
      BITS   IDLE                : 1;      // bit 4
      BITS   RXNE                : 1;      // bit 5
      BITS   TC                  : 1;      // bit 6
      BITS   TXE                 : 1;      // bit 7
    };  // SR bitfield

    /// register _USART1_SR reset value
    #define sfr_USART1_SR_RESET_VALUE   ((uint8_t) 0xC0)

  } SR;


  /** USART1 data register (DR at 0x5231) */
  union {

    /// bytewise access to DR
    uint8_t  byte;

    /// bitwise access to register DR
    struct {
      BITS   DR                  : 8;      // bits 0-7
    };  // DR bitfield

    /// register _USART1_DR reset value
    #define sfr_USART1_DR_RESET_VALUE   ((uint8_t) 0x00)

  } DR;


  /** USART1 baud rate register 1 (BRR1 at 0x5232) */
  union {

    /// bytewise access to BRR1
    uint8_t  byte;

    /// bitwise access to register BRR1
    struct {
      BITS   USART_DIV4          : 1;      // bit 0
      BITS   USART_DIV5          : 1;      // bit 1
      BITS   USART_DIV6          : 1;      // bit 2
      BITS   USART_DIV7          : 1;      // bit 3
      BITS   USART_DIV8          : 1;      // bit 4
      BITS   USART_DIV9          : 1;      // bit 5
      BITS   USART_DIV10         : 1;      // bit 6
      BITS   USART_DIV11         : 1;      // bit 7
    };  // BRR1 bitfield

    /// register _USART1_BRR1 reset value
    #define sfr_USART1_BRR1_RESET_VALUE   ((uint8_t) 0x00)

  } BRR1;


  /** USART1 baud rate register 2 (BRR2 at 0x5233) */
  union {

    /// bytewise access to BRR2
    uint8_t  byte;

    /// bitwise access to register BRR2
    struct {
      BITS   USART_DIV0          : 1;      // bit 0
      BITS   USART_DIV1          : 1;      // bit 1
      BITS   USART_DIV2          : 1;      // bit 2
      BITS   USART_DIV3          : 1;      // bit 3
      BITS   USART_DIV12         : 1;      // bit 4
      BITS   USART_DIV13         : 1;      // bit 5
      BITS   USART_DIV14         : 1;      // bit 6
      BITS   USART_DIV15         : 1;      // bit 7
    };  // BRR2 bitfield

    /// register _USART1_BRR2 reset value
    #define sfr_USART1_BRR2_RESET_VALUE   ((uint8_t) 0x00)

  } BRR2;


  /** USART1 control register 1 (CR1 at 0x5234) */
  union {

    /// bytewise access to CR1
    uint8_t  byte;

    /// bitwise access to register CR1
    struct {
      BITS   PIEN                : 1;      // bit 0
      BITS   PS                  : 1;      // bit 1
      BITS   PCEN                : 1;      // bit 2
      BITS   WAKE                : 1;      // bit 3
      BITS   M                   : 1;      // bit 4
      BITS   USARTD              : 1;      // bit 5
      BITS   T8                  : 1;      // bit 6
      BITS   R8                  : 1;      // bit 7
    };  // CR1 bitfield

    /// register _USART1_CR1 reset value
    #define sfr_USART1_CR1_RESET_VALUE   ((uint8_t) 0x00)

  } CR1;


  /** USART1 control register 2 (CR2 at 0x5235) */
  union {

    /// bytewise access to CR2
    uint8_t  byte;

    /// bitwise access to register CR2
    struct {
      BITS   SBK                 : 1;      // bit 0
      BITS   RWU                 : 1;      // bit 1
      BITS   REN                 : 1;      // bit 2
      BITS   TEN                 : 1;      // bit 3
      BITS   ILIEN               : 1;      // bit 4
      BITS   RIEN                : 1;      // bit 5
      BITS   TCIEN               : 1;      // bit 6
      BITS   TIEN                : 1;      // bit 7
    };  // CR2 bitfield

    /// register _USART1_CR2 reset value
    #define sfr_USART1_CR2_RESET_VALUE   ((uint8_t) 0x00)

  } CR2;


  /** USART1 control register 3 (CR3 at 0x5236) */
  union {

    /// bytewise access to CR3
    uint8_t  byte;

    /// bitwise access to register CR3
    struct {
      BITS   LBCL                : 1;      // bit 0
      BITS   CPHA                : 1;      // bit 1
      BITS   CPOL                : 1;      // bit 2
      BITS   CLKEN               : 1;      // bit 3
      BITS   STOP0               : 1;      // bit 4
      BITS   STOP1               : 1;      // bit 5
      BITS                       : 2;      // 2 bits
    };  // CR3 bitfield

    /// register _USART1_CR3 reset value
    #define sfr_USART1_CR3_RESET_VALUE   ((uint8_t) 0x00)

  } CR3;


  /** USART1 control register 4 (CR4 at 0x5237) */
  union {

    /// bytewise access to CR4
    uint8_t  byte;

    /// bitwise access to register CR4
    struct {
      BITS   ADD0                : 1;      // bit 0
      BITS   ADD1                : 1;      // bit 1
      BITS   ADD2                : 1;      // bit 2
      BITS   ADD3                : 1;      // bit 3
      BITS                       : 4;      // 4 bits
    };  // CR4 bitfield

    /// register _USART1_CR4 reset value
    #define sfr_USART1_CR4_RESET_VALUE   ((uint8_t) 0x00)

  } CR4;


  /** USART1 control register 5 (CR5 at 0x5238) */
  union {

    /// bytewise access to CR5
    uint8_t  byte;

    /// bitwise access to register CR5
    struct {
      BITS   EIE                 : 1;      // bit 0
      BITS   IREN                : 1;      // bit 1
      BITS   IRLP                : 1;      // bit 2
      BITS   HDSEL               : 1;      // bit 3
      BITS   NACK                : 1;      // bit 4
      BITS   SCEN                : 1;      // bit 5
      BITS   DMAR                : 1;      // bit 6
      BITS   DMAT                : 1;      // bit 7
    };  // CR5 bitfield

    /// register _USART1_CR5 reset value
    #define sfr_USART1_CR5_RESET_VALUE   ((uint8_t) 0x00)

  } CR5;


  /** USART1 guard time register (GTR at 0x5239) */
  union {

    /// bytewise access to GTR
    uint8_t  byte;

    /// bitwise access to register GTR
    struct {
      BITS   GT                  : 8;      // bits 0-7
    };  // GTR bitfield

    /// register _USART1_GTR reset value
    #define sfr_USART1_GTR_RESET_VALUE   ((uint8_t) 0x00)

  } GTR;


  /** USART1 prescaler register (PSCR at 0x523a) */
  union {

    /// bytewise access to PSCR
    uint8_t  byte;

    /// bitwise access to register PSCR
    struct {
      BITS   PSC                 : 8;      // bits 0-7
    };  // PSCR bitfield

    /// register _USART1_PSCR reset value
    #define sfr_USART1_PSCR_RESET_VALUE   ((uint8_t) 0x00)

  } PSCR;

} USART1_t;

/// access to USART1 SFR registers
#define sfr_USART1   (*((USART1_t*) 0x5230))


//------------------------
// Module WFE
//------------------------

/** struct containing WFE module registers */
typedef struct {

  /** WFE control register 1 (CR1 at 0x50a6) */
  union {

    /// bytewise access to CR1
    uint8_t  byte;

    /// bitwise access to register CR1
    struct {
      BITS   TIM2_EV0            : 1;      // bit 0
      BITS   TIM2_EV1            : 1;      // bit 1
      BITS   TIM1_EV0            : 1;      // bit 2
      BITS   TIM1_EV1            : 1;      // bit 3
      BITS   EXTI_EV0            : 1;      // bit 4
      BITS   EXTI_EV1            : 1;      // bit 5
      BITS   EXTI_EV2            : 1;      // bit 6
      BITS   EXTI_EV3            : 1;      // bit 7
    };  // CR1 bitfield

    /// register _WFE_CR1 reset value
    #define sfr_WFE_CR1_RESET_VALUE   ((uint8_t) 0x00)

  } CR1;


  /** WFE control register 2 (CR2 at 0x50a7) */
  union {

    /// bytewise access to CR2
    uint8_t  byte;

    /// bitwise access to register CR2
    struct {
      BITS   EXTI_EV4            : 1;      // bit 0
      BITS   EXTI_EV5            : 1;      // bit 1
      BITS   EXTI_EV6            : 1;      // bit 2
      BITS   EXTI_EV7            : 1;      // bit 3
      BITS   EXTI_EVB            : 1;      // bit 4
      BITS   EXTI_EVD            : 1;      // bit 5
      BITS   EXTI_EVF            : 1;      // bit 6
      BITS   ADC1_COMP_EV        : 1;      // bit 7
    };  // CR2 bitfield

    /// register _WFE_CR2 reset value
    #define sfr_WFE_CR2_RESET_VALUE   ((uint8_t) 0x00)

  } CR2;


  /** WFE control register 3 (CR3 at 0x50a8) */
  union {

    /// bytewise access to CR3
    uint8_t  byte;

    /// bitwise access to register CR3
    struct {
      BITS   TIM3_EV0            : 1;      // bit 0
      BITS   TIM3_EV1            : 1;      // bit 1
      BITS   TIM4_EV             : 1;      // bit 2
      BITS   SPI1_EV             : 1;      // bit 3
      BITS   I2C1_EV             : 1;      // bit 4
      BITS   USART1_EV           : 1;      // bit 5
      BITS   DMA1CH01_EV         : 1;      // bit 6
      BITS   DMA1CH23_EV         : 1;      // bit 7
    };  // CR3 bitfield

    /// register _WFE_CR3 reset value
    #define sfr_WFE_CR3_RESET_VALUE   ((uint8_t) 0x00)

  } CR3;

} WFE_t;

/// access to WFE SFR registers
#define sfr_WFE   (*((WFE_t*) 0x50a6))


//------------------------
// Module WWDG
//------------------------

/** struct containing WWDG module registers */
typedef struct {

  /** WWDG control register (CR at 0x50d3) */
  union {

    /// bytewise access to CR
    uint8_t  byte;

    /// bitwise access to register CR
    struct {
      BITS   T0                  : 1;      // bit 0
      BITS   T1                  : 1;      // bit 1
      BITS   T2                  : 1;      // bit 2
      BITS   T3                  : 1;      // bit 3
      BITS   T4                  : 1;      // bit 4
      BITS   T5                  : 1;      // bit 5
      BITS   T6                  : 1;      // bit 6
      BITS   T7                  : 1;      // bit 7
    };  // CR bitfield

    /// register _WWDG_CR reset value
    #define sfr_WWDG_CR_RESET_VALUE   ((uint8_t) 0x7F)

  } CR;


  /** WWDR window register (WR at 0x50d4) */
  union {

    /// bytewise access to WR
    uint8_t  byte;

    /// bitwise access to register WR
    struct {
      BITS   W0                  : 1;      // bit 0
      BITS   W1                  : 1;      // bit 1
      BITS   W2                  : 1;      // bit 2
      BITS   W3                  : 1;      // bit 3
      BITS   W4                  : 1;      // bit 4
      BITS   W5                  : 1;      // bit 5
      BITS   W6                  : 1;      // bit 6
      BITS   W7                  : 1;      // bit 7
    };  // WR bitfield

    /// register _WWDG_WR reset value
    #define sfr_WWDG_WR_RESET_VALUE   ((uint8_t) 0x7F)

  } WR;

} WWDG_t;

/// access to WWDG SFR registers
#define sfr_WWDG   (*((WWDG_t*) 0x50d3))


// undefine local macros
#undef  BITS

// required for C++
#ifdef __cplusplus
  }   // extern "C"
#endif

/*-------------------------------------------------------------------------
  END OF MODULE DEFINITION FOR MULTIPLE INLUSION
-------------------------------------------------------------------------*/
#endif // STM8L152C6_H
