/*-------------------------------------------------------------------------

  STM8S105C6.h - Device Declarations

  STM8S/STM8AF, medium density with ROM bootloader

  Copyright (C) 2020, Georg Icking-Konert

  Mainstream Access line 8-bit MCU with 32 Kbytes Flash, 16 MHz CPU, integrated EEPROM 

  datasheet: https://www.st.com/resource/en/datasheet/stm8s105c6.pdf
  reference: RM0016 https://www.st.com/content/ccc/resource/technical/document/reference_manual/9a/1b/85/07/ca/eb/4f/dd/CD00190271.pdf/files/CD00190271.pdf/jcr:content/translations/en.CD00190271.pdf

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
#ifndef STM8S105C6_H
#define STM8S105C6_H

// DEVICE NAME
#define DEVICE_STM8S105C6

// DEVICE FAMILY
#define FAMILY_STM8S

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


// FLASH
#define FLASH_ADDR_START              0x008000
#define FLASH_ADDR_END                0x00FFFF
#define FLASH_SIZE                    32768


// SFR1
#define SFR1_ADDR_START               0x005000
#define SFR1_ADDR_END                 0x0057FF
#define SFR1_SIZE                     2048


// SFR2
#define SFR2_ADDR_START               0x007F00
#define SFR2_ADDR_END                 0x007FFF
#define SFR2_SIZE                     256


// BOOTROM
#define BOOTROM_ADDR_START            0x006000
#define BOOTROM_ADDR_END              0x0067FF
#define BOOTROM_SIZE                  2048


// EEPROM
#define EEPROM_ADDR_START             0x004000
#define EEPROM_ADDR_END               0x0043FF
#define EEPROM_SIZE                   1024


// OPTION
#define OPTION_ADDR_START             0x004800
#define OPTION_ADDR_END               0x00487F
#define OPTION_SIZE                   128


// MEMORY WIDTH (>32kB flash exceeds 16bit, as flash starts at 0x8000)
#define FLASH_ADDR_WIDTH            16                    ///< width of address space
#define FLASH_POINTER_T             uint16_t              ///< address variable type


/*-------------------------------------------------------------------------
  UNIQUE IDENTIFIER (size in bytes)
-------------------------------------------------------------------------*/

#define UID_ADDR_START                0x48CD                ///< start address of unique identifier
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
#define _AWU_VECTOR_                             1          ///< AWU interrupt vector: enable: AWU_CSR1.AWUEN, pending: AWU_CSR1.AWUF, priority: ITC_SPR1.VECT1SPR
#define _CLK_CSS_VECTOR_                         2          ///< CLK_CSS interrupt vector: enable: CLK_CSSR.CSSDIE, pending: CLK_CSSR.CSSD, priority: ITC_SPR1.VECT2SPR
#define _CLK_SWITCH_VECTOR_                      2          ///< CLK_SWITCH interrupt vector: enable: CLK_SWCR.SWIEN, pending: CLK_SWCR.SWIF, priority: ITC_SPR1.VECT2SPR
#define _EXTI0_VECTOR_                           3          ///< EXTI0 interrupt vector: enable: PA_CR2.C20, pending: PA_IDR.IDR0, priority: ITC_SPR1.VECT3SPR
#define _EXTI1_VECTOR_                           4          ///< EXTI1 interrupt vector: enable: PB_CR2.C20, pending: PB_IDR.IDR0, priority: ITC_SPR2.VECT4SPR
#define _EXTI2_VECTOR_                           5          ///< EXTI2 interrupt vector: enable: PC_CR2.C20, pending: PC_IDR.IDR0, priority: ITC_SPR2.VECT5SPR
#define _EXTI3_VECTOR_                           6          ///< EXTI3 interrupt vector: enable: PD_CR2.C20, pending: PD_IDR.IDR0, priority: ITC_SPR2.VECT6SPR
#define _EXTI4_VECTOR_                           7          ///< EXTI4 interrupt vector: enable: PE_CR2.C20, pending: PE_IDR.IDR0, priority: ITC_SPR2.VECT7SPR
#define _SPI_CRCERR_VECTOR_                      10         ///< SPI_CRCERR interrupt vector: enable: SPI_ICR.ERRIE, pending: SPI_SR.CRCERR, priority: ITC_SPR3.VECT10SPR
#define _SPI_MODF_VECTOR_                        10         ///< SPI_MODF interrupt vector: enable: SPI_ICR.ERRIE, pending: SPI_SR.MODF, priority: ITC_SPR3.VECT10SPR
#define _SPI_OVR_VECTOR_                         10         ///< SPI_OVR interrupt vector: enable: SPI_ICR.ERRIE, pending: SPI_SR.OVR, priority: ITC_SPR3.VECT10SPR
#define _SPI_RXNE_VECTOR_                        10         ///< SPI_RXNE interrupt vector: enable: SPI_ICR.RXIE, pending: SPI_SR.RXNE, priority: ITC_SPR3.VECT10SPR
#define _SPI_TXE_VECTOR_                         10         ///< SPI_TXE interrupt vector: enable: SPI_ICR.TXIE, pending: SPI_SR.TXE, priority: ITC_SPR3.VECT10SPR
#define _SPI_WKUP_VECTOR_                        10         ///< SPI_WKUP interrupt vector: enable: SPI_ICR.WKIE, pending: SPI_SR.WKUP, priority: ITC_SPR3.VECT10SPR
#define _TIM1_OVR_BIF_VECTOR_                    11         ///< TIM1_OVR_BIF interrupt vector: enable: TIM1_IER.BIE, pending: TIM1_SR1.BIF, priority: ITC_SPR3.VECT11SPR
#define _TIM1_OVR_TIF_VECTOR_                    11         ///< TIM1_OVR_TIF interrupt vector: enable: TIM1_IER.TIE, pending: TIM1_SR1.TIF, priority: ITC_SPR3.VECT11SPR
#define _TIM1_OVR_UIF_VECTOR_                    11         ///< TIM1_OVR_UIF interrupt vector: enable: TIM1_IER.UIE, pending: TIM1_SR1.UIF, priority: ITC_SPR3.VECT11SPR
#define _TIM1_CAPCOM_CC1IF_VECTOR_               12         ///< TIM1_CAPCOM_CC1IF interrupt vector: enable: TIM1_IER.CC1IE, pending: TIM1_SR1.CC1IF, priority: ITC_SPR4.VECT12SPR
#define _TIM1_CAPCOM_CC2IF_VECTOR_               12         ///< TIM1_CAPCOM_CC2IF interrupt vector: enable: TIM1_IER.CC2IE, pending: TIM1_SR1.CC2IF, priority: ITC_SPR4.VECT12SPR
#define _TIM1_CAPCOM_CC3IF_VECTOR_               12         ///< TIM1_CAPCOM_CC3IF interrupt vector: enable: TIM1_IER.CC3IE, pending: TIM1_SR1.CC3IF, priority: ITC_SPR4.VECT12SPR
#define _TIM1_CAPCOM_CC4IF_VECTOR_               12         ///< TIM1_CAPCOM_CC4IF interrupt vector: enable: TIM1_IER.CC4IE, pending: TIM1_SR1.CC4IF, priority: ITC_SPR4.VECT12SPR
#define _TIM1_CAPCOM_COMIF_VECTOR_               12         ///< TIM1_CAPCOM_COMIF interrupt vector: enable: TIM1_IER.COMIE, pending: TIM1_SR1.COMIF, priority: ITC_SPR4.VECT12SPR
#define _TIM2_OVR_UIF_VECTOR_                    13         ///< TIM2_OVR_UIF interrupt vector: enable: TIM2_IER.UIE, pending: TIM2_SR1.UIF, priority: ITC_SPR4.VECT13SPR
#define _TIM3_OVR_UIF_VECTOR_                    15         ///< TIM3_OVR_UIF interrupt vector: enable: TIM3_IER.UIE, pending: TIM3_SR1.UIF, priority: ITC_SPR4.VECT15SPR
#define _TIM2_CAPCOM_CC1IF_VECTOR_               14         ///< TIM2_CAPCOM_CC1IF interrupt vector: enable: TIM2_IER.CC1IE, pending: TIM2_SR1.CC1IF, priority: ITC_SPR4.VECT14SPR
#define _TIM2_CAPCOM_CC2IF_VECTOR_               14         ///< TIM2_CAPCOM_CC2IF interrupt vector: enable: TIM2_IER.CC2IE, pending: TIM2_SR1.CC2IF, priority: ITC_SPR4.VECT14SPR
#define _TIM2_CAPCOM_CC3IF_VECTOR_               14         ///< TIM2_CAPCOM_CC3IF interrupt vector: enable: TIM2_IER.CC3IE, pending: TIM2_SR1.CC3IF, priority: ITC_SPR4.VECT14SPR
#define _TIM2_CAPCOM_TIF_VECTOR_                 14         ///< TIM2_CAPCOM_TIF interrupt vector: enable: TIM2_IER.TIE, pending: TIM2_SR1.TIF, priority: ITC_SPR4.VECT14SPR
#define _TIM3_CAPCOM_CC1IF_VECTOR_               16         ///< TIM3_CAPCOM_CC1IF interrupt vector: enable: TIM3_IER.CC1IE, pending: TIM3_SR1.CC1IF, priority: ITC_SPR5.VECT16SPR
#define _TIM3_CAPCOM_CC2IF_VECTOR_               16         ///< TIM3_CAPCOM_CC2IF interrupt vector: enable: TIM3_IER.CC2IE, pending: TIM3_SR1.CC2IF, priority: ITC_SPR5.VECT16SPR
#define _TIM3_CAPCOM_CC3IF_VECTOR_               16         ///< TIM3_CAPCOM_CC3IF interrupt vector: enable: TIM3_IER.CC3IE, pending: TIM3_SR1.CC3IF, priority: ITC_SPR5.VECT16SPR
#define _TIM3_CAPCOM_TIF_VECTOR_                 16         ///< TIM3_CAPCOM_TIF interrupt vector: enable: TIM3_IER.TIE, pending: TIM3_SR1.TIF, priority: ITC_SPR5.VECT16SPR
#define _I2C_ADD10_VECTOR_                       19         ///< I2C_ADD10 interrupt vector: enable: I2C_ITR.ITEVTEN, pending: I2C_SR1.ADD10, priority: ITC_SPR5.VECT19SPR
#define _I2C_ADDR_VECTOR_                        19         ///< I2C_ADDR interrupt vector: enable: I2C_ITR.ITEVTEN, pending: I2C_SR1.ADDR, priority: ITC_SPR5.VECT19SPR
#define _I2C_AF_VECTOR_                          19         ///< I2C_AF interrupt vector: enable: I2C_ITR.ITEVTEN, pending: I2C_SR2.AF, priority: ITC_SPR5.VECT19SPR
#define _I2C_ARLO_VECTOR_                        19         ///< I2C_ARLO interrupt vector: enable: I2C_ITR.ITEVTEN, pending: I2C_SR2.ARLO, priority: ITC_SPR5.VECT19SPR
#define _I2C_BERR_VECTOR_                        19         ///< I2C_BERR interrupt vector: enable: I2C_ITR.ITEVTEN, pending: I2C_SR2.BERR, priority: ITC_SPR5.VECT19SPR
#define _I2C_BTF_VECTOR_                         19         ///< I2C_BTF interrupt vector: enable: I2C_ITR.ITEVTEN, pending: I2C_SR1.BTF, priority: ITC_SPR5.VECT19SPR
#define _I2C_OVR_VECTOR_                         19         ///< I2C_OVR interrupt vector: enable: I2C_ITR.ITEVTEN, pending: I2C_SR2.OVR, priority: ITC_SPR5.VECT19SPR
#define _I2C_RXNE_VECTOR_                        19         ///< I2C_RXNE interrupt vector: enable: I2C_ITR.ITBUFEN, pending: I2C_SR1.RXNE, priority: ITC_SPR5.VECT19SPR
#define _I2C_SB_VECTOR_                          19         ///< I2C_SB interrupt vector: enable: I2C_ITR.ITEVTEN, pending: I2C_SR1.SB, priority: ITC_SPR5.VECT19SPR
#define _I2C_STOPF_VECTOR_                       19         ///< I2C_STOPF interrupt vector: enable: I2C_ITR.ITEVTEN, pending: I2C_SR1.STOPF, priority: ITC_SPR5.VECT19SPR
#define _I2C_TXE_VECTOR_                         19         ///< I2C_TXE interrupt vector: enable: I2C_ITR.ITBUFEN, pending: I2C_SR1.TXE, priority: ITC_SPR5.VECT19SPR
#define _I2C_WUFH_VECTOR_                        19         ///< I2C_WUFH interrupt vector: enable: I2C_ITR.ITEVTEN, pending: I2C_SR2.WUFH, priority: ITC_SPR5.VECT19SPR
#define _UART2_T_TC_VECTOR_                      20         ///< UART2_T_TC interrupt vector: enable: UART2_CR2.TCIEN, pending: UART2_SR.TC, priority: ITC_SPR6.VECT20SPR
#define _UART2_T_TXE_VECTOR_                     20         ///< UART2_T_TXE interrupt vector: enable: UART2_CR2.TIEN, pending: UART2_SR.TXE, priority: ITC_SPR6.VECT20SPR
#define _UART2_R_IDLE_VECTOR_                    21         ///< UART2_R_IDLE interrupt vector: enable: UART2_CR2.ILIEN, pending: UART2_SR.IDLE, priority: ITC_SPR6.VECT21SPR
#define _UART2_R_LBDF_VECTOR_                    21         ///< UART2_R_LBDF interrupt vector: enable: UART2_CR4.LBDIEN, pending: UART2_CR4.LBDF, priority: ITC_SPR6.VECT21SPR
#define _UART2_R_OR_VECTOR_                      21         ///< UART2_R_OR interrupt vector: enable: UART2_CR2.RIEN, pending: UART2_SR.OR_LHE, priority: ITC_SPR6.VECT21SPR
#define _UART2_R_PE_VECTOR_                      21         ///< UART2_R_PE interrupt vector: enable: UART2_CR1.PIEN, pending: UART2_SR.PE, priority: ITC_SPR6.VECT21SPR
#define _UART2_R_RXNE_VECTOR_                    21         ///< UART2_R_RXNE interrupt vector: enable: UART2_CR2.RIEN, pending: UART2_SR.RXNE, priority: ITC_SPR6.VECT21SPR
#define _ADC1_AWDG_VECTOR_                       22         ///< ADC1_AWDG interrupt vector: enable: ADC_CSR.AWDIE, pending: ADC_CSR.AWD, priority: ITC_SPR6.VECT22SPR
#define _ADC1_AWS0_VECTOR_                       22         ///< ADC1_AWS0 interrupt vector: enable: ADC_AWCRL.AWEN0, pending: ADC_AWSRL.AWS0, priority: ITC_SPR6.VECT22SPR
#define _ADC1_AWS1_VECTOR_                       22         ///< ADC1_AWS1 interrupt vector: enable: ADC_AWCRL.AWEN1, pending: ADC_AWSRL.AWS1, priority: ITC_SPR6.VECT22SPR
#define _ADC1_AWS2_VECTOR_                       22         ///< ADC1_AWS2 interrupt vector: enable: ADC_AWCRL.AWEN2, pending: ADC_AWSRL.AWS2, priority: ITC_SPR6.VECT22SPR
#define _ADC1_AWS3_VECTOR_                       22         ///< ADC1_AWS3 interrupt vector: enable: ADC_AWCRL.AWEN3, pending: ADC_AWSRL.AWS3, priority: ITC_SPR6.VECT22SPR
#define _ADC1_AWS4_VECTOR_                       22         ///< ADC1_AWS4 interrupt vector: enable: ADC_AWCRL.AWEN4, pending: ADC_AWSRL.AWS4, priority: ITC_SPR6.VECT22SPR
#define _ADC1_AWS5_VECTOR_                       22         ///< ADC1_AWS5 interrupt vector: enable: ADC_AWCRL.AWEN5, pending: ADC_AWSRL.AWS5, priority: ITC_SPR6.VECT22SPR
#define _ADC1_AWS6_VECTOR_                       22         ///< ADC1_AWS6 interrupt vector: enable: ADC_AWCRL.AWEN6, pending: ADC_AWSRL.AWS6, priority: ITC_SPR6.VECT22SPR
#define _ADC1_AWS7_VECTOR_                       22         ///< ADC1_AWS7 interrupt vector: enable: ADC_AWCRL.AWEN7, pending: ADC_AWSRL.AWS7, priority: ITC_SPR6.VECT22SPR
#define _ADC1_AWS8_VECTOR_                       22         ///< ADC1_AWS8 interrupt vector: enable: ADC_AWCRH.AWEN8, pending: ADC_AWSRH.AWS8, priority: ITC_SPR6.VECT22SPR
#define _ADC1_AWS9_VECTOR_                       22         ///< ADC1_AWS9 interrupt vector: enable: ADC_AWCRH.AWEN9, pending: ADC_AWSRH.AWS9, priority: ITC_SPR6.VECT22SPR
#define _ADC1_EOC_VECTOR_                        22         ///< ADC1_EOC interrupt vector: enable: ADC_CSR.EOCIE, pending: ADC_CSR.EOC, priority: ITC_SPR6.VECT22SPR
#define _TIM4_OVR_UIF_VECTOR_                    23         ///< TIM4_OVR_UIF interrupt vector: enable: TIM4_IER.UIE, pending: TIM4_SR.UIF, priority: ITC_SPR6.VECT23SPR
#define _FLASH_EOP_VECTOR_                       24         ///< FLASH_EOP interrupt vector: enable: FLASH_CR1.IE, pending: FLASH_IAPSR.EOP, priority: ITC_SPR6.VECT24SPR
#define _FLASH_WR_PG_DIS_VECTOR_                 24         ///< FLASH_WR_PG_DIS interrupt vector: enable: FLASH_CR1.IE, pending: FLASH_IAPSR.WR_PG_DIS, priority: ITC_SPR6.VECT24SPR


/*-------------------------------------------------------------------------
  DEFINITION OF STM8 PERIPHERAL REGISTERS
-------------------------------------------------------------------------*/

//------------------------
// Module ADC1
//------------------------

/** struct containing ADC1 module registers */
typedef struct {

  /** ADC data buffer registers (DB0RH at 0x53e0) */
  union {

    /// bytewise access to DB0RH
    uint8_t  byte;

    /// bitwise access to register DB0RH
    struct {
      BITS   DBH                 : 8;      // bits 0-7
    };  // DB0RH bitfield

    /// register _ADC1_DB0RH reset value
    #define sfr_ADC1_DB0RH_RESET_VALUE   ((uint8_t) 0x00)

  } DB0RH;


  /** ADC data buffer registers (DB0RL at 0x53e1) */
  union {

    /// bytewise access to DB0RL
    uint8_t  byte;

    /// bitwise access to register DB0RL
    struct {
      BITS   DL                  : 8;      // bits 0-7
    };  // DB0RL bitfield

    /// register _ADC1_DB0RL reset value
    #define sfr_ADC1_DB0RL_RESET_VALUE   ((uint8_t) 0x00)

  } DB0RL;


  /** ADC data buffer registers (DB1RH at 0x53e2) */
  union {

    /// bytewise access to DB1RH
    uint8_t  byte;

    /// bitwise access to register DB1RH
    struct {
      BITS   DBH                 : 8;      // bits 0-7
    };  // DB1RH bitfield

    /// register _ADC1_DB1RH reset value
    #define sfr_ADC1_DB1RH_RESET_VALUE   ((uint8_t) 0x00)

  } DB1RH;


  /** ADC data buffer registers (DB1RL at 0x53e3) */
  union {

    /// bytewise access to DB1RL
    uint8_t  byte;

    /// bitwise access to register DB1RL
    struct {
      BITS   DL                  : 8;      // bits 0-7
    };  // DB1RL bitfield

    /// register _ADC1_DB1RL reset value
    #define sfr_ADC1_DB1RL_RESET_VALUE   ((uint8_t) 0x00)

  } DB1RL;


  /** ADC data buffer registers (DB2RH at 0x53e4) */
  union {

    /// bytewise access to DB2RH
    uint8_t  byte;

    /// bitwise access to register DB2RH
    struct {
      BITS   DBH                 : 8;      // bits 0-7
    };  // DB2RH bitfield

    /// register _ADC1_DB2RH reset value
    #define sfr_ADC1_DB2RH_RESET_VALUE   ((uint8_t) 0x00)

  } DB2RH;


  /** ADC data buffer registers (DB2RL at 0x53e5) */
  union {

    /// bytewise access to DB2RL
    uint8_t  byte;

    /// bitwise access to register DB2RL
    struct {
      BITS   DL                  : 8;      // bits 0-7
    };  // DB2RL bitfield

    /// register _ADC1_DB2RL reset value
    #define sfr_ADC1_DB2RL_RESET_VALUE   ((uint8_t) 0x00)

  } DB2RL;


  /** ADC data buffer registers (DB3RH at 0x53e6) */
  union {

    /// bytewise access to DB3RH
    uint8_t  byte;

    /// bitwise access to register DB3RH
    struct {
      BITS   DBH                 : 8;      // bits 0-7
    };  // DB3RH bitfield

    /// register _ADC1_DB3RH reset value
    #define sfr_ADC1_DB3RH_RESET_VALUE   ((uint8_t) 0x00)

  } DB3RH;


  /** ADC data buffer registers (DB3RL at 0x53e7) */
  union {

    /// bytewise access to DB3RL
    uint8_t  byte;

    /// bitwise access to register DB3RL
    struct {
      BITS   DL                  : 8;      // bits 0-7
    };  // DB3RL bitfield

    /// register _ADC1_DB3RL reset value
    #define sfr_ADC1_DB3RL_RESET_VALUE   ((uint8_t) 0x00)

  } DB3RL;


  /** ADC data buffer registers (DB4RH at 0x53e8) */
  union {

    /// bytewise access to DB4RH
    uint8_t  byte;

    /// bitwise access to register DB4RH
    struct {
      BITS   DBH                 : 8;      // bits 0-7
    };  // DB4RH bitfield

    /// register _ADC1_DB4RH reset value
    #define sfr_ADC1_DB4RH_RESET_VALUE   ((uint8_t) 0x00)

  } DB4RH;


  /** ADC data buffer registers (DB4RL at 0x53e9) */
  union {

    /// bytewise access to DB4RL
    uint8_t  byte;

    /// bitwise access to register DB4RL
    struct {
      BITS   DL                  : 8;      // bits 0-7
    };  // DB4RL bitfield

    /// register _ADC1_DB4RL reset value
    #define sfr_ADC1_DB4RL_RESET_VALUE   ((uint8_t) 0x00)

  } DB4RL;


  /** ADC data buffer registers (DB5RH at 0x53ea) */
  union {

    /// bytewise access to DB5RH
    uint8_t  byte;

    /// bitwise access to register DB5RH
    struct {
      BITS   DBH                 : 8;      // bits 0-7
    };  // DB5RH bitfield

    /// register _ADC1_DB5RH reset value
    #define sfr_ADC1_DB5RH_RESET_VALUE   ((uint8_t) 0x00)

  } DB5RH;


  /** ADC data buffer registers (DB5RL at 0x53eb) */
  union {

    /// bytewise access to DB5RL
    uint8_t  byte;

    /// bitwise access to register DB5RL
    struct {
      BITS   DL                  : 8;      // bits 0-7
    };  // DB5RL bitfield

    /// register _ADC1_DB5RL reset value
    #define sfr_ADC1_DB5RL_RESET_VALUE   ((uint8_t) 0x00)

  } DB5RL;


  /** ADC data buffer registers (DB6RH at 0x53ec) */
  union {

    /// bytewise access to DB6RH
    uint8_t  byte;

    /// bitwise access to register DB6RH
    struct {
      BITS   DBH                 : 8;      // bits 0-7
    };  // DB6RH bitfield

    /// register _ADC1_DB6RH reset value
    #define sfr_ADC1_DB6RH_RESET_VALUE   ((uint8_t) 0x00)

  } DB6RH;


  /** ADC data buffer registers (DB6RL at 0x53ed) */
  union {

    /// bytewise access to DB6RL
    uint8_t  byte;

    /// bitwise access to register DB6RL
    struct {
      BITS   DL                  : 8;      // bits 0-7
    };  // DB6RL bitfield

    /// register _ADC1_DB6RL reset value
    #define sfr_ADC1_DB6RL_RESET_VALUE   ((uint8_t) 0x00)

  } DB6RL;


  /** ADC data buffer registers (DB7RH at 0x53ee) */
  union {

    /// bytewise access to DB7RH
    uint8_t  byte;

    /// bitwise access to register DB7RH
    struct {
      BITS   DBH                 : 8;      // bits 0-7
    };  // DB7RH bitfield

    /// register _ADC1_DB7RH reset value
    #define sfr_ADC1_DB7RH_RESET_VALUE   ((uint8_t) 0x00)

  } DB7RH;


  /** ADC data buffer registers (DB7RL at 0x53ef) */
  union {

    /// bytewise access to DB7RL
    uint8_t  byte;

    /// bitwise access to register DB7RL
    struct {
      BITS   DL                  : 8;      // bits 0-7
    };  // DB7RL bitfield

    /// register _ADC1_DB7RL reset value
    #define sfr_ADC1_DB7RL_RESET_VALUE   ((uint8_t) 0x00)

  } DB7RL;


  /** ADC data buffer registers (DB8RH at 0x53f0) */
  union {

    /// bytewise access to DB8RH
    uint8_t  byte;

    /// bitwise access to register DB8RH
    struct {
      BITS   DBH                 : 8;      // bits 0-7
    };  // DB8RH bitfield

    /// register _ADC1_DB8RH reset value
    #define sfr_ADC1_DB8RH_RESET_VALUE   ((uint8_t) 0x00)

  } DB8RH;


  /** ADC data buffer registers (DB8RL at 0x53f1) */
  union {

    /// bytewise access to DB8RL
    uint8_t  byte;

    /// bitwise access to register DB8RL
    struct {
      BITS   DL                  : 8;      // bits 0-7
    };  // DB8RL bitfield

    /// register _ADC1_DB8RL reset value
    #define sfr_ADC1_DB8RL_RESET_VALUE   ((uint8_t) 0x00)

  } DB8RL;


  /** ADC data buffer registers (DB9RH at 0x53f2) */
  union {

    /// bytewise access to DB9RH
    uint8_t  byte;

    /// bitwise access to register DB9RH
    struct {
      BITS   DBH                 : 8;      // bits 0-7
    };  // DB9RH bitfield

    /// register _ADC1_DB9RH reset value
    #define sfr_ADC1_DB9RH_RESET_VALUE   ((uint8_t) 0x00)

  } DB9RH;


  /** ADC data buffer registers (DB9RL at 0x53f3) */
  union {

    /// bytewise access to DB9RL
    uint8_t  byte;

    /// bitwise access to register DB9RL
    struct {
      BITS   DL                  : 8;      // bits 0-7
    };  // DB9RL bitfield

    /// register _ADC1_DB9RL reset value
    #define sfr_ADC1_DB9RL_RESET_VALUE   ((uint8_t) 0x00)

  } DB9RL;


  /// Reserved register (12B)
  uint8_t     Reserved_1[12];


  /** ADC control/status register (CSR at 0x5400) */
  union {

    /// bytewise access to CSR
    uint8_t  byte;

    /// bitwise access to register CSR
    struct {
      BITS   CH                  : 4;      // bits 0-3
      BITS   AWDIE               : 1;      // bit 4
      BITS   EOCIE               : 1;      // bit 5
      BITS   AWD                 : 1;      // bit 6
      BITS   EOC                 : 1;      // bit 7
    };  // CSR bitfield

    /// register _ADC1_CSR reset value
    #define sfr_ADC1_CSR_RESET_VALUE   ((uint8_t) 0x00)

  } CSR;


  /** ADC configuration register 1 (CR1 at 0x5401) */
  union {

    /// bytewise access to CR1
    uint8_t  byte;

    /// bitwise access to register CR1
    struct {
      BITS   ADON                : 1;      // bit 0
      BITS   CONT                : 1;      // bit 1
      BITS                       : 2;      // 2 bits
      BITS   SPSEL               : 3;      // bits 4-6
      BITS                       : 1;      // 1 bit
    };  // CR1 bitfield

    /// register _ADC1_CR1 reset value
    #define sfr_ADC1_CR1_RESET_VALUE   ((uint8_t) 0x00)

  } CR1;


  /** ADC configuration register 2 (CR2 at 0x5402) */
  union {

    /// bytewise access to CR2
    uint8_t  byte;

    /// bitwise access to register CR2
    struct {
      BITS                       : 1;      // 1 bit
      BITS   SCAN                : 1;      // bit 1
      BITS                       : 1;      // 1 bit
      BITS   ALIGN               : 1;      // bit 3
      BITS   EXTSEL              : 2;      // bits 4-5
      BITS   EXTTRIG             : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // CR2 bitfield

    /// register _ADC1_CR2 reset value
    #define sfr_ADC1_CR2_RESET_VALUE   ((uint8_t) 0x00)

  } CR2;


  /** ADC configuration register 3 (CR3 at 0x5403) */
  union {

    /// bytewise access to CR3
    uint8_t  byte;

    /// bitwise access to register CR3
    struct {
      BITS                       : 6;      // 6 bits
      BITS   OVR                 : 1;      // bit 6
      BITS   DBUF                : 1;      // bit 7
    };  // CR3 bitfield

    /// register _ADC1_CR3 reset value
    #define sfr_ADC1_CR3_RESET_VALUE   ((uint8_t) 0x00)

  } CR3;


  /** ADC data register high (DRH at 0x5404) */
  union {

    /// bytewise access to DRH
    uint8_t  byte;

    /// bitwise access to register DRH
    struct {
      BITS   DH                  : 8;      // bits 0-7
    };  // DRH bitfield

    /// register _ADC1_DRH reset value
    #define sfr_ADC1_DRH_RESET_VALUE   ((uint8_t) 0x00)

  } DRH;


  /** ADC data register low (DRL at 0x5405) */
  union {

    /// bytewise access to DRL
    uint8_t  byte;

    /// bitwise access to register DRL
    struct {
      BITS   DL                  : 8;      // bits 0-7
    };  // DRL bitfield

    /// register _ADC1_DRL reset value
    #define sfr_ADC1_DRL_RESET_VALUE   ((uint8_t) 0x00)

  } DRL;


  /** ADC Schmitt trigger disable register high (TDRH at 0x5406) */
  union {

    /// bytewise access to TDRH
    uint8_t  byte;

    /// bitwise access to register TDRH
    struct {
      BITS   TD                  : 8;      // bits 0-7
    };  // TDRH bitfield

    /// register _ADC1_TDRH reset value
    #define sfr_ADC1_TDRH_RESET_VALUE   ((uint8_t) 0x00)

  } TDRH;


  /** ADC Schmitt trigger disable register low (TDRL at 0x5407) */
  union {

    /// bytewise access to TDRL
    uint8_t  byte;

    /// bitwise access to register TDRL
    struct {
      BITS   TL                  : 8;      // bits 0-7
    };  // TDRL bitfield

    /// register _ADC1_TDRL reset value
    #define sfr_ADC1_TDRL_RESET_VALUE   ((uint8_t) 0x00)

  } TDRL;


  /** ADC high threshold register high (HTRH at 0x5408) */
  union {

    /// bytewise access to HTRH
    uint8_t  byte;

    /// bitwise access to register HTRH
    struct {
      BITS   HT                  : 8;      // bits 0-7
    };  // HTRH bitfield

    /// register _ADC1_HTRH reset value
    #define sfr_ADC1_HTRH_RESET_VALUE   ((uint8_t) 0x03)

  } HTRH;


  /** ADC high threshold register low (HTRL at 0x5409) */
  union {

    /// bytewise access to HTRL
    uint8_t  byte;

    /// bitwise access to register HTRL
    struct {
      BITS   HT                  : 2;      // bits 0-1
      BITS                       : 6;      // 6 bits
    };  // HTRL bitfield

    /// register _ADC1_HTRL reset value
    #define sfr_ADC1_HTRL_RESET_VALUE   ((uint8_t) 0xFF)

  } HTRL;


  /** ADC low threshold register high (LTRH at 0x540a) */
  union {

    /// bytewise access to LTRH
    uint8_t  byte;

    /// bitwise access to register LTRH
    struct {
      BITS   LT                  : 8;      // bits 0-7
    };  // LTRH bitfield

    /// register _ADC1_LTRH reset value
    #define sfr_ADC1_LTRH_RESET_VALUE   ((uint8_t) 0x00)

  } LTRH;


  /** ADC low threshold register low (LTRL at 0x540b) */
  union {

    /// bytewise access to LTRL
    uint8_t  byte;

    /// bitwise access to register LTRL
    struct {
      BITS   LT                  : 2;      // bits 0-1
      BITS                       : 6;      // 6 bits
    };  // LTRL bitfield

    /// register _ADC1_LTRL reset value
    #define sfr_ADC1_LTRL_RESET_VALUE   ((uint8_t) 0x00)

  } LTRL;


  /** ADC analog watchdog status register high (AWSRH at 0x540c) */
  union {

    /// bytewise access to AWSRH
    uint8_t  byte;

    /// bitwise access to register AWSRH
    struct {
      BITS   AWS8                : 1;      // bit 0
      BITS   AWS9                : 1;      // bit 1
      BITS                       : 6;      // 6 bits
    };  // AWSRH bitfield

    /// register _ADC1_AWSRH reset value
    #define sfr_ADC1_AWSRH_RESET_VALUE   ((uint8_t) 0x00)

  } AWSRH;


  /** ADC analog watchdog status register low (AWSRL at 0x540d) */
  union {

    /// bytewise access to AWSRL
    uint8_t  byte;

    /// bitwise access to register AWSRL
    struct {
      BITS   AWS0                : 1;      // bit 0
      BITS   AWS1                : 1;      // bit 1
      BITS   AWS2                : 1;      // bit 2
      BITS   AWS3                : 1;      // bit 3
      BITS   AWS4                : 1;      // bit 4
      BITS   AWS5                : 1;      // bit 5
      BITS   AWS6                : 1;      // bit 6
      BITS   AWS7                : 1;      // bit 7
    };  // AWSRL bitfield

    /// register _ADC1_AWSRL reset value
    #define sfr_ADC1_AWSRL_RESET_VALUE   ((uint8_t) 0x00)

  } AWSRL;


  /** ADC analog watchdog control register high (AWCRH at 0x540e) */
  union {

    /// bytewise access to AWCRH
    uint8_t  byte;

    /// bitwise access to register AWCRH
    struct {
      BITS   AWEN8               : 1;      // bit 0
      BITS   AWEN9               : 1;      // bit 1
      BITS                       : 6;      // 6 bits
    };  // AWCRH bitfield

    /// register _ADC1_AWCRH reset value
    #define sfr_ADC1_AWCRH_RESET_VALUE   ((uint8_t) 0x00)

  } AWCRH;


  /** ADC analog watchdog control register low (AWCRL at 0x540f) */
  union {

    /// bytewise access to AWCRL
    uint8_t  byte;

    /// bitwise access to register AWCRL
    struct {
      BITS   AWEN0               : 1;      // bit 0
      BITS   AWEN1               : 1;      // bit 1
      BITS   AWEN2               : 1;      // bit 2
      BITS   AWEN3               : 1;      // bit 3
      BITS   AWEN4               : 1;      // bit 4
      BITS   AWEN5               : 1;      // bit 5
      BITS   AWEN6               : 1;      // bit 6
      BITS   AWEN7               : 1;      // bit 7
    };  // AWCRL bitfield

    /// register _ADC1_AWCRL reset value
    #define sfr_ADC1_AWCRL_RESET_VALUE   ((uint8_t) 0x00)

  } AWCRL;

} ADC1_t;

/// access to ADC1 SFR registers
#define sfr_ADC1   (*((ADC1_t*) 0x53e0))


//------------------------
// Module AWU
//------------------------

/** struct containing AWU module registers */
typedef struct {

  /** AWU control/status register 1 (CSR1 at 0x50f0) */
  union {

    /// bytewise access to CSR1
    uint8_t  byte;

    /// bitwise access to register CSR1
    struct {
      BITS   MSR                 : 1;      // bit 0
      BITS                       : 3;      // 3 bits
      BITS   AWUEN               : 1;      // bit 4
      BITS   AWUF                : 1;      // bit 5
      BITS                       : 2;      // 2 bits
    };  // CSR1 bitfield

    /// register _AWU_CSR1 reset value
    #define sfr_AWU_CSR1_RESET_VALUE   ((uint8_t) 0x00)

  } CSR1;


  /** AWU asynchronous prescaler buffer register (APR at 0x50f1) */
  union {

    /// bytewise access to APR
    uint8_t  byte;

    /// bitwise access to register APR
    struct {
      BITS   APR                 : 6;      // bits 0-5
      BITS                       : 2;      // 2 bits
    };  // APR bitfield

    /// register _AWU_APR reset value
    #define sfr_AWU_APR_RESET_VALUE   ((uint8_t) 0x3F)

  } APR;


  /** AWU timebase selection register (TBR at 0x50f2) */
  union {

    /// bytewise access to TBR
    uint8_t  byte;

    /// bitwise access to register TBR
    struct {
      BITS   AWUTB               : 4;      // bits 0-3
      BITS                       : 4;      // 4 bits
    };  // TBR bitfield

    /// register _AWU_TBR reset value
    #define sfr_AWU_TBR_RESET_VALUE   ((uint8_t) 0x00)

  } TBR;

} AWU_t;

/// access to AWU SFR registers
#define sfr_AWU   (*((AWU_t*) 0x50f0))


//------------------------
// Module BEEP
//------------------------

/** struct containing BEEP module registers */
typedef struct {

  /** BEEP control/status register (CSR at 0x50f3) */
  union {

    /// bytewise access to CSR
    uint8_t  byte;

    /// bitwise access to register CSR
    struct {
      BITS   BEEPDIV             : 5;      // bits 0-4
      BITS   BEEPEN              : 1;      // bit 5
      BITS   BEEPSEL             : 2;      // bits 6-7
    };  // CSR bitfield

    /// register _BEEP_CSR reset value
    #define sfr_BEEP_CSR_RESET_VALUE   ((uint8_t) 0x1F)

  } CSR;

} BEEP_t;

/// access to BEEP SFR registers
#define sfr_BEEP   (*((BEEP_t*) 0x50f3))


//------------------------
// Module CLK
//------------------------

/** struct containing CLK module registers */
typedef struct {

  /** Internal clock control register (ICKR at 0x50c0) */
  union {

    /// bytewise access to ICKR
    uint8_t  byte;

    /// bitwise access to register ICKR
    struct {
      BITS   HSIEN               : 1;      // bit 0
      BITS   HSIRDY              : 1;      // bit 1
      BITS   FHW                 : 1;      // bit 2
      BITS   LSIEN               : 1;      // bit 3
      BITS   LSIRDY              : 1;      // bit 4
      BITS   REGAH               : 1;      // bit 5
      BITS                       : 2;      // 2 bits
    };  // ICKR bitfield

    /// register _CLK_ICKR reset value
    #define sfr_CLK_ICKR_RESET_VALUE   ((uint8_t) 0x01)

  } ICKR;


  /** External clock control register (ECKR at 0x50c1) */
  union {

    /// bytewise access to ECKR
    uint8_t  byte;

    /// bitwise access to register ECKR
    struct {
      BITS   HSEEN               : 1;      // bit 0
      BITS   HSERDY              : 1;      // bit 1
      BITS                       : 6;      // 6 bits
    };  // ECKR bitfield

    /// register _CLK_ECKR reset value
    #define sfr_CLK_ECKR_RESET_VALUE   ((uint8_t) 0x00)

  } ECKR;


  /// Reserved register (1B)
  uint8_t     Reserved_1[1];


  /** Clock master status register (CMSR at 0x50c3) */
  union {

    /// bytewise access to CMSR
    uint8_t  byte;

    /// bitwise access to register CMSR
    struct {
      BITS   CKM                 : 8;      // bits 0-7
    };  // CMSR bitfield

    /// register _CLK_CMSR reset value
    #define sfr_CLK_CMSR_RESET_VALUE   ((uint8_t) 0xE1)

  } CMSR;


  /** Clock master switch register (SWR at 0x50c4) */
  union {

    /// bytewise access to SWR
    uint8_t  byte;

    /// bitwise access to register SWR
    struct {
      BITS   SWI                 : 8;      // bits 0-7
    };  // SWR bitfield

    /// register _CLK_SWR reset value
    #define sfr_CLK_SWR_RESET_VALUE   ((uint8_t) 0xE1)

  } SWR;


  /** Clock switch control register (SWCR at 0x50c5) */
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


  /** Clock divider register (CKDIVR at 0x50c6) */
  union {

    /// bytewise access to CKDIVR
    uint8_t  byte;

    /// bitwise access to register CKDIVR
    struct {
      BITS   CPUDIV              : 3;      // bits 0-2
      BITS   HSIDIV              : 2;      // bits 3-4
      BITS                       : 3;      // 3 bits
    };  // CKDIVR bitfield

    /// register _CLK_CKDIVR reset value
    #define sfr_CLK_CKDIVR_RESET_VALUE   ((uint8_t) 0x18)

  } CKDIVR;


  /** Peripheral clock gating register 1 (PCKENR1 at 0x50c7) */
  union {

    /// bytewise access to PCKENR1
    uint8_t  byte;

    /// bitwise access to register PCKENR1
    struct {
      BITS   PCKEN               : 8;      // bits 0-7
    };  // PCKENR1 bitfield

    /// register _CLK_PCKENR1 reset value
    #define sfr_CLK_PCKENR1_RESET_VALUE   ((uint8_t) 0xFF)

  } PCKENR1;


  /** Clock security system register (CSSR at 0x50c8) */
  union {

    /// bytewise access to CSSR
    uint8_t  byte;

    /// bitwise access to register CSSR
    struct {
      BITS   CSSEN               : 1;      // bit 0
      BITS   AUX                 : 1;      // bit 1
      BITS   CSSDIE              : 1;      // bit 2
      BITS   CSSD                : 1;      // bit 3
      BITS                       : 4;      // 4 bits
    };  // CSSR bitfield

    /// register _CLK_CSSR reset value
    #define sfr_CLK_CSSR_RESET_VALUE   ((uint8_t) 0x00)

  } CSSR;


  /** Configurable clock control register (CCOR at 0x50c9) */
  union {

    /// bytewise access to CCOR
    uint8_t  byte;

    /// bitwise access to register CCOR
    struct {
      BITS   CCOEN               : 1;      // bit 0
      BITS   CCOSEL              : 4;      // bits 1-4
      BITS   CCORDY              : 1;      // bit 5
      BITS   CC0BSY              : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // CCOR bitfield

    /// register _CLK_CCOR reset value
    #define sfr_CLK_CCOR_RESET_VALUE   ((uint8_t) 0x00)

  } CCOR;


  /** Peripheral clock gating register 2 (PCKENR2 at 0x50ca) */
  union {

    /// bytewise access to PCKENR2
    uint8_t  byte;

    /// bitwise access to register PCKENR2
    struct {
      BITS   PCKEN2              : 8;      // bits 0-7
    };  // PCKENR2 bitfield

    /// register _CLK_PCKENR2 reset value
    #define sfr_CLK_PCKENR2_RESET_VALUE   ((uint8_t) 0xFF)

  } PCKENR2;


  /** CAN clock control register (CANCCR at 0x50cb) */
  union {

    /// bytewise access to CANCCR
    uint8_t  byte;

    /// bitwise access to register CANCCR
    struct {
      BITS   CANDIV              : 3;      // bits 0-2
      BITS                       : 5;      // 5 bits
    };  // CANCCR bitfield

    /// register _CLK_CANCCR reset value
    #define sfr_CLK_CANCCR_RESET_VALUE   ((uint8_t) 0x00)

  } CANCCR;


  /** HSI clock calibration trimming register (HSITRIMR at 0x50cc) */
  union {

    /// bytewise access to HSITRIMR
    uint8_t  byte;

    /// bitwise access to register HSITRIMR
    struct {
      BITS   HSITRIM             : 4;      // bits 0-3
      BITS                       : 4;      // 4 bits
    };  // HSITRIMR bitfield

    /// register _CLK_HSITRIMR reset value
    #define sfr_CLK_HSITRIMR_RESET_VALUE   ((uint8_t) 0x00)

  } HSITRIMR;


  /** SWIM clock control register (SWIMCCR at 0x50cd) */
  union {

    /// bytewise access to SWIMCCR
    uint8_t  byte;

    /// bitwise access to register SWIMCCR
    struct {
      BITS   SWIMCLK             : 1;      // bit 0
      BITS                       : 7;      // 7 bits
    };  // SWIMCCR bitfield

    /// register _CLK_SWIMCCR reset value
    #define sfr_CLK_SWIMCCR_RESET_VALUE   ((uint8_t) 0x00)

  } SWIMCCR;

} CLK_t;

/// access to CLK SFR registers
#define sfr_CLK   (*((CLK_t*) 0x50c0))


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
    #define sfr_CPU_SPH_RESET_VALUE   ((uint8_t) 0x07)

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
      BITS   NF                  : 1;      // bit 2
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
      BITS   SWO                 : 1;      // bit 0
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


  /** DM debug module control register 1 (CR1 at 0x7f96) */
  union {

    /// bytewise access to CR1
    uint8_t  byte;

    /// skip bitwise access to register CR1

    /// register _DM_CR1 reset value
    #define sfr_DM_CR1_RESET_VALUE   ((uint8_t) 0x00)

  } CR1;


  /** DM debug module control register 2 (CR2 at 0x7f97) */
  union {

    /// bytewise access to CR2
    uint8_t  byte;

    /// skip bitwise access to register CR2

    /// register _DM_CR2 reset value
    #define sfr_DM_CR2_RESET_VALUE   ((uint8_t) 0x00)

  } CR2;


  /** DM debug module control/status register 1 (CSR1 at 0x7f98) */
  union {

    /// bytewise access to CSR1
    uint8_t  byte;

    /// skip bitwise access to register CSR1

    /// register _DM_CSR1 reset value
    #define sfr_DM_CSR1_RESET_VALUE   ((uint8_t) 0x10)

  } CSR1;


  /** DM debug module control/status register 2 (CSR2 at 0x7f99) */
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
// Module FLASH
//------------------------

/** struct containing FLASH module registers */
typedef struct {

  /** Flash control register 1 (CR1 at 0x505a) */
  union {

    /// bytewise access to CR1
    uint8_t  byte;

    /// bitwise access to register CR1
    struct {
      BITS   FIX                 : 1;      // bit 0
      BITS   IE                  : 1;      // bit 1
      BITS   AHALT               : 1;      // bit 2
      BITS   HALT                : 1;      // bit 3
      BITS                       : 4;      // 4 bits
    };  // CR1 bitfield

    /// register _FLASH_CR1 reset value
    #define sfr_FLASH_CR1_RESET_VALUE   ((uint8_t) 0x00)

  } CR1;


  /** Flash control register 2 (CR2 at 0x505b) */
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


  /** Flash complementary control register 2 (NCR2 at 0x505c) */
  union {

    /// bytewise access to NCR2
    uint8_t  byte;

    /// bitwise access to register NCR2
    struct {
      BITS   NPRG                : 1;      // bit 0
      BITS                       : 3;      // 3 bits
      BITS   NFPRG               : 1;      // bit 4
      BITS   NERASE              : 1;      // bit 5
      BITS   NWPRG               : 1;      // bit 6
      BITS   NOPT                : 1;      // bit 7
    };  // NCR2 bitfield

    /// register _FLASH_NCR2 reset value
    #define sfr_FLASH_NCR2_RESET_VALUE   ((uint8_t) 0xFF)

  } NCR2;


  /** Flash protection register (FPR at 0x505d) */
  union {

    /// bytewise access to FPR
    uint8_t  byte;

    /// bitwise access to register FPR
    struct {
      BITS   WPB0                : 1;      // bit 0
      BITS   WPB1                : 1;      // bit 1
      BITS   WPB2                : 1;      // bit 2
      BITS   WPB3                : 1;      // bit 3
      BITS   WPB4                : 1;      // bit 4
      BITS   WPB5                : 1;      // bit 5
      BITS                       : 2;      // 2 bits
    };  // FPR bitfield

    /// register _FLASH_FPR reset value
    #define sfr_FLASH_FPR_RESET_VALUE   ((uint8_t) 0x00)

  } FPR;


  /** Flash complementary protection register (NFPR at 0x505e) */
  union {

    /// bytewise access to NFPR
    uint8_t  byte;

    /// bitwise access to register NFPR
    struct {
      BITS   NWPB0               : 1;      // bit 0
      BITS   NWPB1               : 1;      // bit 1
      BITS   NWPB2               : 1;      // bit 2
      BITS   NWPB3               : 1;      // bit 3
      BITS   NWPB4               : 1;      // bit 4
      BITS   NWPB5               : 1;      // bit 5
      BITS                       : 2;      // 2 bits
    };  // NFPR bitfield

    /// register _FLASH_NFPR reset value
    #define sfr_FLASH_NFPR_RESET_VALUE   ((uint8_t) 0xFF)

  } NFPR;


  /** Flash in-application programming status register (IAPSR at 0x505f) */
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


  /// Reserved register (2B)
  uint8_t     Reserved_1[2];


  /** Flash program memory unprotection register (PUKR at 0x5062) */
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


  /// Reserved register (1B)
  uint8_t     Reserved_2[1];


  /** Data EEPROM unprotection register (DUKR at 0x5064) */
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

} FLASH_t;

/// access to FLASH SFR registers
#define sfr_FLASH   (*((FLASH_t*) 0x505a))


//------------------------
// Module I2C
//------------------------

/** struct containing I2C module registers */
typedef struct {

  /** I2C control register 1 (CR1 at 0x5210) */
  union {

    /// bytewise access to CR1
    uint8_t  byte;

    /// bitwise access to register CR1
    struct {
      BITS   PE                  : 1;      // bit 0
      BITS                       : 5;      // 5 bits
      BITS   ENGC                : 1;      // bit 6
      BITS   NOSTRETCH           : 1;      // bit 7
    };  // CR1 bitfield

    /// register _I2C_CR1 reset value
    #define sfr_I2C_CR1_RESET_VALUE   ((uint8_t) 0x00)

  } CR1;


  /** I2C control register 2 (CR2 at 0x5211) */
  union {

    /// bytewise access to CR2
    uint8_t  byte;

    /// bitwise access to register CR2
    struct {
      BITS   START               : 1;      // bit 0
      BITS   STOP                : 1;      // bit 1
      BITS   ACK                 : 1;      // bit 2
      BITS   POS                 : 1;      // bit 3
      BITS                       : 3;      // 3 bits
      BITS   SWRST               : 1;      // bit 7
    };  // CR2 bitfield

    /// register _I2C_CR2 reset value
    #define sfr_I2C_CR2_RESET_VALUE   ((uint8_t) 0x00)

  } CR2;


  /** I2C frequency register (FREQR at 0x5212) */
  union {

    /// bytewise access to FREQR
    uint8_t  byte;

    /// bitwise access to register FREQR
    struct {
      BITS   FREQ                : 6;      // bits 0-5
      BITS                       : 2;      // 2 bits
    };  // FREQR bitfield

    /// register _I2C_FREQR reset value
    #define sfr_I2C_FREQR_RESET_VALUE   ((uint8_t) 0x00)

  } FREQR;


  /** I2C Own address register low (OARL at 0x5213) */
  union {

    /// bytewise access to OARL
    uint8_t  byte;

    /// bitwise access to register OARL
    struct {
      BITS   ADD0                : 1;      // bit 0
      BITS   ADD                 : 7;      // bits 1-7
    };  // OARL bitfield

    /// register _I2C_OARL reset value
    #define sfr_I2C_OARL_RESET_VALUE   ((uint8_t) 0x00)

  } OARL;


  /** I2C own address register high (OARH at 0x5214) */
  union {

    /// bytewise access to OARH
    uint8_t  byte;

    /// bitwise access to register OARH
    struct {
      BITS                       : 1;      // 1 bit
      BITS   ADD                 : 2;      // bits 1-2
      BITS                       : 3;      // 3 bits
      BITS   ADDCONF             : 1;      // bit 6
      BITS   ADDMODE             : 1;      // bit 7
    };  // OARH bitfield

    /// register _I2C_OARH reset value
    #define sfr_I2C_OARH_RESET_VALUE   ((uint8_t) 0x00)

  } OARH;


  /// Reserved register (1B)
  uint8_t     Reserved_1[1];


  /** I2C data register (DR at 0x5216) */
  union {

    /// bytewise access to DR
    uint8_t  byte;

    /// bitwise access to register DR
    struct {
      BITS   DR                  : 8;      // bits 0-7
    };  // DR bitfield

    /// register _I2C_DR reset value
    #define sfr_I2C_DR_RESET_VALUE   ((uint8_t) 0x00)

  } DR;


  /** I2C status register 1 (SR1 at 0x5217) */
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

    /// register _I2C_SR1 reset value
    #define sfr_I2C_SR1_RESET_VALUE   ((uint8_t) 0x00)

  } SR1;


  /** I2C status register 2 (SR2 at 0x5218) */
  union {

    /// bytewise access to SR2
    uint8_t  byte;

    /// bitwise access to register SR2
    struct {
      BITS   BERR                : 1;      // bit 0
      BITS   ARLO                : 1;      // bit 1
      BITS   AF                  : 1;      // bit 2
      BITS   OVR                 : 1;      // bit 3
      BITS                       : 1;      // 1 bit
      BITS   WUFH                : 1;      // bit 5
      BITS                       : 2;      // 2 bits
    };  // SR2 bitfield

    /// register _I2C_SR2 reset value
    #define sfr_I2C_SR2_RESET_VALUE   ((uint8_t) 0x00)

  } SR2;


  /** I2C status register 3 (SR3 at 0x5219) */
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
      BITS                       : 3;      // 3 bits
    };  // SR3 bitfield

    /// register _I2C_SR3 reset value
    #define sfr_I2C_SR3_RESET_VALUE   ((uint8_t) 0x00)

  } SR3;


  /** I2C interrupt control register (ITR at 0x521a) */
  union {

    /// bytewise access to ITR
    uint8_t  byte;

    /// bitwise access to register ITR
    struct {
      BITS   ITERREN             : 1;      // bit 0
      BITS   ITEVTEN             : 1;      // bit 1
      BITS   ITBUFEN             : 1;      // bit 2
      BITS                       : 5;      // 5 bits
    };  // ITR bitfield

    /// register _I2C_ITR reset value
    #define sfr_I2C_ITR_RESET_VALUE   ((uint8_t) 0x00)

  } ITR;


  /** I2C clock control register low (CCRL at 0x521b) */
  union {

    /// bytewise access to CCRL
    uint8_t  byte;

    /// bitwise access to register CCRL
    struct {
      BITS   CCR                 : 8;      // bits 0-7
    };  // CCRL bitfield

    /// register _I2C_CCRL reset value
    #define sfr_I2C_CCRL_RESET_VALUE   ((uint8_t) 0x00)

  } CCRL;


  /** I2C clock control register high (CCRH at 0x521c) */
  union {

    /// bytewise access to CCRH
    uint8_t  byte;

    /// bitwise access to register CCRH
    struct {
      BITS   CCR                 : 4;      // bits 0-3
      BITS                       : 2;      // 2 bits
      BITS   DUTY                : 1;      // bit 6
      BITS   F_S                 : 1;      // bit 7
    };  // CCRH bitfield

    /// register _I2C_CCRH reset value
    #define sfr_I2C_CCRH_RESET_VALUE   ((uint8_t) 0x00)

  } CCRH;


  /** I2C TRISE register (TRISER at 0x521d) */
  union {

    /// bytewise access to TRISER
    uint8_t  byte;

    /// bitwise access to register TRISER
    struct {
      BITS   TRISE               : 6;      // bits 0-5
      BITS                       : 2;      // 2 bits
    };  // TRISER bitfield

    /// register _I2C_TRISER reset value
    #define sfr_I2C_TRISER_RESET_VALUE   ((uint8_t) 0x02)

  } TRISER;


  /** I2C packet error checking register (PECR at 0x521e) */
  union {

    /// bytewise access to PECR
    uint8_t  byte;

    /// bitwise access to register PECR
    struct {
      BITS   PEC                 : 8;      // bits 0-7
    };  // PECR bitfield

    /// register _I2C_PECR reset value
    #define sfr_I2C_PECR_RESET_VALUE   ((uint8_t) 0x00)

  } PECR;

} I2C_t;

/// access to I2C SFR registers
#define sfr_I2C   (*((I2C_t*) 0x5210))


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
      BITS   PAIS                : 2;      // bits 0-1
      BITS   PBIS                : 2;      // bits 2-3
      BITS   PCIS                : 2;      // bits 4-5
      BITS   PDIS                : 2;      // bits 6-7
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
      BITS   PEIS                : 2;      // bits 0-1
      BITS   TLIS                : 1;      // bit 2
      BITS                       : 5;      // 5 bits
    };  // CR2 bitfield

    /// register _ITC_EXTI_CR2 reset value
    #define sfr_ITC_EXTI_CR2_RESET_VALUE   ((uint8_t) 0x00)

  } CR2;

} ITC_EXTI_t;

/// access to ITC_EXTI SFR registers
#define sfr_ITC_EXTI   (*((ITC_EXTI_t*) 0x50a0))


//------------------------
// Module ITC_SPR
//------------------------

/** struct containing ITC_SPR module registers */
typedef struct {

  /** Interrupt software priority register 1 (SPR1 at 0x7f70) */
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


  /** Interrupt software priority register 2 (SPR2 at 0x7f71) */
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


  /** Interrupt software priority register 3 (SPR3 at 0x7f72) */
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


  /** Interrupt software priority register 4 (SPR4 at 0x7f73) */
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


  /** Interrupt software priority register 5 (SPR5 at 0x7f74) */
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


  /** Interrupt software priority register 6 (SPR6 at 0x7f75) */
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


  /** Interrupt software priority register 7 (SPR7 at 0x7f76) */
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


  /** Interrupt software priority register 8 (SPR8 at 0x7f77) */
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
      BITS   KEY                 : 8;      // bits 0-7
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
    #define sfr_OPT_OPT0_RESET_VALUE   ((uint8_t) 0x00)

  } OPT0;


  /** User boot code (UBC) (OPT1 at 0x4801) */
  union {

    /// bytewise access to OPT1
    uint8_t  byte;

    /// skip bitwise access to register OPT1

    /// register _OPT_OPT1 reset value
    #define sfr_OPT_OPT1_RESET_VALUE   ((uint8_t) 0x00)

  } OPT1;


  /** User boot code (UBC) (complementary byte) (NOPT1 at 0x4802) */
  union {

    /// bytewise access to NOPT1
    uint8_t  byte;

    /// skip bitwise access to register NOPT1

    /// register _OPT_NOPT1 reset value
    #define sfr_OPT_NOPT1_RESET_VALUE   ((uint8_t) 0xFF)

  } NOPT1;


  /** Alternate function remapping (AFR) (OPT2 at 0x4803) */
  union {

    /// bytewise access to OPT2
    uint8_t  byte;

    /// skip bitwise access to register OPT2

    /// register _OPT_OPT2 reset value
    #define sfr_OPT_OPT2_RESET_VALUE   ((uint8_t) 0x00)

  } OPT2;


  /** Alternate function remapping (AFR) (complementary byte) (NOPT2 at 0x4804) */
  union {

    /// bytewise access to NOPT2
    uint8_t  byte;

    /// skip bitwise access to register NOPT2

    /// register _OPT_NOPT2 reset value
    #define sfr_OPT_NOPT2_RESET_VALUE   ((uint8_t) 0xFF)

  } NOPT2;


  /** Misc. option (OPT3 at 0x4805) */
  union {

    /// bytewise access to OPT3
    uint8_t  byte;

    /// skip bitwise access to register OPT3

    /// register _OPT_OPT3 reset value
    #define sfr_OPT_OPT3_RESET_VALUE   ((uint8_t) 0x00)

  } OPT3;


  /** Misc. option (complementary byte) (NOPT3 at 0x4806) */
  union {

    /// bytewise access to NOPT3
    uint8_t  byte;

    /// skip bitwise access to register NOPT3

    /// register _OPT_NOPT3 reset value
    #define sfr_OPT_NOPT3_RESET_VALUE   ((uint8_t) 0xFF)

  } NOPT3;


  /** Clock option (OPT4 at 0x4807) */
  union {

    /// bytewise access to OPT4
    uint8_t  byte;

    /// skip bitwise access to register OPT4

    /// register _OPT_OPT4 reset value
    #define sfr_OPT_OPT4_RESET_VALUE   ((uint8_t) 0x00)

  } OPT4;


  /** Clock option (complementary byte) (NOPT4 at 0x4808) */
  union {

    /// bytewise access to NOPT4
    uint8_t  byte;

    /// skip bitwise access to register NOPT4

    /// register _OPT_NOPT4 reset value
    #define sfr_OPT_NOPT4_RESET_VALUE   ((uint8_t) 0xFF)

  } NOPT4;


  /** HSE clock startup (OPT5 at 0x4809) */
  union {

    /// bytewise access to OPT5
    uint8_t  byte;

    /// skip bitwise access to register OPT5

    /// register _OPT_OPT5 reset value
    #define sfr_OPT_OPT5_RESET_VALUE   ((uint8_t) 0x00)

  } OPT5;


  /** HSE clock startup (complementary byte) (NOPT5 at 0x480a) */
  union {

    /// bytewise access to NOPT5
    uint8_t  byte;

    /// skip bitwise access to register NOPT5

    /// register _OPT_NOPT5 reset value
    #define sfr_OPT_NOPT5_RESET_VALUE   ((uint8_t) 0xFF)

  } NOPT5;


  /// Reserved register (115B)
  uint8_t     Reserved_1[115];


  /** Bootloader (OPTBL at 0x487e) */
  union {

    /// bytewise access to OPTBL
    uint8_t  byte;

    /// skip bitwise access to register OPTBL

    /// register _OPT_OPTBL reset value
    #define sfr_OPT_OPTBL_RESET_VALUE   ((uint8_t) 0x00)

  } OPTBL;


  /** Bootloader (complementary byte) (NOPTBL at 0x487f) */
  union {

    /// bytewise access to NOPTBL
    uint8_t  byte;

    /// skip bitwise access to register NOPTBL

    /// register _OPT_NOPTBL reset value
    #define sfr_OPT_NOPTBL_RESET_VALUE   ((uint8_t) 0xFF)

  } NOPTBL;

} OPT_t;

/// access to OPT SFR registers
#define sfr_OPT   (*((OPT_t*) 0x4800))


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
    #define sfr_PORT_CR1_RESET_VALUE   ((uint8_t) 0x00)

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


/// access to PORTG SFR registers
#define sfr_PORTG   (*((PORT_t*) 0x501e))


/// access to PORTH SFR registers
#define sfr_PORTH   (*((PORT_t*) 0x5023))


/// access to PORTI SFR registers
#define sfr_PORTI   (*((PORT_t*) 0x5028))


//------------------------
// Module RST
//------------------------

/** struct containing RST module registers */
typedef struct {

  /** Reset status register (SR at 0x50b3) */
  union {

    /// bytewise access to SR
    uint8_t  byte;

    /// bitwise access to register SR
    struct {
      BITS   WWDGF               : 1;      // bit 0
      BITS   IWDGF               : 1;      // bit 1
      BITS   ILLOPF              : 1;      // bit 2
      BITS   SWIMF               : 1;      // bit 3
      BITS   EMCF                : 1;      // bit 4
      BITS                       : 3;      // 3 bits
    };  // SR bitfield

    /// register _RST_SR reset value
    #define sfr_RST_SR_RESET_VALUE   ((uint8_t) 0x00)

  } SR;

} RST_t;

/// access to RST SFR registers
#define sfr_RST   (*((RST_t*) 0x50b3))


//------------------------
// Module SPI
//------------------------

/** struct containing SPI module registers */
typedef struct {

  /** SPI control register 1 (CR1 at 0x5200) */
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

    /// register _SPI_CR1 reset value
    #define sfr_SPI_CR1_RESET_VALUE   ((uint8_t) 0x00)

  } CR1;


  /** SPI control register 2 (CR2 at 0x5201) */
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
      BITS   CECEN               : 1;      // bit 5
      BITS   BDOE                : 1;      // bit 6
      BITS   BDM                 : 1;      // bit 7
    };  // CR2 bitfield

    /// register _SPI_CR2 reset value
    #define sfr_SPI_CR2_RESET_VALUE   ((uint8_t) 0x00)

  } CR2;


  /** SPI interrupt control register (ICR at 0x5202) */
  union {

    /// bytewise access to ICR
    uint8_t  byte;

    /// bitwise access to register ICR
    struct {
      BITS                       : 4;      // 4 bits
      BITS   WKIE                : 1;      // bit 4
      BITS   ERRIE               : 1;      // bit 5
      BITS   RXIE                : 1;      // bit 6
      BITS   TXIE                : 1;      // bit 7
    };  // ICR bitfield

    /// register _SPI_ICR reset value
    #define sfr_SPI_ICR_RESET_VALUE   ((uint8_t) 0x00)

  } ICR;


  /** SPI status register (SR at 0x5203) */
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

    /// register _SPI_SR reset value
    #define sfr_SPI_SR_RESET_VALUE   ((uint8_t) 0x02)

  } SR;


  /** SPI data register (DR at 0x5204) */
  union {

    /// bytewise access to DR
    uint8_t  byte;

    /// bitwise access to register DR
    struct {
      BITS   DR                  : 8;      // bits 0-7
    };  // DR bitfield

    /// register _SPI_DR reset value
    #define sfr_SPI_DR_RESET_VALUE   ((uint8_t) 0x00)

  } DR;


  /** SPI CRC polynomial register (CRCPR at 0x5205) */
  union {

    /// bytewise access to CRCPR
    uint8_t  byte;

    /// bitwise access to register CRCPR
    struct {
      BITS   CRCPOLY             : 8;      // bits 0-7
    };  // CRCPR bitfield

    /// register _SPI_CRCPR reset value
    #define sfr_SPI_CRCPR_RESET_VALUE   ((uint8_t) 0x07)

  } CRCPR;


  /** SPI Rx CRC register (RXCRCR at 0x5206) */
  union {

    /// bytewise access to RXCRCR
    uint8_t  byte;

    /// bitwise access to register RXCRCR
    struct {
      BITS   RXCRC               : 8;      // bits 0-7
    };  // RXCRCR bitfield

    /// register _SPI_RXCRCR reset value
    #define sfr_SPI_RXCRCR_RESET_VALUE   ((uint8_t) 0xFF)

  } RXCRCR;


  /** SPI Tx CRC register (TXCRCR at 0x5207) */
  union {

    /// bytewise access to TXCRCR
    uint8_t  byte;

    /// bitwise access to register TXCRCR
    struct {
      BITS   TXCRC               : 8;      // bits 0-7
    };  // TXCRCR bitfield

    /// register _SPI_TXCRCR reset value
    #define sfr_SPI_TXCRCR_RESET_VALUE   ((uint8_t) 0xFF)

  } TXCRCR;

} SPI_t;

/// access to SPI SFR registers
#define sfr_SPI   (*((SPI_t*) 0x5200))


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

  /** TIM1 control register 1 (CR1 at 0x5250) */
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


  /** TIM1 control register 2 (CR2 at 0x5251) */
  union {

    /// bytewise access to CR2
    uint8_t  byte;

    /// bitwise access to register CR2
    struct {
      BITS   CCPG                : 1;      // bit 0
      BITS                       : 1;      // 1 bit
      BITS   COMS                : 1;      // bit 2
      BITS                       : 1;      // 1 bit
      BITS   MMS                 : 3;      // bits 4-6
      BITS                       : 1;      // 1 bit
    };  // CR2 bitfield

    /// register _TIM1_CR2 reset value
    #define sfr_TIM1_CR2_RESET_VALUE   ((uint8_t) 0x00)

  } CR2;


  /** TIM1 slave mode control register (SMCR at 0x5252) */
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

    /// register _TIM1_SMCR reset value
    #define sfr_TIM1_SMCR_RESET_VALUE   ((uint8_t) 0x00)

  } SMCR;


  /** TIM1 external trigger register (ETR at 0x5253) */
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


  /** TIM1 interrupt enable register (IER at 0x5254) */
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


  /** TIM1 status register 1 (SR1 at 0x5255) */
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


  /** TIM1 status register 2 (SR2 at 0x5256) */
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


  /** TIM1 event generation register (EGR at 0x5257) */
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


  /** TIM1 capture/compare mode register 1 (CCMR1 at 0x5258) */
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


  /** TIM1 capture/compare mode register 2 (CCMR2 at 0x5259) */
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


  /** TIM1 capture/compare mode register 3 (CCMR3 at 0x525a) */
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


  /** TIM1 capture/compare mode register 4 (CCMR4 at 0x525b) */
  union {

    /// bytewise access to CCMR4
    uint8_t  byte;

    /// bitwise access to register CCMR4
    struct {
      BITS   CC4S                : 2;      // bits 0-1
      BITS   OC4FE               : 1;      // bit 2
      BITS   OC4PE               : 1;      // bit 3
      BITS   OC4M                : 3;      // bits 4-6
      BITS   OC4CE               : 1;      // bit 7
    };  // CCMR4 bitfield

    /// register _TIM1_CCMR4 reset value
    #define sfr_TIM1_CCMR4_RESET_VALUE   ((uint8_t) 0x00)

  } CCMR4;


  /** TIM1 capture/compare enable register 1 (CCER1 at 0x525c) */
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


  /** TIM1 capture/compare enable register 2 (CCER2 at 0x525d) */
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


  /** TIM1 counter high (CNTRH at 0x525e) */
  union {

    /// bytewise access to CNTRH
    uint8_t  byte;

    /// bitwise access to register CNTRH
    struct {
      BITS   CNT                 : 8;      // bits 0-7
    };  // CNTRH bitfield

    /// register _TIM1_CNTRH reset value
    #define sfr_TIM1_CNTRH_RESET_VALUE   ((uint8_t) 0x00)

  } CNTRH;


  /** TIM1 counter low (CNTRL at 0x525f) */
  union {

    /// bytewise access to CNTRL
    uint8_t  byte;

    /// bitwise access to register CNTRL
    struct {
      BITS   CNT                 : 8;      // bits 0-7
    };  // CNTRL bitfield

    /// register _TIM1_CNTRL reset value
    #define sfr_TIM1_CNTRL_RESET_VALUE   ((uint8_t) 0x00)

  } CNTRL;


  /** TIM1 prescaler register high (PSCRH at 0x5260) */
  union {

    /// bytewise access to PSCRH
    uint8_t  byte;

    /// bitwise access to register PSCRH
    struct {
      BITS   PSC                 : 8;      // bits 0-7
    };  // PSCRH bitfield

    /// register _TIM1_PSCRH reset value
    #define sfr_TIM1_PSCRH_RESET_VALUE   ((uint8_t) 0x00)

  } PSCRH;


  /** TIM1 prescaler register low (PSCRL at 0x5261) */
  union {

    /// bytewise access to PSCRL
    uint8_t  byte;

    /// bitwise access to register PSCRL
    struct {
      BITS   PSC                 : 8;      // bits 0-7
    };  // PSCRL bitfield

    /// register _TIM1_PSCRL reset value
    #define sfr_TIM1_PSCRL_RESET_VALUE   ((uint8_t) 0x00)

  } PSCRL;


  /** TIM1 auto-reload register high (ARRH at 0x5262) */
  union {

    /// bytewise access to ARRH
    uint8_t  byte;

    /// bitwise access to register ARRH
    struct {
      BITS   ARR                 : 8;      // bits 0-7
    };  // ARRH bitfield

    /// register _TIM1_ARRH reset value
    #define sfr_TIM1_ARRH_RESET_VALUE   ((uint8_t) 0xFF)

  } ARRH;


  /** TIM1 auto-reload register low (ARRL at 0x5263) */
  union {

    /// bytewise access to ARRL
    uint8_t  byte;

    /// bitwise access to register ARRL
    struct {
      BITS   ARR                 : 8;      // bits 0-7
    };  // ARRL bitfield

    /// register _TIM1_ARRL reset value
    #define sfr_TIM1_ARRL_RESET_VALUE   ((uint8_t) 0xFF)

  } ARRL;


  /** TIM1 repetition counter register (RCR at 0x5264) */
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


  /** TIM1 capture/compare register 1 high (CCR1H at 0x5265) */
  union {

    /// bytewise access to CCR1H
    uint8_t  byte;

    /// bitwise access to register CCR1H
    struct {
      BITS   CCR1                : 8;      // bits 0-7
    };  // CCR1H bitfield

    /// register _TIM1_CCR1H reset value
    #define sfr_TIM1_CCR1H_RESET_VALUE   ((uint8_t) 0x00)

  } CCR1H;


  /** TIM1 capture/compare register 1 low (CCR1L at 0x5266) */
  union {

    /// bytewise access to CCR1L
    uint8_t  byte;

    /// bitwise access to register CCR1L
    struct {
      BITS   CCR1                : 8;      // bits 0-7
    };  // CCR1L bitfield

    /// register _TIM1_CCR1L reset value
    #define sfr_TIM1_CCR1L_RESET_VALUE   ((uint8_t) 0x00)

  } CCR1L;


  /** TIM1 capture/compare register 2 high (CCR2H at 0x5267) */
  union {

    /// bytewise access to CCR2H
    uint8_t  byte;

    /// bitwise access to register CCR2H
    struct {
      BITS   CCR2                : 8;      // bits 0-7
    };  // CCR2H bitfield

    /// register _TIM1_CCR2H reset value
    #define sfr_TIM1_CCR2H_RESET_VALUE   ((uint8_t) 0x00)

  } CCR2H;


  /** TIM1 capture/compare register 2 low (CCR2L at 0x5268) */
  union {

    /// bytewise access to CCR2L
    uint8_t  byte;

    /// bitwise access to register CCR2L
    struct {
      BITS   CCR2                : 8;      // bits 0-7
    };  // CCR2L bitfield

    /// register _TIM1_CCR2L reset value
    #define sfr_TIM1_CCR2L_RESET_VALUE   ((uint8_t) 0x00)

  } CCR2L;


  /** TIM1 capture/compare register 3 high (CCR3H at 0x5269) */
  union {

    /// bytewise access to CCR3H
    uint8_t  byte;

    /// bitwise access to register CCR3H
    struct {
      BITS   CCR3                : 8;      // bits 0-7
    };  // CCR3H bitfield

    /// register _TIM1_CCR3H reset value
    #define sfr_TIM1_CCR3H_RESET_VALUE   ((uint8_t) 0x00)

  } CCR3H;


  /** TIM1 capture/compare register 3 low (CCR3L at 0x526a) */
  union {

    /// bytewise access to CCR3L
    uint8_t  byte;

    /// bitwise access to register CCR3L
    struct {
      BITS   CCR3                : 8;      // bits 0-7
    };  // CCR3L bitfield

    /// register _TIM1_CCR3L reset value
    #define sfr_TIM1_CCR3L_RESET_VALUE   ((uint8_t) 0x00)

  } CCR3L;


  /** TIM1 capture/compare register 4 high (CCR4H at 0x526b) */
  union {

    /// bytewise access to CCR4H
    uint8_t  byte;

    /// bitwise access to register CCR4H
    struct {
      BITS   CCR4                : 8;      // bits 0-7
    };  // CCR4H bitfield

    /// register _TIM1_CCR4H reset value
    #define sfr_TIM1_CCR4H_RESET_VALUE   ((uint8_t) 0x00)

  } CCR4H;


  /** TIM1 capture/compare register 4 low (CCR4L at 0x526c) */
  union {

    /// bytewise access to CCR4L
    uint8_t  byte;

    /// bitwise access to register CCR4L
    struct {
      BITS   CCR4                : 8;      // bits 0-7
    };  // CCR4L bitfield

    /// register _TIM1_CCR4L reset value
    #define sfr_TIM1_CCR4L_RESET_VALUE   ((uint8_t) 0x00)

  } CCR4L;


  /** TIM1 break register (BKR at 0x526d) */
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


  /** TIM1 dead-time register (DTR at 0x526e) */
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


  /** TIM1 output idle state register (OISR at 0x526f) */
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
      BITS   OIS4                : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // OISR bitfield

    /// register _TIM1_OISR reset value
    #define sfr_TIM1_OISR_RESET_VALUE   ((uint8_t) 0x00)

  } OISR;

} TIM1_t;

/// access to TIM1 SFR registers
#define sfr_TIM1   (*((TIM1_t*) 0x5250))


//------------------------
// Module TIM2
//------------------------

/** struct containing TIM2 module registers */
typedef struct {

  /** TIM2 control register 1 (CR1 at 0x5300) */
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

    /// register _TIM2_CR1 reset value
    #define sfr_TIM2_CR1_RESET_VALUE   ((uint8_t) 0x00)

  } CR1;


  /** TIM2 interrupt enable register (IER at 0x5301) */
  union {

    /// bytewise access to IER
    uint8_t  byte;

    /// bitwise access to register IER
    struct {
      BITS   UIE                 : 1;      // bit 0
      BITS   CC1IE               : 1;      // bit 1
      BITS   CC2IE               : 1;      // bit 2
      BITS   CC3IE               : 1;      // bit 3
      BITS                       : 2;      // 2 bits
      BITS   TIE                 : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // IER bitfield

    /// register _TIM2_IER reset value
    #define sfr_TIM2_IER_RESET_VALUE   ((uint8_t) 0x00)

  } IER;


  /** TIM2 status register 1 (SR1 at 0x5302) */
  union {

    /// bytewise access to SR1
    uint8_t  byte;

    /// bitwise access to register SR1
    struct {
      BITS   UIF                 : 1;      // bit 0
      BITS   CC1IF               : 1;      // bit 1
      BITS   CC2IF               : 1;      // bit 2
      BITS   CC3IF               : 1;      // bit 3
      BITS                       : 2;      // 2 bits
      BITS   TIF                 : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // SR1 bitfield

    /// register _TIM2_SR1 reset value
    #define sfr_TIM2_SR1_RESET_VALUE   ((uint8_t) 0x00)

  } SR1;


  /** TIM2 status register 2 (SR2 at 0x5303) */
  union {

    /// bytewise access to SR2
    uint8_t  byte;

    /// bitwise access to register SR2
    struct {
      BITS                       : 1;      // 1 bit
      BITS   CC1OF               : 1;      // bit 1
      BITS   CC2OF               : 1;      // bit 2
      BITS   CC3OF               : 1;      // bit 3
      BITS                       : 4;      // 4 bits
    };  // SR2 bitfield

    /// register _TIM2_SR2 reset value
    #define sfr_TIM2_SR2_RESET_VALUE   ((uint8_t) 0x00)

  } SR2;


  /** TIM2 event generation register (EGR at 0x5304) */
  union {

    /// bytewise access to EGR
    uint8_t  byte;

    /// bitwise access to register EGR
    struct {
      BITS   UG                  : 1;      // bit 0
      BITS   CC1G                : 1;      // bit 1
      BITS   CC2G                : 1;      // bit 2
      BITS   CC3G                : 1;      // bit 3
      BITS                       : 2;      // 2 bits
      BITS   TG                  : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // EGR bitfield

    /// register _TIM2_EGR reset value
    #define sfr_TIM2_EGR_RESET_VALUE   ((uint8_t) 0x00)

  } EGR;


  /** TIM2 capture/compare mode register 1 (CCMR1 at 0x5305) */
  union {

    /// bytewise access to CCMR1
    uint8_t  byte;

    /// bitwise access to register CCMR1
    struct {
      BITS   CC1S                : 2;      // bits 0-1
      BITS                       : 1;      // 1 bit
      BITS   OC1PE               : 1;      // bit 3
      BITS   OC1M                : 3;      // bits 4-6
      BITS                       : 1;      // 1 bit
    };  // CCMR1 bitfield

    /// register _TIM2_CCMR1 reset value
    #define sfr_TIM2_CCMR1_RESET_VALUE   ((uint8_t) 0x00)

  } CCMR1;


  /** TIM2 capture/compare mode register 2 (CCMR2 at 0x5306) */
  union {

    /// bytewise access to CCMR2
    uint8_t  byte;

    /// bitwise access to register CCMR2
    struct {
      BITS   CC2S                : 2;      // bits 0-1
      BITS                       : 1;      // 1 bit
      BITS   OC2PE               : 1;      // bit 3
      BITS   OC2M                : 3;      // bits 4-6
      BITS                       : 1;      // 1 bit
    };  // CCMR2 bitfield

    /// register _TIM2_CCMR2 reset value
    #define sfr_TIM2_CCMR2_RESET_VALUE   ((uint8_t) 0x00)

  } CCMR2;


  /** TIM2 capture/compare mode register 3 (CCMR3 at 0x5307) */
  union {

    /// bytewise access to CCMR3
    uint8_t  byte;

    /// bitwise access to register CCMR3
    struct {
      BITS   CC3S                : 2;      // bits 0-1
      BITS                       : 1;      // 1 bit
      BITS   OC3PE               : 1;      // bit 3
      BITS   OC3M                : 3;      // bits 4-6
      BITS                       : 1;      // 1 bit
    };  // CCMR3 bitfield

    /// register _TIM2_CCMR3 reset value
    #define sfr_TIM2_CCMR3_RESET_VALUE   ((uint8_t) 0x00)

  } CCMR3;


  /** TIM2 capture/compare enable register 1 (CCER1 at 0x5308) */
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


  /** TIM2 capture/compare enable register 2 (CCER2 at 0x5309) */
  union {

    /// bytewise access to CCER2
    uint8_t  byte;

    /// bitwise access to register CCER2
    struct {
      BITS   CC3E                : 1;      // bit 0
      BITS   CC3P                : 1;      // bit 1
      BITS                       : 6;      // 6 bits
    };  // CCER2 bitfield

    /// register _TIM2_CCER2 reset value
    #define sfr_TIM2_CCER2_RESET_VALUE   ((uint8_t) 0x00)

  } CCER2;


  /** TIM2 counter high (CNTRH at 0x530a) */
  union {

    /// bytewise access to CNTRH
    uint8_t  byte;

    /// bitwise access to register CNTRH
    struct {
      BITS   CNT                 : 8;      // bits 0-7
    };  // CNTRH bitfield

    /// register _TIM2_CNTRH reset value
    #define sfr_TIM2_CNTRH_RESET_VALUE   ((uint8_t) 0x00)

  } CNTRH;


  /** TIM2 counter low (CNTRL at 0x530b) */
  union {

    /// bytewise access to CNTRL
    uint8_t  byte;

    /// bitwise access to register CNTRL
    struct {
      BITS   CNT                 : 8;      // bits 0-7
    };  // CNTRL bitfield

    /// register _TIM2_CNTRL reset value
    #define sfr_TIM2_CNTRL_RESET_VALUE   ((uint8_t) 0x00)

  } CNTRL;


  /** TIM2 prescaler register (PSCR at 0x530c) */
  union {

    /// bytewise access to PSCR
    uint8_t  byte;

    /// bitwise access to register PSCR
    struct {
      BITS   PSC                 : 4;      // bits 0-3
      BITS                       : 4;      // 4 bits
    };  // PSCR bitfield

    /// register _TIM2_PSCR reset value
    #define sfr_TIM2_PSCR_RESET_VALUE   ((uint8_t) 0x00)

  } PSCR;


  /** TIM2 auto-reload register high (ARRH at 0x530d) */
  union {

    /// bytewise access to ARRH
    uint8_t  byte;

    /// bitwise access to register ARRH
    struct {
      BITS   ARR                 : 8;      // bits 0-7
    };  // ARRH bitfield

    /// register _TIM2_ARRH reset value
    #define sfr_TIM2_ARRH_RESET_VALUE   ((uint8_t) 0xFF)

  } ARRH;


  /** TIM2 auto-reload register low (ARRL at 0x530e) */
  union {

    /// bytewise access to ARRL
    uint8_t  byte;

    /// bitwise access to register ARRL
    struct {
      BITS   ARR                 : 8;      // bits 0-7
    };  // ARRL bitfield

    /// register _TIM2_ARRL reset value
    #define sfr_TIM2_ARRL_RESET_VALUE   ((uint8_t) 0xFF)

  } ARRL;


  /** TIM2 capture/compare register 1 high (CCR1H at 0x530f) */
  union {

    /// bytewise access to CCR1H
    uint8_t  byte;

    /// bitwise access to register CCR1H
    struct {
      BITS   CCR1                : 8;      // bits 0-7
    };  // CCR1H bitfield

    /// register _TIM2_CCR1H reset value
    #define sfr_TIM2_CCR1H_RESET_VALUE   ((uint8_t) 0x00)

  } CCR1H;


  /** TIM2 capture/compare register 1 low (CCR1L at 0x5310) */
  union {

    /// bytewise access to CCR1L
    uint8_t  byte;

    /// bitwise access to register CCR1L
    struct {
      BITS   CCR1                : 8;      // bits 0-7
    };  // CCR1L bitfield

    /// register _TIM2_CCR1L reset value
    #define sfr_TIM2_CCR1L_RESET_VALUE   ((uint8_t) 0x00)

  } CCR1L;


  /** TIM2 capture/compare reg (CCR2H at 0x5311) */
  union {

    /// bytewise access to CCR2H
    uint8_t  byte;

    /// bitwise access to register CCR2H
    struct {
      BITS   CCR2                : 8;      // bits 0-7
    };  // CCR2H bitfield

    /// register _TIM2_CCR2H reset value
    #define sfr_TIM2_CCR2H_RESET_VALUE   ((uint8_t) 0x00)

  } CCR2H;


  /** TIM2 capture/compare register 2 low (CCR2L at 0x5312) */
  union {

    /// bytewise access to CCR2L
    uint8_t  byte;

    /// bitwise access to register CCR2L
    struct {
      BITS   CCR2                : 8;      // bits 0-7
    };  // CCR2L bitfield

    /// register _TIM2_CCR2L reset value
    #define sfr_TIM2_CCR2L_RESET_VALUE   ((uint8_t) 0x00)

  } CCR2L;


  /** TIM2 capture/compare register 3 high (CCR3H at 0x5313) */
  union {

    /// bytewise access to CCR3H
    uint8_t  byte;

    /// bitwise access to register CCR3H
    struct {
      BITS   CCR3                : 8;      // bits 0-7
    };  // CCR3H bitfield

    /// register _TIM2_CCR3H reset value
    #define sfr_TIM2_CCR3H_RESET_VALUE   ((uint8_t) 0x00)

  } CCR3H;


  /** TIM2 capture/compare register 3 low (CCR3L at 0x5314) */
  union {

    /// bytewise access to CCR3L
    uint8_t  byte;

    /// bitwise access to register CCR3L
    struct {
      BITS   CCR3                : 8;      // bits 0-7
    };  // CCR3L bitfield

    /// register _TIM2_CCR3L reset value
    #define sfr_TIM2_CCR3L_RESET_VALUE   ((uint8_t) 0x00)

  } CCR3L;

} TIM2_t;

/// access to TIM2 SFR registers
#define sfr_TIM2   (*((TIM2_t*) 0x5300))


//------------------------
// Module TIM3
//------------------------

/** struct containing TIM3 module registers */
typedef struct {

  /** TIM3 control register 1 (CR1 at 0x5320) */
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

    /// register _TIM3_CR1 reset value
    #define sfr_TIM3_CR1_RESET_VALUE   ((uint8_t) 0x00)

  } CR1;


  /** TIM3 interrupt enable register (IER at 0x5321) */
  union {

    /// bytewise access to IER
    uint8_t  byte;

    /// bitwise access to register IER
    struct {
      BITS   UIE                 : 1;      // bit 0
      BITS   CC1IE               : 1;      // bit 1
      BITS   CC2IE               : 1;      // bit 2
      BITS   CC3IE               : 1;      // bit 3
      BITS                       : 2;      // 2 bits
      BITS   TIE                 : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // IER bitfield

    /// register _TIM3_IER reset value
    #define sfr_TIM3_IER_RESET_VALUE   ((uint8_t) 0x00)

  } IER;


  /** TIM3 status register 1 (SR1 at 0x5322) */
  union {

    /// bytewise access to SR1
    uint8_t  byte;

    /// bitwise access to register SR1
    struct {
      BITS   UIF                 : 1;      // bit 0
      BITS   CC1IF               : 1;      // bit 1
      BITS   CC2IF               : 1;      // bit 2
      BITS   CC3IF               : 1;      // bit 3
      BITS                       : 2;      // 2 bits
      BITS   TIF                 : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // SR1 bitfield

    /// register _TIM3_SR1 reset value
    #define sfr_TIM3_SR1_RESET_VALUE   ((uint8_t) 0x00)

  } SR1;


  /** TIM3 status register 2 (SR2 at 0x5323) */
  union {

    /// bytewise access to SR2
    uint8_t  byte;

    /// bitwise access to register SR2
    struct {
      BITS                       : 1;      // 1 bit
      BITS   CC1OF               : 1;      // bit 1
      BITS   CC2OF               : 1;      // bit 2
      BITS   CC3OF               : 1;      // bit 3
      BITS                       : 4;      // 4 bits
    };  // SR2 bitfield

    /// register _TIM3_SR2 reset value
    #define sfr_TIM3_SR2_RESET_VALUE   ((uint8_t) 0x00)

  } SR2;


  /** TIM3 event generation register (EGR at 0x5324) */
  union {

    /// bytewise access to EGR
    uint8_t  byte;

    /// bitwise access to register EGR
    struct {
      BITS   UG                  : 1;      // bit 0
      BITS   CC1G                : 1;      // bit 1
      BITS   CC2G                : 1;      // bit 2
      BITS   CC3G                : 1;      // bit 3
      BITS                       : 2;      // 2 bits
      BITS   TG                  : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // EGR bitfield

    /// register _TIM3_EGR reset value
    #define sfr_TIM3_EGR_RESET_VALUE   ((uint8_t) 0x00)

  } EGR;


  /** TIM3 capture/compare mode register 1 (CCMR1 at 0x5325) */
  union {

    /// bytewise access to CCMR1
    uint8_t  byte;

    /// bitwise access to register CCMR1
    struct {
      BITS   CC1S                : 2;      // bits 0-1
      BITS                       : 1;      // 1 bit
      BITS   OC1PE               : 1;      // bit 3
      BITS   OC1M                : 3;      // bits 4-6
      BITS                       : 1;      // 1 bit
    };  // CCMR1 bitfield

    /// register _TIM3_CCMR1 reset value
    #define sfr_TIM3_CCMR1_RESET_VALUE   ((uint8_t) 0x00)

  } CCMR1;


  /** TIM3 capture/compare mode register 2 (CCMR2 at 0x5326) */
  union {

    /// bytewise access to CCMR2
    uint8_t  byte;

    /// bitwise access to register CCMR2
    struct {
      BITS   CC2S                : 2;      // bits 0-1
      BITS                       : 1;      // 1 bit
      BITS   OC2PE               : 1;      // bit 3
      BITS   OC2M                : 3;      // bits 4-6
      BITS                       : 1;      // 1 bit
    };  // CCMR2 bitfield

    /// register _TIM3_CCMR2 reset value
    #define sfr_TIM3_CCMR2_RESET_VALUE   ((uint8_t) 0x00)

  } CCMR2;


  /** TIM3 capture/compare enable register 1 (CCER1 at 0x5327) */
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


  /** TIM3 counter high (CNTRH at 0x5328) */
  union {

    /// bytewise access to CNTRH
    uint8_t  byte;

    /// bitwise access to register CNTRH
    struct {
      BITS   CNT                 : 8;      // bits 0-7
    };  // CNTRH bitfield

    /// register _TIM3_CNTRH reset value
    #define sfr_TIM3_CNTRH_RESET_VALUE   ((uint8_t) 0x00)

  } CNTRH;


  /** TIM3 counter low (CNTRL at 0x5329) */
  union {

    /// bytewise access to CNTRL
    uint8_t  byte;

    /// bitwise access to register CNTRL
    struct {
      BITS   CNT                 : 8;      // bits 0-7
    };  // CNTRL bitfield

    /// register _TIM3_CNTRL reset value
    #define sfr_TIM3_CNTRL_RESET_VALUE   ((uint8_t) 0x00)

  } CNTRL;


  /** TIM3 prescaler register (PSCR at 0x532a) */
  union {

    /// bytewise access to PSCR
    uint8_t  byte;

    /// bitwise access to register PSCR
    struct {
      BITS   PSC                 : 4;      // bits 0-3
      BITS                       : 4;      // 4 bits
    };  // PSCR bitfield

    /// register _TIM3_PSCR reset value
    #define sfr_TIM3_PSCR_RESET_VALUE   ((uint8_t) 0x00)

  } PSCR;


  /** TIM3 auto-reload register high (ARRH at 0x532b) */
  union {

    /// bytewise access to ARRH
    uint8_t  byte;

    /// bitwise access to register ARRH
    struct {
      BITS   ARR                 : 8;      // bits 0-7
    };  // ARRH bitfield

    /// register _TIM3_ARRH reset value
    #define sfr_TIM3_ARRH_RESET_VALUE   ((uint8_t) 0xFF)

  } ARRH;


  /** TIM3 auto-reload register low (ARRL at 0x532c) */
  union {

    /// bytewise access to ARRL
    uint8_t  byte;

    /// bitwise access to register ARRL
    struct {
      BITS   ARR                 : 8;      // bits 0-7
    };  // ARRL bitfield

    /// register _TIM3_ARRL reset value
    #define sfr_TIM3_ARRL_RESET_VALUE   ((uint8_t) 0xFF)

  } ARRL;


  /** TIM3 capture/compare register 1 high (CCR1H at 0x532d) */
  union {

    /// bytewise access to CCR1H
    uint8_t  byte;

    /// bitwise access to register CCR1H
    struct {
      BITS   CCR1                : 8;      // bits 0-7
    };  // CCR1H bitfield

    /// register _TIM3_CCR1H reset value
    #define sfr_TIM3_CCR1H_RESET_VALUE   ((uint8_t) 0x00)

  } CCR1H;


  /** TIM3 capture/compare register 1 low (CCR1L at 0x532e) */
  union {

    /// bytewise access to CCR1L
    uint8_t  byte;

    /// bitwise access to register CCR1L
    struct {
      BITS   CCR1                : 8;      // bits 0-7
    };  // CCR1L bitfield

    /// register _TIM3_CCR1L reset value
    #define sfr_TIM3_CCR1L_RESET_VALUE   ((uint8_t) 0x00)

  } CCR1L;


  /** TIM3 capture/compare register 2 high (CCR2H at 0x532f) */
  union {

    /// bytewise access to CCR2H
    uint8_t  byte;

    /// bitwise access to register CCR2H
    struct {
      BITS   CCR2                : 8;      // bits 0-7
    };  // CCR2H bitfield

    /// register _TIM3_CCR2H reset value
    #define sfr_TIM3_CCR2H_RESET_VALUE   ((uint8_t) 0x00)

  } CCR2H;


  /** TIM3 capture/compare register 2 low (CCR2L at 0x5330) */
  union {

    /// bytewise access to CCR2L
    uint8_t  byte;

    /// bitwise access to register CCR2L
    struct {
      BITS   CCR2                : 8;      // bits 0-7
    };  // CCR2L bitfield

    /// register _TIM3_CCR2L reset value
    #define sfr_TIM3_CCR2L_RESET_VALUE   ((uint8_t) 0x00)

  } CCR2L;

} TIM3_t;

/// access to TIM3 SFR registers
#define sfr_TIM3   (*((TIM3_t*) 0x5320))


//------------------------
// Module TIM4
//------------------------

/** struct containing TIM4 module registers */
typedef struct {

  /** TIM4 control register 1 (CR1 at 0x5340) */
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


  /** TIM4 interrupt enable register (IER at 0x5341) */
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


  /** TIM4 status register (SR at 0x5342) */
  union {

    /// bytewise access to SR
    uint8_t  byte;

    /// bitwise access to register SR
    struct {
      BITS   UIF                 : 1;      // bit 0
      BITS                       : 5;      // 5 bits
      BITS   TIF                 : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // SR bitfield

    /// register _TIM4_SR reset value
    #define sfr_TIM4_SR_RESET_VALUE   ((uint8_t) 0x00)

  } SR;


  /** TIM4 event generation register (EGR at 0x5343) */
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


  /** TIM4 counter (CNTR at 0x5344) */
  union {

    /// bytewise access to CNTR
    uint8_t  byte;

    /// bitwise access to register CNTR
    struct {
      BITS   CNT                 : 8;      // bits 0-7
    };  // CNTR bitfield

    /// register _TIM4_CNTR reset value
    #define sfr_TIM4_CNTR_RESET_VALUE   ((uint8_t) 0x00)

  } CNTR;


  /** TIM4 prescaler register (PSCR at 0x5345) */
  union {

    /// bytewise access to PSCR
    uint8_t  byte;

    /// bitwise access to register PSCR
    struct {
      BITS   PSC                 : 3;      // bits 0-2
      BITS                       : 5;      // 5 bits
    };  // PSCR bitfield

    /// register _TIM4_PSCR reset value
    #define sfr_TIM4_PSCR_RESET_VALUE   ((uint8_t) 0x00)

  } PSCR;


  /** TIM4 auto-reload register (ARR at 0x5346) */
  union {

    /// bytewise access to ARR
    uint8_t  byte;

    /// bitwise access to register ARR
    struct {
      BITS   ARR                 : 8;      // bits 0-7
    };  // ARR bitfield

    /// register _TIM4_ARR reset value
    #define sfr_TIM4_ARR_RESET_VALUE   ((uint8_t) 0xFF)

  } ARR;

} TIM4_t;

/// access to TIM4 SFR registers
#define sfr_TIM4   (*((TIM4_t*) 0x5340))


//------------------------
// Module UART2
//------------------------

/** struct containing UART2 module registers */
typedef struct {

  /** UART2 status register (SR at 0x5240) */
  union {

    /// bytewise access to SR
    uint8_t  byte;

    /// bitwise access to register SR
    struct {
      BITS   PE                  : 1;      // bit 0
      BITS   FE                  : 1;      // bit 1
      BITS   NF                  : 1;      // bit 2
      BITS   OR_LHE              : 1;      // bit 3
      BITS   IDLE                : 1;      // bit 4
      BITS   RXNE                : 1;      // bit 5
      BITS   TC                  : 1;      // bit 6
      BITS   TXE                 : 1;      // bit 7
    };  // SR bitfield

    /// register _UART2_SR reset value
    #define sfr_UART2_SR_RESET_VALUE   ((uint8_t) 0xC0)

  } SR;


  /** UART2 data register (DR at 0x5241) */
  union {

    /// bytewise access to DR
    uint8_t  byte;

    /// bitwise access to register DR
    struct {
      BITS   DR                  : 8;      // bits 0-7
    };  // DR bitfield

    /// register _UART2_DR reset value
    #define sfr_UART2_DR_RESET_VALUE   ((uint8_t) 0x00)

  } DR;


  /** UART2 baud rate register 1 (BRR1 at 0x5242) */
  union {

    /// bytewise access to BRR1
    uint8_t  byte;

    /// bitwise access to register BRR1
    struct {
      BITS   UART_DIV            : 8;      // bits 0-7
    };  // BRR1 bitfield

    /// register _UART2_BRR1 reset value
    #define sfr_UART2_BRR1_RESET_VALUE   ((uint8_t) 0x00)

  } BRR1;


  /** UART2 baud rate register 2 (BRR2 at 0x5243) */
  union {

    /// bytewise access to BRR2
    uint8_t  byte;

    /// bitwise access to register BRR2
    struct {
      BITS   UART_DIV            : 8;      // bits 0-7
    };  // BRR2 bitfield

    /// register _UART2_BRR2 reset value
    #define sfr_UART2_BRR2_RESET_VALUE   ((uint8_t) 0x00)

  } BRR2;


  /** UART2 control register 1 (CR1 at 0x5244) */
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
      BITS   UART0               : 1;      // bit 5
      BITS   T8                  : 1;      // bit 6
      BITS   R8                  : 1;      // bit 7
    };  // CR1 bitfield

    /// register _UART2_CR1 reset value
    #define sfr_UART2_CR1_RESET_VALUE   ((uint8_t) 0x00)

  } CR1;


  /** UART2 control register 2 (CR2 at 0x5245) */
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

    /// register _UART2_CR2 reset value
    #define sfr_UART2_CR2_RESET_VALUE   ((uint8_t) 0x00)

  } CR2;


  /** UART2 control register 3 (CR3 at 0x5246) */
  union {

    /// bytewise access to CR3
    uint8_t  byte;

    /// bitwise access to register CR3
    struct {
      BITS   LBCL                : 1;      // bit 0
      BITS   CPHA                : 1;      // bit 1
      BITS   CPOL                : 1;      // bit 2
      BITS   CKEN                : 1;      // bit 3
      BITS   STOP                : 2;      // bits 4-5
      BITS                       : 1;      // 1 bit
      BITS   LINEN               : 1;      // bit 7
    };  // CR3 bitfield

    /// register _UART2_CR3 reset value
    #define sfr_UART2_CR3_RESET_VALUE   ((uint8_t) 0x00)

  } CR3;


  /** UART2 control register 4 (CR4 at 0x5247) */
  union {

    /// bytewise access to CR4
    uint8_t  byte;

    /// bitwise access to register CR4
    struct {
      BITS   ADD                 : 4;      // bits 0-3
      BITS   LBDF                : 1;      // bit 4
      BITS   LBDL                : 1;      // bit 5
      BITS   LBDIEN              : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // CR4 bitfield

    /// register _UART2_CR4 reset value
    #define sfr_UART2_CR4_RESET_VALUE   ((uint8_t) 0x00)

  } CR4;


  /// Reserved register (1B)
  uint8_t     Reserved_1[1];


  /** UART2 control register 6 (CR6 at 0x5249) */
  union {

    /// bytewise access to CR6
    uint8_t  byte;

    /// bitwise access to register CR6
    struct {
      BITS   LSF                 : 1;      // bit 0
      BITS   LHDF                : 1;      // bit 1
      BITS   LHDIEN              : 1;      // bit 2
      BITS                       : 1;      // 1 bit
      BITS   LASE                : 1;      // bit 4
      BITS   LSLV                : 1;      // bit 5
      BITS                       : 1;      // 1 bit
      BITS   LDUM                : 1;      // bit 7
    };  // CR6 bitfield

    /// register _UART2_CR6 reset value
    #define sfr_UART2_CR6_RESET_VALUE   ((uint8_t) 0x00)

  } CR6;


  /** UART2 guard time register (GTR at 0x524a) */
  union {

    /// bytewise access to GTR
    uint8_t  byte;

    /// bitwise access to register GTR
    struct {
      BITS   GT                  : 8;      // bits 0-7
    };  // GTR bitfield

    /// register _UART2_GTR reset value
    #define sfr_UART2_GTR_RESET_VALUE   ((uint8_t) 0x00)

  } GTR;


  /** UART2 prescaler register (PSCR at 0x524b) */
  union {

    /// bytewise access to PSCR
    uint8_t  byte;

    /// bitwise access to register PSCR
    struct {
      BITS   PSC                 : 8;      // bits 0-7
    };  // PSCR bitfield

    /// register _UART2_PSCR reset value
    #define sfr_UART2_PSCR_RESET_VALUE   ((uint8_t) 0x00)

  } PSCR;

} UART2_t;

/// access to UART2 SFR registers
#define sfr_UART2   (*((UART2_t*) 0x5240))


//------------------------
// Module WWDG
//------------------------

/** struct containing WWDG module registers */
typedef struct {

  /** WWDG control register (CR at 0x50d1) */
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
      BITS   WDGA                : 1;      // bit 7
    };  // CR bitfield

    /// register _WWDG_CR reset value
    #define sfr_WWDG_CR_RESET_VALUE   ((uint8_t) 0x7F)

  } CR;


  /** WWDR window register (WR at 0x50d2) */
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
      BITS                       : 1;      // 1 bit
    };  // WR bitfield

    /// register _WWDG_WR reset value
    #define sfr_WWDG_WR_RESET_VALUE   ((uint8_t) 0x7F)

  } WR;

} WWDG_t;

/// access to WWDG SFR registers
#define sfr_WWDG   (*((WWDG_t*) 0x50d1))


// undefine local macros
#undef  BITS

// required for C++
#ifdef __cplusplus
  }   // extern "C"
#endif

/*-------------------------------------------------------------------------
  END OF MODULE DEFINITION FOR MULTIPLE INLUSION
-------------------------------------------------------------------------*/
#endif // STM8S105C6_H
