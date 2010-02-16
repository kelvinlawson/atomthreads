/**
  ******************************************************************************
  * @file stm8s_clk.h
  * @brief This file contains all functions prototype and macros for the CLK peripheral.
  * @author STMicroelectronics - MCD Application Team
  * @version V1.1.1
  * @date 06/05/2009
  ******************************************************************************
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  * @image html logo.bmp
  ******************************************************************************
  */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM8S_CLK_H
#define __STM8S_CLK_H

/* Includes ------------------------------------------------------------------*/
/* Contains the description of all STM8 hardware registers */
#include "stm8s.h"

/* Exported types ------------------------------------------------------------*/
/** @addtogroup CLK_Exported_Types
  * @{
  */

/**
   * @brief Switch Mode Auto, Manual.
   */
typedef enum {
  CLK_SWITCHMODE_MANUAL = (u8)0x00, /*!< Enable the manual clock switching mode */
  CLK_SWITCHMODE_AUTO   = (u8)0x01  /*!< Enable the automatic clock switching mode */
} CLK_SwitchMode_TypeDef;

/**
   * @brief Current Clock State.
   */
typedef enum {
  CLK_CURRENTCLOCKSTATE_DISABLE = (u8)0x00, /*!< Current clock disable */
  CLK_CURRENTCLOCKSTATE_ENABLE  = (u8)0x01  /*!< Current clock enable */
} CLK_CurrentClockState_TypeDef;

/**
   * @brief  Clock security system configuration.
   */
typedef enum {
  CLK_CSSCONFIG_ENABLEWITHIT = (u8)0x05, /*!< Enable CSS with detection interrupt */
  CLK_CSSCONFIG_ENABLE    = (u8)0x01, /*!< Enable CSS without detection interrupt */
  CLK_CSSCONFIG_DISABLE      = (u8)0x00  /*!< Leave CSS desactivated (to be used in CLK_Init() function) */
} CLK_CSSConfig_TypeDef;

/**
   * @brief  CLK Clock Source.
   */
typedef enum {
  CLK_SOURCE_HSI    = (u8)0xE1, /*!< Clock Source HSI. */
  CLK_SOURCE_LSI    = (u8)0xD2, /*!< Clock Source LSI. */
  CLK_SOURCE_HSE    = (u8)0xB4 /*!< Clock Source HSE. */
} CLK_Source_TypeDef;

/**
   * @brief  CLK HSI Calibration Value.
   */
typedef enum {
  CLK_HSITRIMVALUE_0   = (u8)0x00, /*!< HSI Calibtation Value 0 */
  CLK_HSITRIMVALUE_1   = (u8)0x01, /*!< HSI Calibtation Value 1 */
  CLK_HSITRIMVALUE_2   = (u8)0x02, /*!< HSI Calibtation Value 2 */
  CLK_HSITRIMVALUE_3   = (u8)0x03, /*!< HSI Calibtation Value 3 */
  CLK_HSITRIMVALUE_4   = (u8)0x04, /*!< HSI Calibtation Value 4 */
  CLK_HSITRIMVALUE_5   = (u8)0x05, /*!< HSI Calibtation Value 5 */
  CLK_HSITRIMVALUE_6   = (u8)0x06, /*!< HSI Calibtation Value 6 */
  CLK_HSITRIMVALUE_7   = (u8)0x07  /*!< HSI Calibtation Value 7 */
} CLK_HSITrimValue_TypeDef;

/**
   * @brief   CLK  Clock Output
   */
typedef enum {
  CLK_OUTPUT_HSI      = (u8)0x00, /*!< Clock Output HSI */
  CLK_OUTPUT_LSI      = (u8)0x02, /*!< Clock Output LSI */
  CLK_OUTPUT_HSE      = (u8)0x04, /*!< Clock Output HSE */
  CLK_OUTPUT_CPU      = (u8)0x08, /*!< Clock Output CPU */
  CLK_OUTPUT_CPUDIV2  = (u8)0x0A, /*!< Clock Output CPU/2 */
  CLK_OUTPUT_CPUDIV4  = (u8)0x0C, /*!< Clock Output CPU/4 */
  CLK_OUTPUT_CPUDIV8  = (u8)0x0E, /*!< Clock Output CPU/8 */
  CLK_OUTPUT_CPUDIV16 = (u8)0x10, /*!< Clock Output CPU/16 */
  CLK_OUTPUT_CPUDIV32 = (u8)0x12, /*!< Clock Output CPU/32 */
  CLK_OUTPUT_CPUDIV64 = (u8)0x14, /*!< Clock Output CPU/64 */
  CLK_OUTPUT_HSIRC    = (u8)0x16, /*!< Clock Output HSI RC */
  CLK_OUTPUT_MASTER   = (u8)0x18, /*!< Clock Output Master */
  CLK_OUTPUT_OTHERS   = (u8)0x1A  /*!< Clock Output OTHER */
} CLK_Output_TypeDef;

/**
   * @brief   CLK Enable peripheral
   */
/* Elements values convention: 0xXY
    X = choice between the peripheral registers
        X = 0 : PCKENR1
        X = 1 : PCKENR2
    Y = Peripheral position in the register
*/
typedef enum {
  CLK_PERIPHERAL_I2C    = (u8)0x00, /*!< Peripheral Clock Enable 1, I2C */
  CLK_PERIPHERAL_SPI    = (u8)0x01, /*!< Peripheral Clock Enable 1, SPI */
#if defined(STM8S208) || defined(STM8S207)	
  CLK_PERIPHERAL_UART1   = (u8)0x02, /*!< Peripheral Clock Enable 1, UART1 */
#else
	CLK_PERIPHERAL_UART1   = (u8)0x03, /*!< Peripheral Clock Enable 1, UART1 */
#endif
  CLK_PERIPHERAL_UART2   = (u8)0x03, /*!< Peripheral Clock Enable 1, UART2 */
  CLK_PERIPHERAL_UART3   = (u8)0x03, /*!< Peripheral Clock Enable 1, UART3 */
  CLK_PERIPHERAL_TIMER6 = (u8)0x04, /*!< Peripheral Clock Enable 1, Timer6 */
  CLK_PERIPHERAL_TIMER4 = (u8)0x04, /*!< Peripheral Clock Enable 1, Timer4 */
  CLK_PERIPHERAL_TIMER5 = (u8)0x05, /*!< Peripheral Clock Enable 1, Timer5 */
  CLK_PERIPHERAL_TIMER2 = (u8)0x05, /*!< Peripheral Clock Enable 1, Timer2 */
  CLK_PERIPHERAL_TIMER3 = (u8)0x06, /*!< Peripheral Clock Enable 1, Timer3 */
  CLK_PERIPHERAL_TIMER1 = (u8)0x07, /*!< Peripheral Clock Enable 1, Timer1 */
  CLK_PERIPHERAL_AWU = (u8)0x12, /*!< Peripheral Clock Enable 2, AWU */
  CLK_PERIPHERAL_ADC = (u8)0x13, /*!< Peripheral Clock Enable 2, ADC */
  CLK_PERIPHERAL_CAN = (u8)0x17 /*!< Peripheral Clock Enable 2, CAN */
} CLK_Peripheral_TypeDef;

/**
   * @brief  CLK Flags.
   */
/* Elements values convention: 0xXZZ
    X = choice between the flags registers
        X = 1 : ICKR
        X = 2 : ECKR
        X = 3 : SWCR
    X = 4 : CSSR
 X = 5 : CCOR
   ZZ = flag mask in the register (same as map file)
*/
typedef enum {
  CLK_FLAG_LSIRDY  = (u16)0x0110, /*!< Low speed internal oscillator ready Flag */
  CLK_FLAG_HSIRDY  = (u16)0x0102, /*!< High speed internal oscillator ready Flag */
  CLK_FLAG_HSERDY  = (u16)0x0202, /*!< High speed external oscillator ready Flag */
  CLK_FLAG_SWIF   = (u16)0x0308, /*!< Clock switch interrupt Flag */
  CLK_FLAG_SWBSY    = (u16)0x0301, /*!< Switch busy Flag */
  CLK_FLAG_CSSD   = (u16)0x0408, /*!< Clock security system detection Flag */
  CLK_FLAG_AUX   = (u16)0x0402, /*!< Auxiliary oscillator connected to master clock */
  CLK_FLAG_CCOBSY  = (u16)0x0504, /*!< Configurable clock output busy */
  CLK_FLAG_CCORDY    = (u16)0x0502 /*!< Configurable clock output ready */

}CLK_Flag_TypeDef;

/**
   * @brief CLK interrupt configuration and Flags cleared by software.
   */
typedef enum {
  CLK_IT_CSSD   = (u8)0x0C, /*!< Clock security system detection Flag */
  CLK_IT_SWIF   = (u8)0x1C /*!< Clock switch interrupt Flag */
}CLK_IT_TypeDef;

/**
   * @brief  CLK Clock Divisor.
   */
/* Warning:
   0xxxxxx = HSI divider
   1xxxxxx = CPU divider
   Other bits correspond to the divider's bits mapping
*/
typedef enum {
  CLK_PRESCALER_HSIDIV1   = (u8)0x00, /*!< High speed internal clock prescaler: 1 */
  CLK_PRESCALER_HSIDIV2   = (u8)0x08, /*!< High speed internal clock prescaler: 2 */
  CLK_PRESCALER_HSIDIV4   = (u8)0x10, /*!< High speed internal clock prescaler: 4 */
  CLK_PRESCALER_HSIDIV8   = (u8)0x18, /*!< High speed internal clock prescaler: 8 */
  CLK_PRESCALER_CPUDIV1   = (u8)0x80, /*!< CPU clock division factors 1 */
  CLK_PRESCALER_CPUDIV2   = (u8)0x81, /*!< CPU clock division factors 2 */
  CLK_PRESCALER_CPUDIV4   = (u8)0x82, /*!< CPU clock division factors 4 */
  CLK_PRESCALER_CPUDIV8   = (u8)0x83, /*!< CPU clock division factors 8 */
  CLK_PRESCALER_CPUDIV16  = (u8)0x84, /*!< CPU clock division factors 16 */
  CLK_PRESCALER_CPUDIV32  = (u8)0x85, /*!< CPU clock division factors 32 */
  CLK_PRESCALER_CPUDIV64  = (u8)0x86, /*!< CPU clock division factors 64 */
  CLK_PRESCALER_CPUDIV128 = (u8)0x87  /*!< CPU clock division factors 128 */
} CLK_Prescaler_TypeDef;

/**
   * @brief  SWIM Clock divider.
   */
typedef enum {
  CLK_SWIMDIVIDER_2 = (u8)0x00, /*!< SWIM clock is divided by 2 */
  CLK_SWIMDIVIDER_OTHER = (u8)0x01 /*!< SWIM clock is not divided by 2 */
}CLK_SWIMDivider_TypeDef;

/**
   * @brief External CAN clock dividern.
   */
typedef enum{
  CLK_CANDIVIDER_1 = (u8)0x00, /*!< External CAN clock = HSE/1 */
  CLK_CANDIVIDER_2 = (u8)0x01, /*!< External CAN clock = HSE/2 */
  CLK_CANDIVIDER_3 = (u8)0x02, /*!< External CAN clock = HSE/3 */
  CLK_CANDIVIDER_4 = (u8)0x03, /*!< External CAN clock = HSE/4 */
  CLK_CANDIVIDER_5 = (u8)0x04, /*!< External CAN clock = HSE/5 */
  CLK_CANDIVIDER_6 = (u8)0x05, /*!< External CAN clock = HSE/6 */
  CLK_CANDIVIDER_7 = (u8)0x06, /*!< External CAN clock = HSE/7 */
  CLK_CANDIVIDER_8 = (u8)0x07  /*!< External CAN clock = HSE/8 */
}CLK_CANDivider_TypeDef;

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/

/** @addtogroup CLK_Exported_Constants
  * @{
  */

#define HSI_VALUE   ((u32)16000000) /*!< Typical Value of the HSI in Hz */
#define LSI_VALUE   ((u32)128000)   /*!< Typical Value of the LSI in Hz */
#define CLK_TIMEOUT ((u16)0x491)    /*!< Timeout for the clock switch operation. */
/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/** @addtogroup CLK_Private_Macros
  * @{
  */

/**
  * @brief Macros used by the assert function in order to check the different functions parameters.
  */

/**
  * @brief Macros used by the assert function in order to check the clock switching modes.
  */
#define IS_CLK_SWITCHMODE_OK(MODE) (((MODE) == CLK_SWITCHMODE_MANUAL) || ((MODE) == CLK_SWITCHMODE_AUTO))

/**
  * @brief Macros used by the assert function in order to check the current clock state.
  */
#define IS_CLK_CURRENTCLOCKSTATE_OK(STATE) (((STATE) == CLK_CURRENTCLOCKSTATE_DISABLE) || ((STATE) == CLK_CURRENTCLOCKSTATE_ENABLE))

/**
  * @brief Macros used by the assert function in order to check the CSS configuration.
  */
#define IS_CLK_CSSCONFIG_OK(CSSVALUE) (((CSSVALUE) == CLK_CSSCONFIG_ENABLEWITHIT) ||\
                                       ((CSSVALUE) == CLK_CSSCONFIG_ENABLE) ||\
                                       ((CSSVALUE) == CLK_CSSCONFIG_DISABLE))

/**
  * @brief Macros used by the assert function in order to check the different clock sources.
  */
#define IS_CLK_SOURCE_OK(SOURCE) (((SOURCE) == CLK_SOURCE_HSI) ||\
                                  ((SOURCE) == CLK_SOURCE_LSI) ||\
                                  ((SOURCE) == CLK_SOURCE_HSE))

/**
  * @brief Macros used by the assert function in order to check the different HSI trimming values.
  */
#define IS_CLK_HSITRIMVALUE_OK(TRIMVALUE) (((TRIMVALUE) == CLK_HSITRIMVALUE_0) ||\
    ((TRIMVALUE) == CLK_HSITRIMVALUE_1) ||\
    ((TRIMVALUE) == CLK_HSITRIMVALUE_2) ||\
    ((TRIMVALUE) == CLK_HSITRIMVALUE_3) ||\
    ((TRIMVALUE) == CLK_HSITRIMVALUE_4) ||\
    ((TRIMVALUE) == CLK_HSITRIMVALUE_5) ||\
    ((TRIMVALUE) == CLK_HSITRIMVALUE_6) ||\
    ((TRIMVALUE) == CLK_HSITRIMVALUE_7))

/**
  * @brief Macros used by the assert function in order to check the different clocks to output.
  */
#define IS_CLK_OUTPUT_OK(OUTPUT) (((OUTPUT) == CLK_OUTPUT_HSI) ||\
                                  ((OUTPUT) == CLK_OUTPUT_HSE) ||\
                                  ((OUTPUT) == CLK_OUTPUT_LSI) ||\
                                  ((OUTPUT) == CLK_OUTPUT_CPU) ||\
                                  ((OUTPUT) == CLK_OUTPUT_CPUDIV2) ||\
                                  ((OUTPUT) == CLK_OUTPUT_CPUDIV4) ||\
                                  ((OUTPUT) == CLK_OUTPUT_CPUDIV8) ||\
                                  ((OUTPUT) == CLK_OUTPUT_CPUDIV16) ||\
                                  ((OUTPUT) == CLK_OUTPUT_CPUDIV32) ||\
                                  ((OUTPUT) == CLK_OUTPUT_CPUDIV64) ||\
                                  ((OUTPUT) == CLK_OUTPUT_HSIRC) ||\
                                  ((OUTPUT) == CLK_OUTPUT_MASTER) ||\
                                  ((OUTPUT) == CLK_OUTPUT_OTHERS))

/**
  * @brief Macros used by the assert function in order to check the different peripheral's clock.
  */
#define IS_CLK_PERIPHERAL_OK(PERIPHERAL) (((PERIPHERAL) == CLK_PERIPHERAL_I2C) ||\
    ((PERIPHERAL) == CLK_PERIPHERAL_SPI) ||\
    ((PERIPHERAL) == CLK_PERIPHERAL_UART3) ||\
    ((PERIPHERAL) == CLK_PERIPHERAL_UART2) ||\
    ((PERIPHERAL) == CLK_PERIPHERAL_UART1) ||\
    ((PERIPHERAL) == CLK_PERIPHERAL_TIMER4) ||\
    ((PERIPHERAL) == CLK_PERIPHERAL_TIMER2) ||\
    ((PERIPHERAL) == CLK_PERIPHERAL_TIMER5) ||\
    ((PERIPHERAL) == CLK_PERIPHERAL_TIMER6) ||\
    ((PERIPHERAL) == CLK_PERIPHERAL_TIMER3) ||\
    ((PERIPHERAL) == CLK_PERIPHERAL_TIMER1) ||\
    ((PERIPHERAL) == CLK_PERIPHERAL_CAN) ||\
    ((PERIPHERAL) == CLK_PERIPHERAL_ADC) ||\
    ((PERIPHERAL) == CLK_PERIPHERAL_AWU))

/**
  * @brief Macros used by the assert function in order to check the different clock flags.
  */
#define IS_CLK_FLAG_OK(FLAG) (((FLAG) == CLK_FLAG_LSIRDY) ||\
                              ((FLAG) == CLK_FLAG_HSIRDY) ||\
                              ((FLAG) == CLK_FLAG_HSERDY) ||\
                              ((FLAG) == CLK_FLAG_SWIF) ||\
                              ((FLAG) == CLK_FLAG_SWBSY) ||\
                              ((FLAG) == CLK_FLAG_CSSD) ||\
                              ((FLAG) == CLK_FLAG_AUX) ||\
                              ((FLAG) == CLK_FLAG_CCOBSY) ||\
                              ((FLAG) == CLK_FLAG_CCORDY))

/**
  * @brief Macros used by the assert function in order to check the different clock IT pending bits.
  */
#define IS_CLK_IT_OK(IT) (((IT) == CLK_IT_CSSD) || ((IT) == CLK_IT_SWIF))

/**
  * @brief Macros used by the assert function in order to check the different HSI prescaler values.
  */
#define IS_CLK_HSIPRESCALER_OK(PRESCALER) (((PRESCALER) == CLK_PRESCALER_HSIDIV1) ||\
    ((PRESCALER) == CLK_PRESCALER_HSIDIV2) ||\
    ((PRESCALER) == CLK_PRESCALER_HSIDIV4) ||\
    ((PRESCALER) == CLK_PRESCALER_HSIDIV8))

/**
  * @brief Macros used by the assert function in order to check the different clock  prescaler values.
  */
#define IS_CLK_PRESCALER_OK(PRESCALER) (((PRESCALER) == CLK_PRESCALER_HSIDIV1) ||\
                                        ((PRESCALER) == CLK_PRESCALER_HSIDIV2) ||\
                                        ((PRESCALER) == CLK_PRESCALER_HSIDIV4) ||\
                                        ((PRESCALER) == CLK_PRESCALER_HSIDIV8) ||\
                                        ((PRESCALER) == CLK_PRESCALER_CPUDIV1) ||\
                                        ((PRESCALER) == CLK_PRESCALER_CPUDIV2) ||\
                                        ((PRESCALER) == CLK_PRESCALER_CPUDIV4) ||\
                                        ((PRESCALER) == CLK_PRESCALER_CPUDIV8) ||\
                                        ((PRESCALER) == CLK_PRESCALER_CPUDIV16) ||\
                                        ((PRESCALER) == CLK_PRESCALER_CPUDIV32) ||\
                                        ((PRESCALER) == CLK_PRESCALER_CPUDIV64) ||\
                                        ((PRESCALER) == CLK_PRESCALER_CPUDIV128))

/**
  * @brief Macros used by the assert function in order to check the different SWIM dividers values.
  */
#define IS_CLK_SWIMDIVIDER_OK(SWIMDIVIDER) (((SWIMDIVIDER) == CLK_SWIMDIVIDER_2) || ((SWIMDIVIDER) == CLK_SWIMDIVIDER_OTHER))

/**
  * @brief Macros used by the assert function in order to check the different CAN dividers values.
  */
#define IS_CLK_CANDIVIDER_OK(CANDIVIDER) (((CANDIVIDER) == CLK_CANDIVIDER_1) ||\
    ((CANDIVIDER) == CLK_CANDIVIDER_2) ||\
    ((CANDIVIDER) == CLK_CANDIVIDER_3) ||\
    ((CANDIVIDER) == CLK_CANDIVIDER_4) ||\
    ((CANDIVIDER) == CLK_CANDIVIDER_5) ||\
    ((CANDIVIDER) == CLK_CANDIVIDER_6) ||\
    ((CANDIVIDER) == CLK_CANDIVIDER_7) ||\
    ((CANDIVIDER) == CLK_CANDIVIDER_8))

/**
  * @}
  */

/** @addtogroup CLK_Exported_functions
  * @{
  */
void CLK_DeInit(void);
void CLK_HSECmd(FunctionalState NewState);
void CLK_HSICmd(FunctionalState NewState);
void CLK_LSICmd(FunctionalState NewState);
void CLK_CCOCmd(FunctionalState NewState);
void CLK_ClockSwitchCmd(FunctionalState NewState);
void CLK_FastHaltWakeUpCmd(FunctionalState NewState);
void CLK_SlowActiveHaltWakeUpCmd(FunctionalState NewState);
void CLK_PeripheralClockConfig(CLK_Peripheral_TypeDef CLK_Peripheral, FunctionalState NewState);
ErrorStatus CLK_ClockSwitchConfig(CLK_SwitchMode_TypeDef CLK_SwitchMode, CLK_Source_TypeDef CLK_NewClock, FunctionalState ITState, CLK_CurrentClockState_TypeDef CLK_CurrentClockState);
void CLK_HSIPrescalerConfig(CLK_Prescaler_TypeDef HSIPrescaler);
void CLK_CCOConfig(CLK_Output_TypeDef CLK_CCO);
void CLK_ITConfig(CLK_IT_TypeDef CLK_IT, FunctionalState NewState);
void CLK_SYSCLKConfig(CLK_Prescaler_TypeDef CLK_Prescaler);
void CLK_SWIMConfig(CLK_SWIMDivider_TypeDef CLK_SWIMDivider);
void CLK_CANConfig(CLK_CANDivider_TypeDef CLK_CANDivider);
void CLK_ClockSecuritySystemEnable(void);
void CLK_SYSCLKEmergencyClear(void);
void CLK_AdjustHSICalibrationValue(CLK_HSITrimValue_TypeDef CLK_HSICalibrationValue);
u32 CLK_GetClockFreq(void);
CLK_Source_TypeDef CLK_GetSYSCLKSource(void);
FlagStatus CLK_GetFlagStatus(CLK_Flag_TypeDef CLK_FLAG);
ITStatus CLK_GetITStatus(CLK_IT_TypeDef CLK_IT);
void CLK_ClearITPendingBit(CLK_IT_TypeDef CLK_IT);

/**
  * @}
  */
#endif /* __STM8S_CLK_H */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
