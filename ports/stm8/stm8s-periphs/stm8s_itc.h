/**
  ******************************************************************************
  * @file stm8s_itc.h
  * @brief This file contains all functions prototype and macros for the ITC peripheral.
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
#ifndef __STM8S_ITC_H__
#define __STM8S_ITC_H__

/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"

/* Exported types ------------------------------------------------------------*/

/** @addtogroup ITC_Exported_Types
  * @{
  */

/**
  * @brief ITC Interrupt Lines selection
  */
typedef enum {
  ITC_IRQ_TLI            = (u8)0,
  ITC_IRQ_AWU            = (u8)1,
  ITC_IRQ_CLK            = (u8)2,
  ITC_IRQ_PORTA          = (u8)3,
  ITC_IRQ_PORTB          = (u8)4,
  ITC_IRQ_PORTC          = (u8)5,
  ITC_IRQ_PORTD          = (u8)6,
  ITC_IRQ_PORTE          = (u8)7,
	
#ifdef STM8S208
  ITC_IRQ_CAN_RX         = (u8)8,
  ITC_IRQ_CAN_TX         = (u8)9,
#endif /*STM8S208*/

#ifdef STM8S903
  ITC_IRQ_PORTF          = (u8)8,
#endif /*STM8S903*/

  ITC_IRQ_SPI            = (u8)10,
  ITC_IRQ_TIM1_OVF       = (u8)11,
  ITC_IRQ_TIM1_CAPCOM    = (u8)12,
	
#ifdef STM8S903
  ITC_IRQ_TIM5_OVFTRI    = (u8)13,
  ITC_IRQ_TIM5_CAPCOM    = (u8)14,
#else	
  ITC_IRQ_TIM2_OVF       = (u8)13,
  ITC_IRQ_TIM2_CAPCOM    = (u8)14,	
#endif /*STM8S903*/

  ITC_IRQ_TIM3_OVF       = (u8)15,
  ITC_IRQ_TIM3_CAPCOM    = (u8)16,
  ITC_IRQ_UART1_TX       = (u8)17,
  ITC_IRQ_UART1_RX       = (u8)18,
  ITC_IRQ_I2C            = (u8)19,
	
#ifdef STM8S105
  ITC_IRQ_UART2_TX       = (u8)20,
  ITC_IRQ_UART2_RX       = (u8)21,
#endif /*STM8S105*/

#if defined(STM8S208) ||defined(STM8S207)
	ITC_IRQ_UART3_TX       = (u8)20,
  ITC_IRQ_UART3_RX       = (u8)21,
  ITC_IRQ_ADC2           = (u8)22,
#endif /*STM8S208 or STM8S207*/

#if defined(STM8S105) ||defined(STM8S103) ||defined(STM8S905)	
  ITC_IRQ_ADC1           = (u8)22,
#endif /*STM8S105, STM8S103 or STM8S905 */

#ifdef STM8S903
	ITC_IRQ_TIM6_OVFTRI    = (u8)23,
#else
	ITC_IRQ_TIM4_OVF       = (u8)23, 
#endif /*STM8S903*/

  ITC_IRQ_EEPROM_EEC     = (u8)24
} ITC_Irq_TypeDef;

/**
  * @brief ITC Priority Levels selection
  */
typedef enum {
  ITC_PRIORITYLEVEL_0 = (u8)0x02, /*!< Software priority level 0 (cannot be written) */
  ITC_PRIORITYLEVEL_1 = (u8)0x01, /*!< Software priority level 1 */
  ITC_PRIORITYLEVEL_2 = (u8)0x00, /*!< Software priority level 2 */
  ITC_PRIORITYLEVEL_3 = (u8)0x03  /*!< Software priority level 3 */
} ITC_PriorityLevel_TypeDef;

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/

/** @addtogroup ITC_Exported_Constants
  * @{
  */

#define CPU_SOFT_INT_DISABLED ((u8)0x28) /*!< Mask for I1 and I0 bits in CPU_CC register */

/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/

/**
  * @brief Macros used by the assert function in order to check the different functions parameters.
  * @addtogroup ITC_Private_Macros
  * @{
  */

/* Used by assert function */
#define IS_ITC_IRQ_OK(IRQ) ((IRQ) <= (u8)24)

/* Used by assert function */
#define IS_ITC_PRIORITY_OK(PriorityValue) \
  (((PriorityValue) == ITC_PRIORITYLEVEL_0) || \
   ((PriorityValue) == ITC_PRIORITYLEVEL_1) || \
   ((PriorityValue) == ITC_PRIORITYLEVEL_2) || \
   ((PriorityValue) == ITC_PRIORITYLEVEL_3))

/* Used by assert function */
#define IS_ITC_INTERRUPTS_DISABLED (ITC_GetSoftIntStatus() == CPU_SOFT_INT_DISABLED)

/**
  * @}
  */

/* Exported functions ------------------------------------------------------- */

/** @addtogroup ITC_Exported_Functions
  * @{
  */

u8 ITC_GetCPUCC(void);
void ITC_DeInit(void);
u8 ITC_GetSoftIntStatus(void);
void ITC_SetSoftwarePriority(ITC_Irq_TypeDef IrqNum, ITC_PriorityLevel_TypeDef PriorityValue);
ITC_PriorityLevel_TypeDef ITC_GetSoftwarePriority(ITC_Irq_TypeDef IrqNum);

/**
  * @}
  */

#endif /* __STM8S_ITC_H__ */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
