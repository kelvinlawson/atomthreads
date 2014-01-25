/**
  ******************************************************************************
  * @file    stm8s_itc.h
  * @author  MCD Application Team
  * @version V2.1.0
  * @date    18-November-2011
  * @brief   This file contains all functions prototype and macros for the ITC peripheral.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
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
  * @brief  ITC Interrupt Lines selection
  */
typedef enum {
  ITC_IRQ_TLI            = (uint8_t)0,   /*!< Software interrupt */
  ITC_IRQ_AWU            = (uint8_t)1,   /*!< Auto wake up from halt interrupt */
  ITC_IRQ_CLK            = (uint8_t)2,   /*!< Clock controller interrupt */
  ITC_IRQ_PORTA          = (uint8_t)3,   /*!< Port A external interrupts */
  ITC_IRQ_PORTB          = (uint8_t)4,   /*!< Port B external interrupts */
  ITC_IRQ_PORTC          = (uint8_t)5,   /*!< Port C external interrupts */
  ITC_IRQ_PORTD          = (uint8_t)6,   /*!< Port D external interrupts */
  ITC_IRQ_PORTE          = (uint8_t)7,   /*!< Port E external interrupts */
  
#if defined(STM8S208) || defined(STM8AF52Ax)
  ITC_IRQ_CAN_RX         = (uint8_t)8,   /*!< beCAN RX interrupt */
  ITC_IRQ_CAN_TX         = (uint8_t)9,   /*!< beCAN TX/ER/SC interrupt */
#endif /*STM8S208 or STM8AF52Ax */

#ifdef STM8S903
  ITC_IRQ_PORTF          = (uint8_t)8,   /*!< Port F external interrupts */
#endif /*STM8S903*/

  ITC_IRQ_SPI            = (uint8_t)10,  /*!< SPI interrupt */
  ITC_IRQ_TIM1_OVF       = (uint8_t)11,  /*!< TIM1 update/overflow/underflow/trigger/
                                         break interrupt*/
  ITC_IRQ_TIM1_CAPCOM    = (uint8_t)12,  /*!< TIM1 capture/compare interrupt */
  
#ifdef STM8S903
  ITC_IRQ_TIM5_OVFTRI    = (uint8_t)13,  /*!< TIM5 update/overflow/underflow/trigger/
                                         interrupt */
  ITC_IRQ_TIM5_CAPCOM    = (uint8_t)14,  /*!< TIM5 capture/compare interrupt */
#else  
  ITC_IRQ_TIM2_OVF       = (uint8_t)13,  /*!< TIM2 update /overflow interrupt */
  ITC_IRQ_TIM2_CAPCOM    = (uint8_t)14,  /*!< TIM2 capture/compare interrupt */
#endif /*STM8S903*/

  ITC_IRQ_TIM3_OVF       = (uint8_t)15,  /*!< TIM3 update /overflow interrupt*/
  ITC_IRQ_TIM3_CAPCOM    = (uint8_t)16,  /*!< TIM3 update /overflow interrupt */
  ITC_IRQ_UART1_TX       = (uint8_t)17,  /*!< USART1 TX interrupt */
  ITC_IRQ_UART1_RX       = (uint8_t)18,  /*!< USART1 RX interrupt */
  ITC_IRQ_I2C            = (uint8_t)19,  /*!< I2C interrupt */
  
#if defined(STM8S105) || defined(STM8S005) || defined(STM8AF626x)
  ITC_IRQ_UART2_TX       = (uint8_t)20,  /*!< USART2 TX interrupt */
  ITC_IRQ_UART2_RX       = (uint8_t)21,  /*!< USART2 RX interrupt */
#endif /*STM8S105 or STM8AF626x */

#if defined(STM8S208) || defined(STM8S207) || defined(STM8S007) || defined(STM8AF52Ax) || defined(STM8AF62Ax)
  ITC_IRQ_UART3_TX       = (uint8_t)20,  /*!< USART3 TX interrupt */
  ITC_IRQ_UART3_RX       = (uint8_t)21,  /*!< USART3 RX interrupt */
  ITC_IRQ_ADC2           = (uint8_t)22,  /*!< ADC2 interrupt */
#endif /*STM8S208 or STM8S207 or STM8AF52Ax or STM8AF62Ax */

#if defined(STM8S105) || defined(STM8S005) || defined(STM8S103) || defined(STM8S003) ||  defined(STM8S903) || defined(STM8AF626x)  
  ITC_IRQ_ADC1           = (uint8_t)22,  /*!< ADC2 interrupt */
#endif /*STM8S105, STM8S103 or STM8S903 or STM8AF626x */

#ifdef STM8S903
  ITC_IRQ_TIM6_OVFTRI    = (uint8_t)23,  /*!< TIM6 update/overflow/underflow/trigger/
                                         interrupt */
#else
  ITC_IRQ_TIM4_OVF       = (uint8_t)23,  /*!< TIM4 update /overflow interrupt */
#endif /*STM8S903*/

  ITC_IRQ_EEPROM_EEC     = (uint8_t)24  /*!< Flash interrupt */
} ITC_Irq_TypeDef;

/**
  * @brief  ITC Priority Levels selection
  */
typedef enum {
  ITC_PRIORITYLEVEL_0 = (uint8_t)0x02, /*!< Software priority level 0 (cannot be written) */
  ITC_PRIORITYLEVEL_1 = (uint8_t)0x01, /*!< Software priority level 1 */
  ITC_PRIORITYLEVEL_2 = (uint8_t)0x00, /*!< Software priority level 2 */
  ITC_PRIORITYLEVEL_3 = (uint8_t)0x03  /*!< Software priority level 3 */
} ITC_PriorityLevel_TypeDef;

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/

/** @addtogroup ITC_Exported_Constants
  * @{
  */

#define CPU_SOFT_INT_DISABLED ((uint8_t)0x28) /*!< Mask for I1 and I0 bits in CPU_CC register */

/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/

/**
  * @brief  Macros used by the assert function in order to check the different functions parameters.
  * @addtogroup ITC_Private_Macros
  * @{
  */

/* Used by assert function */
#define IS_ITC_IRQ_OK(IRQ) ((IRQ) <= (uint8_t)24)

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

uint8_t ITC_GetCPUCC(void);
void ITC_DeInit(void);
uint8_t ITC_GetSoftIntStatus(void);
void ITC_SetSoftwarePriority(ITC_Irq_TypeDef IrqNum, ITC_PriorityLevel_TypeDef PriorityValue);
ITC_PriorityLevel_TypeDef ITC_GetSoftwarePriority(ITC_Irq_TypeDef IrqNum);

/**
  * @}
  */

#endif /* __STM8S_ITC_H__ */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
