/**
  ******************************************************************************
  * @file stm8s_gpio.h
  * @brief This file contains all functions prototype and macros for the GPIO peripheral.
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
#ifndef __STM8S_GPIO_H
#define __STM8S_GPIO_H

/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"

/* Exported variables ------------------------------------------------------- */
/* Exported types ------------------------------------------------------------*/

/** @addtogroup GPIO_Exported_Types
  * @{
  */

/**
  * @brief GPIO modes
  *
  * Bits definitions:
  * - Bit 7: 0 = INPUT mode
  *          1 = OUTPUT mode
  *          1 = PULL-UP (input) or PUSH-PULL (output)
  * - Bit 5: 0 = No external interrupt (input) or No slope control (output)
  *          1 = External interrupt (input) or Slow control enabled (output)
  * - Bit 4: 0 = Low level (output)
  *          1 = High level (output push-pull) or HI-Z (output open-drain)
  */
typedef enum
{
  GPIO_MODE_IN_FL_NO_IT      = (u8)0b00000000,  /*!< Input floating, no external interrupt */
  GPIO_MODE_IN_PU_NO_IT      = (u8)0b01000000,  /*!< Input pull-up, no external interrupt */
  GPIO_MODE_IN_FL_IT         = (u8)0b00100000,  /*!< Input floating, external interrupt */
  GPIO_MODE_IN_PU_IT         = (u8)0b01100000,  /*!< Input pull-up, external interrupt */
  GPIO_MODE_OUT_OD_LOW_FAST  = (u8)0b10100000,  /*!< Output open-drain, low level, 10MHz */
  GPIO_MODE_OUT_PP_LOW_FAST  = (u8)0b11100000,  /*!< Output push-pull, low level, 10MHz */
  GPIO_MODE_OUT_OD_LOW_SLOW  = (u8)0b10000000,  /*!< Output open-drain, low level, 2MHz */
  GPIO_MODE_OUT_PP_LOW_SLOW  = (u8)0b11000000,  /*!< Output push-pull, low level, 2MHz */
  GPIO_MODE_OUT_OD_HIZ_FAST  = (u8)0b10110000,  /*!< Output open-drain, high-impedance level,10MHz */
  GPIO_MODE_OUT_PP_HIGH_FAST = (u8)0b11110000,  /*!< Output push-pull, high level, 10MHz */
  GPIO_MODE_OUT_OD_HIZ_SLOW  = (u8)0b10010000,  /*!< Output open-drain, high-impedance level, 2MHz */
  GPIO_MODE_OUT_PP_HIGH_SLOW = (u8)0b11010000   /*!< Output push-pull, high level, 2MHz */
}GPIO_Mode_TypeDef;

/**
  * @brief Definition of the GPIO pins. Used by the @ref GPIO_Init function in
  * order to select the pins to be initialized.
  */

typedef enum
{
  GPIO_PIN_0    = ((u8)0x01),  /*!< Pin 0 selected */
  GPIO_PIN_1    = ((u8)0x02),  /*!< Pin 1 selected */
  GPIO_PIN_2    = ((u8)0x04),  /*!< Pin 2 selected */
  GPIO_PIN_3    = ((u8)0x08),   /*!< Pin 3 selected */
  GPIO_PIN_4    = ((u8)0x10),  /*!< Pin 4 selected */
  GPIO_PIN_5    = ((u8)0x20),  /*!< Pin 5 selected */
  GPIO_PIN_6    = ((u8)0x40),  /*!< Pin 6 selected */
  GPIO_PIN_7    = ((u8)0x80),  /*!< Pin 7 selected */
  GPIO_PIN_LNIB = ((u8)0x0F),  /*!< Low nibble pins selected */
  GPIO_PIN_HNIB = ((u8)0xF0),  /*!< High nibble pins selected */
  GPIO_PIN_ALL  = ((u8)0xFF)   /*!< All pins selected */
}GPIO_Pin_TypeDef;

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/

/** @addtogroup GPIO_Private_Macros
  * @{
  */

/**
  * @brief Macro used by the assert function to check the different functions parameters.
  */

/**
  * @brief Macro used by the assert function in order to check the different
  * values of GPIOMode_TypeDef.
  */
#define IS_GPIO_MODE_OK(MODE) \
  (((MODE) == GPIO_MODE_IN_FL_NO_IT)    || \
   ((MODE) == GPIO_MODE_IN_PU_NO_IT)    || \
   ((MODE) == GPIO_MODE_IN_FL_IT)       || \
   ((MODE) == GPIO_MODE_IN_PU_IT)       || \
   ((MODE) == GPIO_MODE_OUT_OD_LOW_FAST)  || \
   ((MODE) == GPIO_MODE_OUT_PP_LOW_FAST)  || \
   ((MODE) == GPIO_MODE_OUT_OD_LOW_SLOW)  || \
   ((MODE) == GPIO_MODE_OUT_PP_LOW_SLOW)  || \
   ((MODE) == GPIO_MODE_OUT_OD_HIZ_FAST)  || \
   ((MODE) == GPIO_MODE_OUT_PP_HIGH_FAST)  || \
   ((MODE) == GPIO_MODE_OUT_OD_HIZ_SLOW)  || \
   ((MODE) == GPIO_MODE_OUT_PP_HIGH_SLOW))

/**
  * @brief Macro used by the assert function in order to check the different
  * values of GPIO_Pins.
  */
#define IS_GPIO_PIN_OK(PIN)  ((PIN) != (u8)0x00)

/**
  * @}
  */

/* Exported functions ------------------------------------------------------- */
/** @addtogroup GPIO_Exported_Functions
  * @{
  */

void GPIO_DeInit(GPIO_TypeDef* GPIOx);
void GPIO_Init(GPIO_TypeDef* GPIOx, GPIO_Pin_TypeDef GPIO_Pin, GPIO_Mode_TypeDef GPIO_Mode);
void GPIO_Write(GPIO_TypeDef* GPIOx, u8 PortVal);
void GPIO_WriteHigh(GPIO_TypeDef* GPIOx, GPIO_Pin_TypeDef PortPins);
void GPIO_WriteLow(GPIO_TypeDef* GPIOx, GPIO_Pin_TypeDef PortPins);
void GPIO_WriteReverse(GPIO_TypeDef* GPIOx, GPIO_Pin_TypeDef PortPins);
u8 GPIO_ReadInputData(GPIO_TypeDef* GPIOx);
u8 GPIO_ReadOutputData(GPIO_TypeDef* GPIOx);
BitStatus GPIO_ReadInputPin(GPIO_TypeDef* GPIOx, GPIO_Pin_TypeDef GPIO_Pin);
void GPIO_ExternalPullUpConfig(GPIO_TypeDef* GPIOx, GPIO_Pin_TypeDef GPIO_Pin, FunctionalState NewState);
/**
  * @}
  */

#endif /* __STM8L_GPIO_H */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
