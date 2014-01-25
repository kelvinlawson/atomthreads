/**
  ********************************************************************************
  * @file    stm8s_wwdg.h
  * @author  MCD Application Team
  * @version V2.1.0
  * @date    18-November-2011
  * @brief   This file contains all functions prototype and macros for the WWDG peripheral.
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
#ifndef __STM8S_WWDG_H
#define __STM8S_WWDG_H

/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"

/** @addtogroup STM8S_StdPeriph_Driver
  * @{
  */

/* Private macros ------------------------------------------------------------*/

/** @addtogroup WWDG_Private_Macros
  * @{
  */

/**
  * @brief  Macro used by the assert function in order to check the
  * values of the window register.
  */
#define IS_WWDG_WINDOWLIMITVALUE_OK(WindowLimitValue) ((WindowLimitValue) <= 0x7F)

/**
  * @brief  Macro used by the assert function in order to check the different
  * values of the counter register.
  */
#define IS_WWDG_COUNTERVALUE_OK(CounterValue) ((CounterValue) <= 0x7F)

/**
  * @}
  */

/* Exported types ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */

/** @addtogroup WWDG_Exported_Functions
  * @{
  */

void WWDG_Init(uint8_t Counter, uint8_t WindowValue);
void WWDG_SetCounter(uint8_t Counter);
uint8_t WWDG_GetCounter(void);
void WWDG_SWReset(void);
void WWDG_SetWindowValue(uint8_t WindowValue);


/**
  * @}
  */

#endif /* __STM8S_WWDG_H */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
