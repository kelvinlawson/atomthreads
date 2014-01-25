/**
  ********************************************************************************
  * @file    stm8s_iwdg.c
  * @author  MCD Application Team
  * @version V2.1.0
  * @date    18-November-2011
  * @brief   This file contains all the functions for the IWDG peripheral.
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

/* Includes ------------------------------------------------------------------*/
#include "stm8s_iwdg.h"

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Public functions ----------------------------------------------------------*/

/** @addtogroup IWDG_Public_Functions
  * @{
  */

/**
  * @brief  Enables or disables write access to Prescaler and Reload registers.
  * @param  IWDG_WriteAccess : New state of write access to Prescaler and Reload
  *         registers.  This parameter can be a value of @ref IWDG_WriteAccess_TypeDef.
  * @retval None
  */
void IWDG_WriteAccessCmd(IWDG_WriteAccess_TypeDef IWDG_WriteAccess)
{
    /* Check the parameters */
    assert_param(IS_IWDG_WRITEACCESS_MODE_OK(IWDG_WriteAccess));

    IWDG->KR = (uint8_t)IWDG_WriteAccess; /* Write Access */
}

/**
  * @brief  Sets IWDG Prescaler value.
  * @note   Write access should be enabled
  * @param  IWDG_Prescaler : Specifies the IWDG Prescaler value.
  *         This parameter can be a value of @ref IWDG_Prescaler_TypeDef.
  * @retval None
  */
void IWDG_SetPrescaler(IWDG_Prescaler_TypeDef IWDG_Prescaler)
{
    /* Check the parameters */
    assert_param(IS_IWDG_PRESCALER_OK(IWDG_Prescaler));

    IWDG->PR = (uint8_t)IWDG_Prescaler;
}

/**
  * @brief  Sets IWDG Reload value.
  * @note   Write access should be enabled
  * @param  IWDG_Reload : Reload register value.
  *         This parameter must be a number between 0 and 0xFF.
  * @retval None
  */
void IWDG_SetReload(uint8_t IWDG_Reload)
{
    IWDG->RLR = IWDG_Reload;
}

/**
  * @brief  Reloads IWDG counter
  * @note   Write access should be enabled
  * @param  None
  * @retval None
  */
void IWDG_ReloadCounter(void)
{
    IWDG->KR = IWDG_KEY_REFRESH;
}

/**
  * @brief  Enables IWDG.
  * @param  None
  * @retval None
  */
void IWDG_Enable(void)
{
    IWDG->KR = IWDG_KEY_ENABLE;
}

/**
  * @}
  */
  
/**
  * @}
  */
  
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
