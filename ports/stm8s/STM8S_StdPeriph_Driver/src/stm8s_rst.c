/**
  ******************************************************************************
  * @file    stm8s_rst.c
  * @author  MCD Application Team
  * @version V2.1.0
  * @date    18-November-2011
  * @brief   This file contains all the functions for the RST peripheral.
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

#include "stm8s_rst.h"

/** @addtogroup STM8S_StdPeriph_Driver
  * @{
  */
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private Constants ---------------------------------------------------------*/
/* Public functions ----------------------------------------------------------*/
/**
  * @addtogroup RST_Public_Functions
  * @{
  */


/**
  * @brief   Checks whether the specified RST flag is set or not.
  * @param   RST_Flag : specify the reset flag to check.
  *          This parameter can be a value of @ref RST_FLAG_TypeDef.
  * @retval  FlagStatus: status of the given RST flag.
  */
FlagStatus RST_GetFlagStatus(RST_Flag_TypeDef RST_Flag)
{
    /* Check the parameters */
    assert_param(IS_RST_FLAG_OK(RST_Flag));

    /* Get flag status */

    return ((FlagStatus)((uint8_t)RST->SR & (uint8_t)RST_Flag));
}

/**
  * @brief  Clears the specified RST flag.
  * @param  RST_Flag : specify the reset flag to clear.
  *         This parameter can be a value of @ref RST_FLAG_TypeDef.
  * @retval None
  */
void RST_ClearFlag(RST_Flag_TypeDef RST_Flag)
{
    /* Check the parameters */
    assert_param(IS_RST_FLAG_OK(RST_Flag));

    RST->SR = (uint8_t)RST_Flag;
}

/**
  * @}
  */
  
/**
  * @}
  */
  
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
