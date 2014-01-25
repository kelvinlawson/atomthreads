/**
  ********************************************************************************
  * @file    stm8s_wwdg.c
  * @author  MCD Application Team
  * @version V2.1.0
  * @date    18-November-2011
  * @brief   This file contains all the functions for the WWDG peripheral.
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
#include "stm8s_wwdg.h"

/** @addtogroup STM8S_StdPeriph_Driver
  * @{
  */
/* Private define ------------------------------------------------------------*/
#define BIT_MASK          ((uint8_t)0x7F)
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @addtogroup WWDG_Public_Functions
  * @{
  */

/**
  * @brief  Initializes the WWDG peripheral.
  *         This function set Window Register = WindowValue, Counter Register
  *         according to Counter and \b ENABLE \b WWDG
  * @param  Counter : WWDG counter value
  * @param  WindowValue : specifies the WWDG Window Register, range is 0x00 to 0x7F.
  * @retval None
  */
void WWDG_Init(uint8_t Counter, uint8_t WindowValue)
{
    /* Check the parameters */
    assert_param(IS_WWDG_WINDOWLIMITVALUE_OK(WindowValue));
    
    WWDG->WR = WWDG_WR_RESET_VALUE;
    WWDG->CR = (uint8_t)((uint8_t)(WWDG_CR_WDGA | WWDG_CR_T6) | (uint8_t)Counter);
    WWDG->WR = (uint8_t)((uint8_t)(~WWDG_CR_WDGA) & (uint8_t)(WWDG_CR_T6 | WindowValue));
}

/**
  * @brief  Refreshes the WWDG peripheral.
  * @param  Counter :  WWDG Counter Value
  *         This parameter must be a number between 0x40 and 0x7F.
  * @retval None
  */
void WWDG_SetCounter(uint8_t Counter)
{

    /* Check the parameters */
    assert_param(IS_WWDG_COUNTERVALUE_OK(Counter));

   /* Write to T[6:0] bits to configure the counter value, no need to do
     a read-modify-write; writing a 0 to WDGA bit does nothing */
  WWDG->CR = (uint8_t)(Counter & (uint8_t)BIT_MASK);

}

/**
  * @brief Gets the WWDG Counter Value.
  *        This value could be used to check if WWDG is in the window, where
  *        refresh is allowed.
  * @param  None
  * @retval WWDG Counter Value
  */
uint8_t WWDG_GetCounter(void)
{
    return(WWDG->CR);
}

/**
  * @brief  Generates immediate WWDG RESET.
  * @param  None
  * @retval None
  */
void WWDG_SWReset(void)
{
    WWDG->CR = WWDG_CR_WDGA; /* Activate WWDG, with clearing T6 */
}

/**
  * @brief  Sets the WWDG window value.
  * @param  WindowValue: specifies the window value to be compared to the
  *         downcounter.
  *         This parameter value must be lower than 0x80.
  * @retval None
  */
void WWDG_SetWindowValue(uint8_t WindowValue)
{
    /* Check the parameters */
    assert_param(IS_WWDG_WINDOWLIMITVALUE_OK(WindowValue));
    
    WWDG->WR = (uint8_t)((uint8_t)(~WWDG_CR_WDGA) & (uint8_t)(WWDG_CR_T6 | WindowValue));
}
/**
  * @}
  */
  
/**
  * @}
  */
  
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
