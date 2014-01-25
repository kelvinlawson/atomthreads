/**
  ******************************************************************************
  * @file    stm8s_beep.c
  * @author  MCD Application Team
  * @version V2.1.0
  * @date    18-November-2011
  * @brief   This file contains all the functions for the BEEP peripheral.
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
#include "stm8s_beep.h"

/** @addtogroup STM8S_StdPeriph_Driver
  * @{
  */
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* Public functions ----------------------------------------------------------*/

/**
  * @addtogroup BEEP_Public_Functions
  * @{
  */

/**
  * @brief  Deinitializes the BEEP peripheral registers to their default reset
  * values.
  * @param  None
  * @retval None
  */
void BEEP_DeInit(void)
{
    BEEP->CSR = BEEP_CSR_RESET_VALUE;
}

/**
  * @brief  Initializes the BEEP function according to the specified parameters.
  * @param   BEEP_Frequency Frequency selection.
  * can be one of the values of @ref BEEP_Frequency_TypeDef.
  * @retval None
  * @par Required preconditions:
  * The LS RC calibration must be performed before calling this function.
  */
void BEEP_Init(BEEP_Frequency_TypeDef BEEP_Frequency)
{

    /* Check parameter */
    assert_param(IS_BEEP_FREQUENCY_OK(BEEP_Frequency));

    /* Set a default calibration value if no calibration is done */
    if ((BEEP->CSR & BEEP_CSR_BEEPDIV) == BEEP_CSR_BEEPDIV)
    {
        BEEP->CSR &= (uint8_t)(~BEEP_CSR_BEEPDIV); /* Clear bits */
        BEEP->CSR |= BEEP_CALIBRATION_DEFAULT;
    }

    /* Select the output frequency */
    BEEP->CSR &= (uint8_t)(~BEEP_CSR_BEEPSEL);
    BEEP->CSR |= (uint8_t)(BEEP_Frequency);

}

/**
  * @brief  Enable or disable the BEEP function.
  * @param   NewState Indicates the new state of the BEEP function.
  * @retval None
  * @par Required preconditions:
  * Initialisation of BEEP and LS RC calibration must be done before.
  */
void BEEP_Cmd(FunctionalState NewState)
{
    if (NewState != DISABLE)
    {
        /* Enable the BEEP peripheral */
        BEEP->CSR |= BEEP_CSR_BEEPEN;
    }
    else
    {
        /* Disable the BEEP peripheral */
        BEEP->CSR &= (uint8_t)(~BEEP_CSR_BEEPEN);
    }
}

/**
  * @brief  Update CSR register with the measured LSI frequency.
  * @par Note on the APR calculation:
  * A is the integer part of LSIFreqkHz/4 and x the decimal part.
  * x <= A/(1+2A) is equivalent to A >= x(1+2A) and also to 4A >= 4x(1+2A) [F1]
  * but we know that A + x = LSIFreqkHz/4 ==> 4x = LSIFreqkHz-4A
  * so [F1] can be written :
  * 4A >= (LSIFreqkHz-4A)(1+2A)
  * @param   LSIFreqHz Low Speed RC frequency measured by timer (in Hz).
  * @retval None
  * @par Required preconditions:
  * - BEEP must be disabled to avoid unwanted interrupts.
  */
void BEEP_LSICalibrationConfig(uint32_t LSIFreqHz)
{

    uint16_t lsifreqkhz;
    uint16_t A;

    /* Check parameter */
    assert_param(IS_LSI_FREQUENCY_OK(LSIFreqHz));

    lsifreqkhz = (uint16_t)(LSIFreqHz / 1000); /* Converts value in kHz */

    /* Calculation of BEEPER calibration value */

    BEEP->CSR &= (uint8_t)(~BEEP_CSR_BEEPDIV); /* Clear bits */

    A = (uint16_t)(lsifreqkhz >> 3U); /* Division by 8, keep integer part only */

    if ((8U * A) >= ((lsifreqkhz - (8U * A)) * (1U + (2U * A))))
    {
        BEEP->CSR |= (uint8_t)(A - 2U);
    }
    else
    {
        BEEP->CSR |= (uint8_t)(A - 1U);
    }
}

/**
  * @}
  */
  
/**
  * @}
  */
  
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
