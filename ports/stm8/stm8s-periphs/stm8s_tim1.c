/**
  ******************************************************************************
  * @file stm8s_tim1.c
  * @brief This file contains all the functions for the TIM1 peripheral.
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

/* Includes ------------------------------------------------------------------*/
#include "stm8s_tim1.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/**
  * @addtogroup TIM1_Public_Functions
  * @{
  */

/**
  * @brief Deinitializes the TIM1 peripheral registers to their default reset values.
  * @par Parameters:
  * None
  * @retval None
  */
void TIM1_DeInit(void)
{
    TIM1->CR1  = TIM1_CR1_RESET_VALUE;
    TIM1->CR2  = TIM1_CR2_RESET_VALUE;
    TIM1->SMCR = TIM1_SMCR_RESET_VALUE;
    TIM1->ETR  = TIM1_ETR_RESET_VALUE;
    TIM1->IER  = TIM1_IER_RESET_VALUE;
    TIM1->SR2  = TIM1_SR2_RESET_VALUE;
    /* Disable channels */
    TIM1->CCER1 = TIM1_CCER1_RESET_VALUE;
    TIM1->CCER2 = TIM1_CCER2_RESET_VALUE;
    /* Configure channels as inputs: it is necessary if lock level is equal to 2 or 3 */
    TIM1->CCMR1 = 0x01;
    TIM1->CCMR2 = 0x01;
    TIM1->CCMR3 = 0x01;
    TIM1->CCMR4 = 0x01;
    /* Then reset channel registers: it also works if lock level is equal to 2 or 3 */
    TIM1->CCER1 = TIM1_CCER1_RESET_VALUE;
    TIM1->CCER2 = TIM1_CCER2_RESET_VALUE;
    TIM1->CCMR1 = TIM1_CCMR1_RESET_VALUE;
    TIM1->CCMR2 = TIM1_CCMR2_RESET_VALUE;
    TIM1->CCMR3 = TIM1_CCMR3_RESET_VALUE;
    TIM1->CCMR4 = TIM1_CCMR4_RESET_VALUE;
    TIM1->CNTRH = TIM1_CNTRH_RESET_VALUE;
    TIM1->CNTRL = TIM1_CNTRL_RESET_VALUE;
    TIM1->PSCRH = TIM1_PSCRH_RESET_VALUE;
    TIM1->PSCRL = TIM1_PSCRL_RESET_VALUE;
    TIM1->ARRH  = TIM1_ARRH_RESET_VALUE;
    TIM1->ARRL  = TIM1_ARRL_RESET_VALUE;
    TIM1->CCR1H = TIM1_CCR1H_RESET_VALUE;
    TIM1->CCR1L = TIM1_CCR1L_RESET_VALUE;
    TIM1->CCR2H = TIM1_CCR2H_RESET_VALUE;
    TIM1->CCR2L = TIM1_CCR2L_RESET_VALUE;
    TIM1->CCR3H = TIM1_CCR3H_RESET_VALUE;
    TIM1->CCR3L = TIM1_CCR3L_RESET_VALUE;
    TIM1->CCR4H = TIM1_CCR4H_RESET_VALUE;
    TIM1->CCR4L = TIM1_CCR4L_RESET_VALUE;
    TIM1->OISR  = TIM1_OISR_RESET_VALUE;
    TIM1->EGR   = 0x01; /* TIM1_EGR_UG */
    TIM1->DTR   = TIM1_DTR_RESET_VALUE;
    TIM1->BKR   = TIM1_BKR_RESET_VALUE;
    TIM1->RCR   = TIM1_RCR_RESET_VALUE;
    TIM1->SR1   = TIM1_SR1_RESET_VALUE;
}

/**
  * @brief Initializes the TIM1 Time Base Unit according to the specified parameters.
  * @param[in]  TIM1_Prescaler specifies the Prescaler value.
  * @param[in]  TIM1_CounterMode specifies the counter mode  from @ref TIM1_CounterMode_TypeDef .
  * @param[in]  TIM1_Period specifies the Period value.
  * @param[in]  TIM1_RepetitionCounter specifies the Repetition counter value
  * @retval None
  */
void TIM1_TimeBaseInit(u16 TIM1_Prescaler,
                       TIM1_CounterMode_TypeDef TIM1_CounterMode,
                       u16 TIM1_Period,
                       u8 TIM1_RepetitionCounter)
{

    /* Check parameters */
    assert_param(IS_TIM1_COUNTER_MODE_OK(TIM1_CounterMode));

    /* Set the Autoreload value */
    TIM1->ARRH = (u8)(TIM1_Period >> 8);
    TIM1->ARRL = (u8)(TIM1_Period);

    /* Set the Prescaler value */
    TIM1->PSCRH = (u8)(TIM1_Prescaler >> 8);
    TIM1->PSCRL = (u8)(TIM1_Prescaler);

    /* Select the Counter Mode */
    TIM1->CR1 = (u8)(((TIM1->CR1) & (u8)(~(TIM1_CR1_CMS | TIM1_CR1_DIR))) | (u8)(TIM1_CounterMode));

    /* Set the Repetition Counter value */
    TIM1->RCR = TIM1_RepetitionCounter;

}


/**
  * @brief Enables or disables the TIM1 peripheral.
  * @param[in] NewState new state of the TIM1 peripheral.
	* This parameter can be ENABLE or DISABLE.
  * @retval None
  */
void TIM1_Cmd(FunctionalState NewState)
{
    /* Check the parameters */
    assert_param(IS_FUNCTIONALSTATE_OK(NewState));

    /* set or Reset the CEN Bit */
    if (NewState != DISABLE)
    {
        TIM1->CR1 |= TIM1_CR1_CEN;
    }
    else
    {
        TIM1->CR1 &= (u8)(~TIM1_CR1_CEN);
    }
}


/**
  * @brief Enables or disables the specified TIM1 interrupts.
  * @param[in] NewState new state of the TIM1 peripheral.
  * This parameter can be: ENABLE or DISABLE.
  * @param[in] TIM1_IT specifies the TIM1 interrupts sources to be enabled or disabled.
  * This parameter can be any combination of the following values:
  *  - TIM1_IT_UPDATE: TIM1 update Interrupt source
  *  - TIM1_IT_CC1: TIM1 Capture Compare 1 Interrupt source
  *  - TIM1_IT_CC2: TIM1 Capture Compare 2 Interrupt source
  *  - TIM1_IT_CC3: TIM1 Capture Compare 3 Interrupt source
  *  - TIM1_IT_CC4: TIM1 Capture Compare 4 Interrupt source
  *  - TIM1_IT_CCUpdate: TIM1 Capture Compare Update Interrupt source
  *  - TIM1_IT_TRIGGER: TIM1 Trigger Interrupt source
  *  - TIM1_IT_BREAK: TIM1 Break Interrupt source
  * @param[in] NewState new state of the TIM1 peripheral.
  * @retval None
  */
void TIM1_ITConfig(TIM1_IT_TypeDef  TIM1_IT, FunctionalState NewState)
{
    /* Check the parameters */
    assert_param(IS_TIM1_IT_OK(TIM1_IT));
    assert_param(IS_FUNCTIONALSTATE_OK(NewState));

    if (NewState != DISABLE)
    {
        /* Enable the Interrupt sources */
        TIM1->IER |= (u8)TIM1_IT;
    }
    else
    {
        /* Disable the Interrupt sources */
        TIM1->IER &= (u8)(~(u8)TIM1_IT);
    }
}


/**
  * @}
  */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
