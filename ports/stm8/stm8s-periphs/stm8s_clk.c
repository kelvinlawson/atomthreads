/**
  ******************************************************************************
  * @file stm8s_clk.c
  * @brief This file contains all the functions for the CLK peripheral.
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

#include "stm8s_clk.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Private Constants ---------------------------------------------------------*/

/**
  * @addtogroup CLK_Private_Constants
  * @{
  */

uc8 HSIDivFactor[4] = {1, 2, 4, 8}; /*!< Holds the different HSI Dividor factors */
uc8 CLKPrescTable[8] = {1, 2, 4, 8, 10, 16, 20, 40}; /*!< Holds the different CLK prescaler values */

/**
  * @}
  */

/* Public functions ----------------------------------------------------------*/
/**
  * @addtogroup CLK_Public_Functions
  * @{
  */

/**
  * @brief Deinitializes the CLK peripheral registers to their default reset
  * values.
  * @par Parameters:
  * None
  * @retval None
  * @par Warning:
  * Resetting the CCOR register: \n
  * When the CCOEN bit is set, the reset of the CCOR register require
  * two consecutive write instructions in order to reset first the CCOEN bit
  * and the second one is to reset the CCOSEL bits.
  */
void CLK_DeInit(void)
{

    CLK->ICKR = CLK_ICKR_RESET_VALUE;
    CLK->ECKR = CLK_ECKR_RESET_VALUE;
    CLK->SWR  = CLK_SWR_RESET_VALUE;
    CLK->SWCR = CLK_SWCR_RESET_VALUE;
    CLK->CKDIVR = CLK_CKDIVR_RESET_VALUE;
    CLK->PCKENR1 = CLK_PCKENR1_RESET_VALUE;
    CLK->PCKENR2 = CLK_PCKENR2_RESET_VALUE;
    CLK->CSSR = CLK_CSSR_RESET_VALUE;
    CLK->CCOR = CLK_CCOR_RESET_VALUE;
    while (CLK->CCOR & CLK_CCOR_CCOEN)
    {}
    CLK->CCOR = CLK_CCOR_RESET_VALUE;
    CLK->CANCCR = CLK_CANCCR_RESET_VALUE;
    CLK->HSITRIMR = CLK_HSITRIMR_RESET_VALUE;
    CLK->SWIMCCR = CLK_SWIMCCR_RESET_VALUE;

}

/**
  * @brief  Configures the High Speed Internal oscillator (HSI).
  * @par Full description:
  * If CLK_FastHaltWakeup is enabled, HSI oscillator is automatically
  * switched-on (HSIEN=1) and selected as next clock master
  * (CKM=SWI=HSI) when resuming from HALT/ActiveHalt modes.\n
  * @param[in] NewState this parameter is the Wake-up Mode state.
  * @retval None
  */
void CLK_FastHaltWakeUpCmd(FunctionalState NewState)
{

    /* check the parameters */
    assert_param(IS_FUNCTIONALSTATE_OK(NewState));

    if (NewState != DISABLE)
    {
        /* Set FHWU bit (HSI oscillator is automatically switched-on) */
        CLK->ICKR |= CLK_ICKR_FHWU;
    }
    else  /* FastHaltWakeup = DISABLE */
    {
        /* Reset FHWU bit */
        CLK->ICKR &= (u8)(~CLK_ICKR_FHWU);
    }

}

/**
  * @brief Enable or Disable the External High Speed oscillator (HSE).
  * @param[in] NewState new state of HSEEN, value accepted ENABLE, DISABLE.
  * @retval None
  */
void CLK_HSECmd(FunctionalState NewState)
{

    /* Check the parameters */
    assert_param(IS_FUNCTIONALSTATE_OK(NewState));

    if (NewState != DISABLE)
    {
        /* Set HSEEN bit */
        CLK->ECKR |= CLK_ECKR_HSEEN;
    }
    else
    {
        /* Reset HSEEN bit */
        CLK->ECKR &= (u8)(~CLK_ECKR_HSEEN);
    }

}

/**
  * @brief Enables or disables the Internal High Speed oscillator (HSI).
  * @param[in] NewState new state of HSIEN, value accepted ENABLE, DISABLE.
  * @retval None
  */
void CLK_HSICmd(FunctionalState NewState)
{

    /* Check the parameters */
    assert_param(IS_FUNCTIONALSTATE_OK(NewState));

    if (NewState != DISABLE)
    {
        /* Set HSIEN bit */
        CLK->ICKR |= CLK_ICKR_HSIEN;
    }
    else
    {
        /* Reset HSIEN bit */
        CLK->ICKR &= (u8)(~CLK_ICKR_HSIEN);
    }

}

/**
  * @brief Enables or disables the Internal Low Speed oscillator (LSI).
  * @param[in]  NewState new state of LSIEN, value accepted ENABLE, DISABLE.
  * @retval None
  */
void CLK_LSICmd(FunctionalState NewState)
{

    /* Check the parameters */
    assert_param(IS_FUNCTIONALSTATE_OK(NewState));

    if (NewState != DISABLE)
    {
        /* Set LSIEN bit */
        CLK->ICKR |= CLK_ICKR_LSIEN;
    }
    else
    {
        /* Reset LSIEN bit */
        CLK->ICKR &= (u8)(~CLK_ICKR_LSIEN);
    }

}

/**
  * @brief Enables or disablle the Configurable Clock Output (CCO).
  * @param[in] NewState : New state of CCEN bit (CCO register).
  * This parameter can be any of the @ref FunctionalState enumeration.
  * @retval None
  */
void CLK_CCOCmd(FunctionalState NewState)
{

    /* Check the parameters */
    assert_param(IS_FUNCTIONALSTATE_OK(NewState));

    if (NewState != DISABLE)
    {
        /* Set CCOEN bit */
        CLK->CCOR |= CLK_CCOR_CCOEN;
    }
    else
    {
        /* Reset CCOEN bit */
        CLK->CCOR &= (u8)(~CLK_CCOR_CCOEN);
    }

}

/**
  * @brief Starts or Stops manually the clock switch execution.
  * @par Full description:
  * NewState parameter set the SWEN.
  * @param[in] NewState new state of SWEN, value accepted ENABLE, DISABLE.
  * @retval None
  */
void CLK_ClockSwitchCmd(FunctionalState NewState)
{

    /* Check the parameters */
    assert_param(IS_FUNCTIONALSTATE_OK(NewState));

    if (NewState != DISABLE )
    {
        /* Enable the Clock Switch */
        CLK->SWCR |= CLK_SWCR_SWEN;
    }
    else
    {
        /* Disable the Clock Switch */
        CLK->SWCR &= (u8)(~CLK_SWCR_SWEN);
    }

}

/**
  * @brief Configures the slow active halt wake up
  * @param[in] NewState: specifies the Slow Active Halt wake up state.
  * can be set of the following values:
  * - DISABLE: Slow Active Halt mode disabled;
  * - ENABLE:  Slow Active Halt mode enabled.
  * @retval None
  */
void CLK_SlowActiveHaltWakeUpCmd(FunctionalState NewState)
{

    /* check the parameters */
    assert_param(IS_FUNCTIONALSTATE_OK(NewState));

    if (NewState != DISABLE)
    {
        /* Set S_ACTHALT bit */
        CLK->ICKR |= CLK_ICKR_SWUAH;
    }
    else
    {
        /* Reset S_ACTHALT bit */
        CLK->ICKR &= (u8)(~CLK_ICKR_SWUAH);
    }

}

/**
  * @brief  Enables or disables the specified peripheral CLK.
  * @param[in] CLK_Peripheral : This parameter specifies the peripheral clock to gate.
  * This parameter can be any of the  @ref CLK_Peripheral_TypeDef enumeration.
  * @param[in] NewState : New state of specified peripheral clock.
  * This parameter can be any of the @ref FunctionalState enumeration.
  * @retval None
  */
void CLK_PeripheralClockConfig(CLK_Peripheral_TypeDef CLK_Peripheral, FunctionalState NewState)
{

    /* Check the parameters */
    assert_param(IS_FUNCTIONALSTATE_OK(NewState));
    assert_param(IS_CLK_PERIPHERAL_OK(CLK_Peripheral));

    if (((u8)CLK_Peripheral & (u8)0x10) == 0x00)
    {
        if (NewState != DISABLE)
        {
            /* Enable the peripheral Clock */
            CLK->PCKENR1 |= (u8)((u8)1 << ((u8)CLK_Peripheral & (u8)0x0F));
        }
        else
        {
            /* Disable the peripheral Clock */
            CLK->PCKENR1 &= (u8)(~(u8)(((u8)1 << ((u8)CLK_Peripheral & (u8)0x0F))));
        }
    }
    else
    {
        if (NewState != DISABLE)
        {
            /* Enable the peripheral Clock */
            CLK->PCKENR2 |= (u8)((u8)1 << ((u8)CLK_Peripheral & (u8)0x0F));
        }
        else
        {
            /* Disable the peripheral Clock */
            CLK->PCKENR2 &= (u8)(~(u8)(((u8)1 << ((u8)CLK_Peripheral & (u8)0x0F))));
        }
    }

}

/**
  * @brief configures the Switch from one clock to another
  * @param[in] CLK_SwitchMode select the clock switch mode.
  * It can be set of the values of @ref CLK_SwitchMode_TypeDef
  * @param[in] CLK_NewClock choice of the future clock.
  * It can be set of the values of @ref CLK_Source_TypeDef
  * @param[in] NewState Enable or Disable the Clock Switch interrupt.
  * @param[in] CLK_CurrentClockState current clock to switch OFF or to keep ON.
  * It can be set of the values of @ref CLK_CurrentClockState_TypeDef
  * @retval ErrorStatus this shows the clock switch status (ERROR/SUCCESS).
  */
ErrorStatus CLK_ClockSwitchConfig(CLK_SwitchMode_TypeDef CLK_SwitchMode, CLK_Source_TypeDef CLK_NewClock, FunctionalState ITState, CLK_CurrentClockState_TypeDef CLK_CurrentClockState)
{

    CLK_Source_TypeDef clock_master;
    u16 DownCounter = CLK_TIMEOUT;
    ErrorStatus Swif = ERROR;

    /* Check the parameters */
    assert_param(IS_CLK_SOURCE_OK(CLK_NewClock));
    assert_param(IS_CLK_SWITCHMODE_OK(CLK_SwitchMode));
    assert_param(IS_FUNCTIONALSTATE_OK(ITState));
    assert_param(IS_CLK_CURRENTCLOCKSTATE_OK(CLK_CurrentClockState));

    /* Current clock master saving */
    clock_master = (CLK_Source_TypeDef)CLK->CMSR;

    /* Automatic switch mode management */
    if (CLK_SwitchMode == CLK_SWITCHMODE_AUTO)
    {

        /* Enables Clock switch */
        CLK->SWCR |= CLK_SWCR_SWEN;

        /* Enables or Disables Switch interrupt */
        if (ITState != DISABLE)
        {
            CLK->SWCR |= CLK_SWCR_SWIEN;
        }
        else
        {
            CLK->SWCR &= (u8)(~CLK_SWCR_SWIEN);
        }

        /* Selection of the target clock source */
        CLK->SWR = (u8)CLK_NewClock;

        while (((CLK->SWCR & CLK_SWCR_SWBSY) && (DownCounter != 0)))
        {
            DownCounter--;
        }

        if (DownCounter != 0)
        {
            Swif = SUCCESS;
        }
        else
        {
            Swif = ERROR;
        }

    }
    else /* CLK_SwitchMode == CLK_SWITCHMODE_MANUAL */
    {

        /* Enables or Disables Switch interrupt  if required  */
        if (ITState != DISABLE)
        {
            CLK->SWCR |= CLK_SWCR_SWIEN;
        }
        else
        {
            CLK->SWCR &= (u8)(~CLK_SWCR_SWIEN);
        }

        /* Selection of the target clock source */
        CLK->SWR = (u8)CLK_NewClock;

        /* In manual mode, there is no risk to be stucked in a loop, value returned
          is then always SUCCESS */
        Swif = SUCCESS;

    }

    /* Switch OFF current clock if required */
    if ((CLK_CurrentClockState == CLK_CURRENTCLOCKSTATE_DISABLE) && ( clock_master == CLK_SOURCE_HSI))
    {
        CLK->ICKR &= (u8)(~CLK_ICKR_HSIEN);
    }
    else if ((CLK_CurrentClockState == CLK_CURRENTCLOCKSTATE_DISABLE) && ( clock_master == CLK_SOURCE_LSI))
    {
        CLK->ICKR &= (u8)(~CLK_ICKR_LSIEN);
    }
    else if ((CLK_CurrentClockState == CLK_CURRENTCLOCKSTATE_DISABLE) && ( clock_master == CLK_SOURCE_HSE))
    {
        CLK->ECKR &= (u8)(~CLK_ECKR_HSEEN);
    }

    return(Swif);

}

/**
  * @brief Configures the HSI clock dividers.
  * @param[in] HSIPrescaler : Specifies the HSI clock divider to apply.
  * This parameter can be any of the @ref CLK_Prescaler_TypeDef enumeration.
  * @retval None
  */
void CLK_HSIPrescalerConfig(CLK_Prescaler_TypeDef HSIPrescaler)
{

    /* check the parameters */
    assert_param(IS_CLK_HSIPRESCALER_OK(HSIPrescaler));

    /* Clear High speed internal clock prescaler */
    CLK->CKDIVR &= (u8)(~CLK_CKDIVR_HSIDIV);

    /* Set High speed internal clock prescaler */
    CLK->CKDIVR |= (u8)HSIPrescaler;

}

/**
  * @brief Output the selected clock on a dedicated I/O pin.
  * @param[in] CLK_CCO : Specifies the clock source.
  * This parameter can be any of the  @ref CLK_Output_TypeDef enumeration.
  * @retval None
  * @par Required preconditions:
  * The dedicated I/O pin must be set at 1 in the corresponding Px_CR1 register \n
  * to be set as input with pull-up or push-pull output.
  */
void CLK_CCOConfig(CLK_Output_TypeDef CLK_CCO)
{

    /* check the parameters */
    assert_param(IS_CLK_OUTPUT_OK(CLK_CCO));

    /* Clears of the CCO type bits part */
    CLK->CCOR &= (u8)(~CLK_CCOR_CCOSEL);

    /* Selects the source provided on cco_ck output */
    CLK->CCOR |= (u8)CLK_CCO;

    /* Enable the clock output */
    CLK->CCOR |= CLK_CCOR_CCOEN;

}

/**
  * @brief  Enables or disables the specified CLK interrupts.
  * @param[in] CLK_IT This parameter specifies the interrupt sources.
  * It can be one of the values of @ref CLK_IT_TypeDef.
  * @param[in] NewState New state of the Interrupt.
  * Value accepted ENABLE, DISABLE.
  * @retval None
  */
void CLK_ITConfig(CLK_IT_TypeDef CLK_IT, FunctionalState NewState)
{

    /* check the parameters */
    assert_param(IS_FUNCTIONALSTATE_OK(NewState));
    assert_param(IS_CLK_IT_OK(CLK_IT));

    if (NewState != DISABLE)
    {
        switch (CLK_IT)
        {
        case CLK_IT_SWIF: /* Enable the clock switch interrupt */
            CLK->SWCR |= CLK_SWCR_SWIEN;
            break;
        case CLK_IT_CSSD: /* Enable the clock security system detection interrupt */
            CLK->CSSR |= CLK_CSSR_CSSDIE;
            break;
        default:
            break;
        }
    }
    else  /*(NewState == DISABLE)*/
    {
        switch (CLK_IT)
        {
        case CLK_IT_SWIF: /* Disable the clock switch interrupt */
            CLK->SWCR  &= (u8)(~CLK_SWCR_SWIEN);
            break;
        case CLK_IT_CSSD: /* Disable the clock security system detection interrupt */
            CLK->CSSR &= (u8)(~CLK_CSSR_CSSDIE);
            break;
        default:
            break;
        }
    }

}

/**
  * @brief Configures the HSI and CPU clock dividers.
  * @param[in] ClockPrescaler Specifies the HSI or CPU clock divider to apply.
  * @retval None
  */
void CLK_SYSCLKConfig(CLK_Prescaler_TypeDef ClockPrescaler)
{

    /* check the parameters */
    assert_param(IS_CLK_PRESCALER_OK(ClockPrescaler));

    if (((u8)ClockPrescaler & (u8)0x80) == 0x00) /* Bit7 = 0 means HSI divider */
    {
        CLK->CKDIVR &= (u8)(~CLK_CKDIVR_HSIDIV);
        CLK->CKDIVR |= (u8)((u8)ClockPrescaler & (u8)CLK_CKDIVR_HSIDIV);
    }
    else /* Bit7 = 1 means CPU divider */
    {
        CLK->CKDIVR &= (u8)(~CLK_CKDIVR_CPUDIV);
        CLK->CKDIVR |= (u8)((u8)ClockPrescaler & (u8)CLK_CKDIVR_CPUDIV);
    }

}
/**
  * @brief Configures the SWIM clock frequency on the fly.
  * @param[in] CLK_SWIMDivider Specifies the SWIM clock divider to apply.
  * can be one of the value of @ref CLK_SWIMDivider_TypeDef
  * @retval None
  */
void CLK_SWIMConfig(CLK_SWIMDivider_TypeDef CLK_SWIMDivider)
{

    /* check the parameters */
    assert_param(IS_CLK_SWIMDIVIDER_OK(CLK_SWIMDivider));

    if (CLK_SWIMDivider != CLK_SWIMDIVIDER_2)
    {
        /* SWIM clock is not divided by 2 */
        CLK->SWIMCCR |= CLK_SWIMCCR_SWIMDIV;
    }
    else /* CLK_SWIMDivider == CLK_SWIMDIVIDER_2 */
    {
        /* SWIM clock is divided by 2 */
        CLK->SWIMCCR &= (u8)(~CLK_SWIMCCR_SWIMDIV);
    }

}

/**
  * @brief Configure the divider for the external CAN clock.
  * @param[in] CLK_CANDivider Specifies the CAN clock divider to apply.
  * can be one of the value of @ref CLK_CANDivider_TypeDef
  * @retval None
  */
void CLK_CANConfig(CLK_CANDivider_TypeDef CLK_CANDivider)
{

    /* check the parameters */
    assert_param(IS_CLK_CANDIVIDER_OK(CLK_CANDivider));

    /* Clear the CANDIV bits */
    CLK->CANCCR &= (u8)(~CLK_CANCCR_CANDIV);

    /* Select divider */
    CLK->CANCCR |= (u8)CLK_CANDivider;

}

/**
  * @brief Enables the Clock Security System.
  * @par Full description:
  * once CSS is enabled it cannot be disabled until the next reset.
  * @par Parameters:
  * None
  * @retval None
  */
void CLK_ClockSecuritySystemEnable(void)
{
    /* Set CSSEN bit */
    CLK->CSSR |= CLK_CSSR_CSSEN;
}

/**
  * @brief Returns the clock source used as system clock.
  * @par Parameters:
  * None
  * @retval  Clock source used.
  * can be one of the values of @ref CLK_Source_TypeDef
  */
CLK_Source_TypeDef CLK_GetSYSCLKSource(void)
{
    return((CLK_Source_TypeDef)CLK->CMSR);
}


/**
  * @brief This function returns the frequencies of different on chip clocks.
  * @par Parameters:
  * None
  * @retval the master clock frequency
  */
u32 CLK_GetClockFreq(void)
{

    u32 clockfrequency = 0;
    CLK_Source_TypeDef clocksource = CLK_SOURCE_HSI;
    u8 tmp = 0, presc = 0;

    /* Get CLK source. */
    clocksource = (CLK_Source_TypeDef)CLK->CMSR;

    if (clocksource == CLK_SOURCE_HSI)
    {
        tmp = (u8)(CLK->CKDIVR & CLK_CKDIVR_HSIDIV);
        tmp = (u8)(tmp >> 3);
        presc = HSIDivFactor[tmp];
        clockfrequency = HSI_VALUE / presc;
    }
    else if ( clocksource == CLK_SOURCE_LSI)
    {
        clockfrequency = LSI_VALUE;
    }
    else
    {
        clockfrequency = HSE_VALUE;
    }

    return((u32)clockfrequency);

}

/**
  * @brief Adjusts the Internal High Speed oscillator (HSI) calibration value.
  * @par Full description:
  * @param[in] CLK_HSICalibrationValue calibration trimming value.
  * can be one of the values of @ref CLK_HSITrimValue_TypeDef
  * @retval None
  */
void CLK_AdjustHSICalibrationValue(CLK_HSITrimValue_TypeDef CLK_HSICalibrationValue)
{

    /* check the parameters */
    assert_param(IS_CLK_HSITRIMVALUE_OK(CLK_HSICalibrationValue));

    /* Store the new value */
    CLK->HSITRIMR = (u8)((CLK->HSITRIMR & (u8)(~CLK_HSITRIMR_HSITRIM))|((u8)CLK_HSICalibrationValue));

}

/**
  * @brief Reset the SWBSY flag (SWICR Reister)
  * @par Full description:
  * This function reset SWBSY flag in order to reset clock switch operations (target
  * oscillator is broken, stabilization is longing too much, etc.).  If at the same time \n
  * software attempts to set SWEN and clear SWBSY, SWBSY action takes precedence.
  * @par Parameters:
  * None
  * @retval None
  */
void CLK_SYSCLKEmergencyClear(void)
{
    CLK->SWCR &= (u8)(~CLK_SWCR_SWBSY);
}

/**
  * @brief Checks whether the specified CLK flag is set or not.
  * @par Full description:
  * @param[in] CLK_FLAG Flag to check.
  * can be one of the values of @ref CLK_Flag_TypeDef
  * @retval FlagStatus, status of the checked flag
  */
FlagStatus CLK_GetFlagStatus(CLK_Flag_TypeDef CLK_FLAG)
{

    u16 statusreg = 0;
    u8 tmpreg = 0;
    FlagStatus bitstatus = RESET;

    /* check the parameters */
    assert_param(IS_CLK_FLAG_OK(CLK_FLAG));

    /* Get the CLK register index */
    statusreg = (u16)((u16)CLK_FLAG & (u16)0xFF00);


    if (statusreg == 0x0100) /* The flag to check is in ICKRregister */
    {
        tmpreg = CLK->ICKR;
    }
    else if (statusreg == 0x0200) /* The flag to check is in ECKRregister */
    {
        tmpreg = CLK->ECKR;
    }
    else if (statusreg == 0x0300) /* The flag to check is in SWIC register */
    {
        tmpreg = CLK->SWCR;
    }
    else if (statusreg == 0x0400) /* The flag to check is in CSS register */
    {
        tmpreg = CLK->CSSR;
    }
    else /* The flag to check is in CCO register */
    {
        tmpreg = CLK->CCOR;
    }

    if ((tmpreg & (u8)CLK_FLAG) != (u8)RESET)
    {
        bitstatus = SET;
    }
    else
    {
        bitstatus = RESET;
    }

    /* Return the flag status */
    return((FlagStatus)bitstatus);

}

/**
  * @brief Checks whether the specified CLK interrupt has is enabled or not.
  * @param[in] CLK_IT specifies the CLK interrupt.
  * can be one of the values of @ref CLK_IT_TypeDef
  * @retval ITStatus, new state of CLK_IT (SET or RESET).
  */
ITStatus CLK_GetITStatus(CLK_IT_TypeDef CLK_IT)
{

    ITStatus bitstatus = RESET;

    /* check the parameters */
    assert_param(IS_CLK_IT_OK(CLK_IT));

    if (CLK_IT == CLK_IT_SWIF)
    {
        /* Check the status of the clock switch interrupt */
        if ((CLK->SWCR & (u8)CLK_IT) == (u8)0x0C)
        {
            bitstatus = SET;
        }
        else
        {
            bitstatus = RESET;
        }
    }
    else /* CLK_IT == CLK_IT_CSSDIE */
    {
        /* Check the status of the security system detection interrupt */
        if ((CLK->CSSR & (u8)CLK_IT) == (u8)0x0C)
        {
            bitstatus = SET;
        }
        else
        {
            bitstatus = RESET;
        }
    }

    /* Return the CLK_IT status */
    return bitstatus;

}

/**
  * @brief Clears the CLK’s interrupt pending bits.
  * @param[in] CLK_IT specifies the interrupt pending bits.
  * can be one of the values of @ref CLK_IT_TypeDef
  * @retval None
  */
void CLK_ClearITPendingBit(CLK_IT_TypeDef CLK_IT)
{

    /* check the parameters */
    assert_param(IS_CLK_IT_OK(CLK_IT));

    if (CLK_IT == (u8)CLK_IT_CSSD)
    {
        /* Clear the status of the security system detection interrupt */
        CLK->CSSR &= (u8)(~CLK_CSSR_CSSD);
    }
    else /* CLK_PendingBit == (u8)CLK_IT_SWIF */
    {
        /* Clear the status of the clock switch interrupt */
        CLK->SWCR &= (u8)(~CLK_SWCR_SWIF);
    }

}
/**
  * @}
  */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
