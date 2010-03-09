/**
  ********************************************************************************
  * @file stm8s_uart2.c
  * @brief This file contains all the functions for the UART2 peripheral.
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
#include "stm8s_uart2.h"
#include "stm8s_clk.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Public functions ----------------------------------------------------------*/

/** @}
  * @addtogroup UART2_Public_Functions
  * @{
  */

/**
  * @brief Deinitializes the UART2 peripheral.
  * @par Full description:
  * Set the UART2 peripheral registers to their default reset values.
  * @retval None
  */

void UART2_DeInit(void)
{
    u8 dummy = 0;
    /*< Clear the Idle Line Detected bit in the status rerister by a read
       to the UART2_SR register followed by a Read to the UART2_DR register */
    dummy = UART2->SR;
    dummy = UART2->DR;

    UART2->BRR2 = UART2_BRR2_RESET_VALUE;  /*< Set UART2_BRR2 to reset value 0x00 */
    UART2->BRR1 = UART2_BRR1_RESET_VALUE;  /*< Set UART2_BRR1 to reset value 0x00 */

    UART2->CR1 = UART2_CR1_RESET_VALUE; /*< Set UART2_CR1 to reset value 0x00  */
    UART2->CR2 = UART2_CR2_RESET_VALUE; /*< Set UART2_CR2 to reset value 0x00  */
    UART2->CR3 = UART2_CR3_RESET_VALUE;  /*< Set UART2_CR3 to reset value 0x00  */
    UART2->CR4 = UART2_CR4_RESET_VALUE;  /*< Set UART2_CR4 to reset value 0x00  */
    UART2->CR5 = UART2_CR5_RESET_VALUE; /*< Set UART2_CR5 to reset value 0x00  */
    UART2->CR6 = UART2_CR6_RESET_VALUE; /*< Set UART2_CR6 to reset value 0x00  */

}

/**
  * @brief Initializes the UART2 according to the specified parameters.
  * @param[in] BaudRate: The baudrate.
  * @param[in] WordLength : This parameter can be any of the @ref UART2_WordLength_TypeDef enumeration.
  * @param[in] StopBits: This parameter can be any of the @ref UART2_StopBits_TypeDef enumeration.
  * @param[in] Parity: This parameter can be any of the @ref UART2_Parity_TypeDef enumeration.
  * @param[in] SyncMode: This parameter can be any of the @ref UART2_SyncMode_TypeDef values.
  * @param[in] Mode: This parameter can be any of the @ref UART2_Mode_TypeDef values
  * @retval None
  */
void UART2_Init(u32 BaudRate, UART2_WordLength_TypeDef WordLength, UART2_StopBits_TypeDef StopBits, UART2_Parity_TypeDef Parity, UART2_SyncMode_TypeDef SyncMode, UART2_Mode_TypeDef Mode)
{
    u8 BRR2_1, BRR2_2 = 0;
    u32 BaudRate_Mantissa, BaudRate_Mantissa100 = 0;

    /* assert_param: BaudRate value should be <= 625000 bps */
    assert_param(IS_UART2_BAUDRATE_OK(BaudRate));

    assert_param(IS_UART2_WORDLENGTH_OK(WordLength));

    assert_param(IS_UART2_STOPBITS_OK(StopBits));

    assert_param(IS_UART2_PARITY_OK(Parity));

    /* assert_param: UART2_Mode value should exclude values such as  UART2_ModeTx_Enable|UART2_ModeTx_Disable */
    assert_param(IS_UART2_MODE_OK((u8)Mode));

    /* assert_param: UART2_SyncMode value should exclude values such as
       UART2_CLOCK_ENABLE|UART2_CLOCK_DISABLE */
    assert_param(IS_UART2_SYNCMODE_OK((u8)SyncMode));

    UART2->CR1 &= (u8)(~UART2_CR1_M);  /**< Clear the word length bit */
    UART2->CR1 |= (u8)WordLength; /**< Set the word length bit according to UART2_WordLength value */

    UART2->CR3 &= (u8)(~UART2_CR3_STOP);  /**< Clear the STOP bits */
    UART2->CR3 |= (u8)StopBits;  /**< Set the STOP bits number according to UART2_StopBits value  */

    UART2->CR1 &= (u8)(~(UART2_CR1_PCEN | UART2_CR1_PS  ));  /**< Clear the Parity Control bit */
    UART2->CR1 |= (u8)Parity;  /**< Set the Parity Control bit to UART2_Parity value */

    UART2->BRR1 &= (u8)(~UART2_BRR1_DIVM);  /**< Clear the LSB mantissa of UARTDIV  */
    UART2->BRR2 &= (u8)(~UART2_BRR2_DIVM);  /**< Clear the MSB mantissa of UARTDIV  */
    UART2->BRR2 &= (u8)(~UART2_BRR2_DIVF);  /**< Clear the Fraction bits of UARTDIV */

    /**< Set the UART2 BaudRates in BRR1 and BRR2 registers according to UART2_BaudRate value */
    BaudRate_Mantissa    = ((u32)CLK_GetClockFreq() / (BaudRate << 4));
    BaudRate_Mantissa100 = (((u32)CLK_GetClockFreq() * 100) / (BaudRate << 4));
    /**< The fraction and MSB mantissa should be loaded in one step in the BRR2 register*/
    BRR2_1 = (u8)((u8)(((BaudRate_Mantissa100 - (BaudRate_Mantissa * 100))
                        << 4) / 100) & (u8)0x0F); /**< Set the fraction of UARTDIV  */
    BRR2_2 = (u8)((BaudRate_Mantissa >> 4) & (u8)0xF0);

    UART2->BRR2 = (u8)(BRR2_1 | BRR2_2);
    UART2->BRR1 = (u8)BaudRate_Mantissa;           /**< Set the LSB mantissa of UARTDIV  */

    UART2->CR2 &= (u8)~(UART2_CR2_TEN | UART2_CR2_REN); /**< Disable the Transmitter and Receiver before seting the LBCL, CPOL and CPHA bits */
    UART2->CR3 &= (u8)~(UART2_CR3_CPOL | UART2_CR3_CPHA | UART2_CR3_LBCL); /**< Clear the Clock Polarity, lock Phase, Last Bit Clock pulse */
    UART2->CR3 |= (u8)((u8)SyncMode & (u8)(UART2_CR3_CPOL | UART2_CR3_CPHA | UART2_CR3_LBCL));  /**< Set the Clock Polarity, lock Phase, Last Bit Clock pulse */

    if ((u8)Mode & (u8)UART2_MODE_TX_ENABLE)
    {
        UART2->CR2 |= (u8)UART2_CR2_TEN;  /**< Set the Transmitter Enable bit */
    }
    else
    {
        UART2->CR2 &= (u8)(~UART2_CR2_TEN);  /**< Clear the Transmitter Disable bit */
    }
    if ((u8)Mode & (u8)UART2_MODE_RX_ENABLE)
    {
        UART2->CR2 |= (u8)UART2_CR2_REN;  /**< Set the Receiver Enable bit */
    }
    else
    {
        UART2->CR2 &= (u8)(~UART2_CR2_REN);  /**< Clear the Receiver Disable bit */
    }
    /**< Set the Clock Enable bit, lock Polarity, lock Phase and Last Bit Clock pulse bits according to UART2_Mode value */
    if ((u8)SyncMode&(u8)UART2_SYNCMODE_CLOCK_DISABLE)
    {
        UART2->CR3 &= (u8)(~UART2_CR3_CKEN); /**< Clear the Clock Enable bit */
        /**< configure in Push Pull or Open Drain mode the Tx I/O line by setting the correct I/O Port register according the product package and line configuration*/
    }
    else
    {
        UART2->CR3 |= (u8)((u8)SyncMode & UART2_CR3_CKEN);
    }
}
/**
  * @brief Enable the UART2 peripheral.
  * @par Full description:
  * Enable the UART2 peripheral.
  * @param[in] NewState new state of the UART2 Communication.
  * This parameter can be:
  * - ENABLE
  * - DISABLE
  * @retval None
  */
void UART2_Cmd(FunctionalState NewState)
{

    if (NewState != DISABLE)
    {
        UART2->CR1 &= (u8)(~UART2_CR1_UARTD); /**< UART2 Enable */
    }
    else
    {
        UART2->CR1 |= UART2_CR1_UARTD;  /**< UART2 Disable (for low power consumption) */
    }
}

/**
  * @brief Enables or disables the specified UART2 interrupts.
  * @par Full description:
  * Enables or disables the specified UART2 interrupts.
  * @param[in] UART2_IT specifies the UART2 interrupt sources to be enabled or disabled.
  * This parameter can be one of the following values:
  *   - UART2_IT_LBDF:  LIN Break detection interrupt
  *   - UART2_IT_LHDF:  LIN Break detection interrupt
  *   - UART2_IT_TXE:  Tansmit Data Register empty interrupt
  *   - UART2_IT_TC:   Transmission complete interrupt
  *   - UART2_IT_RXNE_OR: Receive Data register not empty/Over run error interrupt
  *   - UART2_IT_IDLE: Idle line detection interrupt
  *   - UART2_IT_PE:   Parity Error interrupt
  * @param[in] NewState new state of the specified UART2 interrupts.
  * This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void UART2_ITConfig(UART2_IT_TypeDef UART2_IT, FunctionalState NewState)
{
    u8 uartreg, itpos = 0x00;
    assert_param(IS_UART2_CONFIG_IT_OK(UART2_IT));
    assert_param(IS_FUNCTIONALSTATE_OK(NewState));

    /* Get the UART2 register index */
    uartreg = (u8)(UART2_IT >> 0x08);

    /* Get the UART2 IT index */
    itpos = (u8)((u8)1 << (u8)((u8)UART2_IT & (u8)0x0F));

    if (NewState != DISABLE)
    {
        /**< Enable the Interrupt bits according to UART2_IT mask */
        if (uartreg == 0x01)
        {
            UART2->CR1 |= itpos;
        }
        else if (uartreg == 0x02)
        {
            UART2->CR2 |= itpos;
        }
        else if (uartreg == 0x03)
        {
            UART2->CR4 |= itpos;
        }
        else
        {
            UART2->CR6 |= itpos;
        }
    }
    else
    {
        /**< Disable the interrupt bits according to UART2_IT mask */
        if (uartreg == 0x01)
        {
            UART2->CR1 &= (u8)(~itpos);
        }
        else if (uartreg == 0x02)
        {
            UART2->CR2 &= (u8)(~itpos);
        }
        else if (uartreg == 0x03)
        {
            UART2->CR4 &= (u8)(~itpos);
        }
        else
        {
            UART2->CR6 &= (u8)(~itpos);
        }
    }
}
/**
  * @brief Configures the UART2’s IrDA interface.
  * @par Full description:
  * Configures the UART2’s IrDA interface.
  * @par This function is valid only for UART2.
  * @param[in] UART2_IrDAMode specifies the IrDA mode.
  * This parameter can be any of the @ref UART2_IrDAMode_TypeDef values.
  * @retval None
  */
void UART2_IrDAConfig(UART2_IrDAMode_TypeDef UART2_IrDAMode)
{
    assert_param(IS_UART2_IRDAMODE_OK(UART2_IrDAMode));

    if (UART2_IrDAMode != UART2_IRDAMODE_NORMAL)
    {
        UART2->CR5 |= UART2_CR5_IRLP;
    }
    else
    {
        UART2->CR5 &= ((u8)~UART2_CR5_IRLP);
    }
}

/**
  * @brief Enables or disables the UART2’s IrDA interface.
  * @par Full description:
  * Enables or disables the UART2’s IrDA interface.
  * @par This function is related to IrDA mode.
  * @param[in] NewState new state of the IrDA mode.
  * This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void UART2_IrDACmd(FunctionalState NewState)
{

    /* Check parameters */
    assert_param(IS_FUNCTIONALSTATE_OK(NewState));

    if (NewState != DISABLE)
    {
        /* Enable the IrDA mode by setting the IREN bit in the CR3 register */
        UART2->CR5 |= UART2_CR5_IREN;
    }
    else
    {
        /* Disable the IrDA mode by clearing the IREN bit in the CR3 register */
        UART2->CR5 &= ((u8)~UART2_CR5_IREN);
    }
}

/**
  * @brief Sets the UART2 LIN Break detection length.
  * @par Full description:
  * Sets the UART2 LIN Break detection length.
  * @param[in] UART2_LINBreakDetectionLength specifies the LIN break detection length.
  * This parameter can be any of the @ref UART2_LINBreakDetectionLength_TypeDef values.
  * @retval None
  */
void UART2_LINBreakDetectionConfig(UART2_LINBreakDetectionLength_TypeDef UART2_LINBreakDetectionLength)
{
    assert_param(IS_UART2_LINBREAKDETECTIONLENGTH_OK(UART2_LINBreakDetectionLength));

    if (UART2_LINBreakDetectionLength != UART2_LINBREAKDETECTIONLENGTH_10BITS)
    {
        UART2->CR4 |= UART2_CR4_LBDL;
    }
    else
    {
        UART2->CR4 &= ((u8)~UART2_CR4_LBDL);
    }
}

/**
  * @brief Configue the UART2 peripheral.
  * @par Full description:
  * Configue the UART2 peripheral.
  * @param[in] UART2_Mode specifies the LIN mode.
  * This parameter can be any of the @ref UART2_LinMode_TypeDef values.
  * @param[in] UART2_Autosync specifies the LIN automatic resynchronization mode.
  * This parameter can be any of the @ref UART2_LinAutosync_TypeDef values.
  * @param[in] UART2_DivUp specifies the LIN divider update method.
  * This parameter can be any of the @ref UART2_LinDivUp_TypeDef values.
  * @retval None
  */
void UART2_LINConfig(UART2_LinMode_TypeDef UART2_Mode, UART2_LinAutosync_TypeDef UART2_Autosync, UART2_LinDivUp_TypeDef UART2_DivUp)
{
    assert_param(IS_UART2_SLAVE_OK(UART2_Mode));

    assert_param(IS_UART2_AUTOSYNC_OK(UART2_Autosync));

    assert_param(IS_UART2_DIVUP_OK(UART2_DivUp));

    if (UART2_Mode != UART2_LIN_MODE_MASTER)
    {
        UART2->CR6 |=  UART2_CR6_LSLV;
    }
    else
    {
        UART2->CR6 &= ((u8)~UART2_CR6_LSLV);
    }

    if (UART2_Autosync != UART2_LIN_AUTOSYNC_DISABLE)
    {
        UART2->CR6 |=  UART2_CR6_LASE ;
    }
    else
    {
        UART2->CR6 &= ((u8)~ UART2_CR6_LASE );
    }

    if (UART2_DivUp != UART2_LIN_DIVUP_LBRR1)
    {
        UART2->CR6 |=  UART2_CR6_LDUM;
    }
    else
    {
        UART2->CR6 &= ((u8)~ UART2_CR6_LDUM);
    }

}

/**
  * @brief Enables or disables the UART2 LIN mode.
  * @par Full description:
  * Enables or disables the UART2’s LIN mode.
  * @param[in] NewState is new state of the UART2 LIN mode.
  * This parameter can be:
  * - ENABLE
  * - DISABLE
  * @retval None
  */
void UART2_LINCmd(FunctionalState NewState)
{
    assert_param(IS_FUNCTIONALSTATE_OK(NewState));

    if (NewState != DISABLE)
    {
        /* Enable the LIN mode by setting the LINE bit in the CR2 register */
        UART2->CR3 |= UART2_CR3_LINEN;
    }
    else
    {
        /* Disable the LIN mode by clearing the LINE bit in the CR2 register */
        UART2->CR3 &= ((u8)~UART2_CR3_LINEN);
    }
}
/**
  * @brief Enables or disables the UART2 Smart Card mode.
  * @par Full description:
  * Enables or disables the UART2 Smart Card mode.
  * @par This function is related to SmartCard mode.
  * @param[in] NewState: new state of the Smart Card mode.
  * This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void UART2_SmartCardCmd(FunctionalState NewState)
{
    assert_param(IS_FUNCTIONALSTATE_OK(NewState));

    if (NewState != DISABLE)
    {
        /* Enable the SC mode by setting the SCEN bit in the CR5 register */
        UART2->CR5 |= UART2_CR5_SCEN;
    }
    else
    {
        /* Disable the SC mode by clearing the SCEN bit in the CR5 register */
        UART2->CR5 &= ((u8)(~UART2_CR5_SCEN));
    }
}

/**
  * @brief Enables or disables NACK transmission.
  * @par Full description:
  * Enables or disables NACK transmission.
  * @par This function is valid only for UART2 because is related to SmartCard mode.
  * @param[in] NewState: new state of the Smart Card mode.
  * This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void UART2_SmartCardNACKCmd(FunctionalState NewState)
{
    assert_param(IS_FUNCTIONALSTATE_OK(NewState));

    if (NewState != DISABLE)
    {
        /* Enable the NACK transmission by setting the NACK bit in the CR5 register */
        UART2->CR5 |= UART2_CR5_NACK;
    }
    else
    {
        /* Disable the NACK transmission by clearing the NACK bit in the CR5 register */
        UART2->CR5 &= ((u8)~(UART2_CR5_NACK));
    }
}
/**
  * @brief Selects the UART2 WakeUp method.
  * @par Full description:
  * Selects the UART2 WakeUp method.
  * @param[in] UART2_WakeUp: specifies the UART2 wakeup method.
  * This parameter can be any of the @ref UART2_WakeUp_TypeDef values.
  * @retval None
  */
void UART2_WakeUpConfig(UART2_WakeUp_TypeDef UART2_WakeUp)
{
    assert_param(IS_UART2_WAKEUP_OK(UART2_WakeUp));

    UART2->CR1 &= ((u8)~UART2_CR1_WAKE);
    UART2->CR1 |= (u8)UART2_WakeUp;
}

/**
  * @brief Determines if the UART2 is in mute mode or not.
  * @par Full description:
  * Determines if the UART2 is in mute mode or not.
  * @param[in] NewState: new state of the UART2 mode.
  * This parameter can be:
  * - ENABLE
  * - DISABLE
  * @retval None
  */
void UART2_ReceiverWakeUpCmd(FunctionalState NewState)
{
    assert_param(IS_FUNCTIONALSTATE_OK(NewState));

    if (NewState != DISABLE)
    {
        /* Enable the mute mode UART2 by setting the RWU bit in the CR2 register */
        UART2->CR2 |= UART2_CR2_RWU;
    }
    else
    {
        /* Disable the mute mode UART2 by clearing the RWU bit in the CR1 register */
        UART2->CR2 &= ((u8)~UART2_CR2_RWU);
    }
}


/**
  * @brief Returns the most recent received data by the UART2 peripheral.
  * @par Full description:
  * Returns the most recent received data by the UART2 peripheral.
  * @retval u16 Received Data
  * @par Required preconditions:
  * UART2_Cmd(ENABLE);
  */
u8 UART2_ReceiveData8(void)
{
    return ((u8)UART2->DR);
}

/**
  * @brief Returns the most recent received data by the UART2 peripheral.
  * @par Full description:
  * Returns the most recent received data by the UART2 peripheral.
  * @retval u16 Received Data
  * @par Required preconditions:
  * UART2_Cmd(ENABLE);
  */
u16 UART2_ReceiveData9(void)
{
    return (u16)((((u16)UART2->DR) | ((u16)(((u16)((u16)UART2->CR1 & (u16)UART2_CR1_R8)) << 1))) & ((u16)0x01FF));
}



/**
  * @brief Transmits 8 bit data through the UART2 peripheral.
  * @par Full description:
  * Transmits 8 bit data through the UART2 peripheral.
  * @param[in] Data: the data to transmit.
  * @retval None
  * @par Required preconditions:
  * UART2_Cmd(ENABLE);
  */
void UART2_SendData8(u8 Data)
{
    /* Transmit Data */
    UART2->DR = Data;
}

/**
  * @brief Transmits 9 bit data through the UART2 peripheral.
  * @par Full description:
  * Transmits 9 bit data through the UART2 peripheral.
  * @param[in] Data: the data to transmit.
  * @retval None
  * @par Required preconditions:
  * UART2_Cmd(ENABLE);
  */
void UART2_SendData9(u16 Data)
{
    UART2->CR1 &= ((u8)~UART2_CR1_T8);                  /* Clear the transmit data bit 8     */
    UART2->CR1 |= (u8)(((u8)(Data >> 2)) & UART2_CR1_T8); /* Write the transmit data bit [8]   */
    UART2->DR   = (u8)(Data);                    /* Write the transmit data bit [0:7] */

}
/**
  * @brief Transmits break characters.
  * @par Full description:
  * Transmits break characters on the UART2 peripheral.
  * @retval None
  */
void UART2_SendBreak(void)
{
    UART2->CR2 |= UART2_CR2_SBK;
}
/**
  * @brief Sets the address of the UART2 node.
  * @par Full description:
  * Sets the address of the UART2 node.
  * @param[in] UART2_Address: Indicates the address of the UART2 node.
  * @retval None
  */

void UART2_SetAddress(u8 UART2_Address)
{
    /*assert_param for x UART2_Address*/
    assert_param(IS_UART2_ADDRESS_OK(UART2_Address));

    /* Clear the UART2 address */
    UART2->CR4 &= ((u8)~UART2_CR4_ADD);
    /* Set the UART2 address node */
    UART2->CR4 |= UART2_Address;
}
/**
  * @brief Sets the specified UART2 guard time.
  * @par Full description:
  * Sets the address of the UART2 node.
  * @par This function is related to SmartCard mode.
  * @param[in] UART2_GuardTime: specifies the guard time.
  * @retval None
  * @par Required preconditions:
  * SmartCard Mode Enabled
  */
void UART2_SetGuardTime(u8 UART2_GuardTime)
{
    /* Set the UART2 guard time */
    UART2->GTR = UART2_GuardTime;
}

/**
  * @brief Sets the system clock prescaler.
  * @par Full description:
  * Sets the system clock prescaler.
  * @par This function is related to SmartCard and IrDa mode.
  * @param[in] UART2_Prescaler: specifies the prescaler clock.
  *                    This parameter can be one of the following values:
  *                       @par IrDA Low Power Mode
  *   The clock source is diveded by the value given in the register (8 bits)
  *                       - 0000 0000 Reserved
  *                       - 0000 0001 divides the clock source by 1
  *                       - 0000 0010 divides the clock source by 2
  *                       - ...........................................................
  *                       @par Smart Card Mode
  *   The clock source is diveded by the value given in the register (5 significant bits) multipied by 2
  *                       - 0 0000 Reserved
  *                       - 0 0001 divides the clock source by 2
  *                       - 0 0010 divides the clock source by 4
  *                       - 0 0011 divides the clock source by 6
  *                       - ...........................................................
  * @retval None
  * @par Required preconditions:
  * IrDA Low Power mode or smartcard mode enabled
  */
void UART2_SetPrescaler(u8 UART2_Prescaler)
{
    /* Load the UART2 prescaler value*/
    UART2->PSCR = UART2_Prescaler;
}

/**
  * @brief Checks whether the specified UART2 flag is set or not.
  * @par Full description:
  * Checks whether the specified UART2 flag is set or not.
  * @param[in] UART2_FLAG specifies the flag to check.
  * This parameter can be any of the @ref UART2_Flag_TypeDef enumeration.
  * @retval FlagStatus (SET or RESET)
  */

FlagStatus UART2_GetFlagStatus(UART2_Flag_TypeDef UART2_FLAG)
{
    FlagStatus status = RESET;

    /* Check parameters */
    assert_param(IS_UART2_FLAG_OK(UART2_FLAG));

    /* Check the status of the specified UART2 flag*/
    if (UART2_FLAG == UART2_FLAG_LBDF)
    {
        if ((UART2->CR4 & (u8)UART2_FLAG) != (u8)0x00)
        {
            /* UART2_FLAG is set*/
            status = SET;
        }
        else
        {
            /* UART2_FLAG is reset*/
            status = RESET;
        }
    }
    else if (UART2_FLAG == UART2_FLAG_SBK)
    {
        if ((UART2->CR2 & (u8)UART2_FLAG) != (u8)0x00)
        {
            /* UART2_FLAG is set*/
            status = SET;
        }
        else
        {
            /* UART2_FLAG is reset*/
            status = RESET;
        }
    }
    else if ((UART2_FLAG == UART2_FLAG_LHDF) || (UART2_FLAG == UART2_FLAG_LSF))
    {
        if ((UART2->CR6 & (u8)UART2_FLAG) != (u8)0x00)
        {
            /* UART2_FLAG is set*/
            status = SET;
        }
        else
        {
            /* UART2_FLAG is reset*/
            status = RESET;
        }
    }
    else
    {
        if ((UART2->SR & (u8)UART2_FLAG) != (u8)0x00)
        {
            /* UART2_FLAG is set*/
            status = SET;
        }
        else
        {
            /* UART2_FLAG is reset*/
            status = RESET;
        }
    }

    /* Return the UART2_FLAG status*/
    return  status;
}
/**
 * @brief Clears the UART2 flags.
 * @par Full description:
 * Clears the UART2 flags.
 * @param[in] UART2_FLAG specifies the flag to clear
 * This parameter can be any combination of the following values:
 *   - UART2_FLAG_LBDF: LIN Break detection flag.
 *   - UART2_FLAG_LHDF: LIN Header detection flag.
 *   - UART2_FLAG_LSF: LIN synchrone field flag.
 *   - UART2_FLAG_RXNE: Receive data register not empty flag.
 * @par Notes:
 *   - PE (Parity error), FE (Framing error), NE (Noise error), OR (OverRun error)
 *     and IDLE (Idle line detected) flags are cleared by software sequence: a read
 *     operation to UART2_SR register (UART2_GetFlagStatus())followed by a read operation
 *     to UART2_DR register(UART2_ReceiveData8() or UART2_ReceiveData9()).
 *   - RXNE flag can be also cleared by a read to the UART2_DR register
 *     (UART2_ReceiveData8()or UART2_ReceiveData9()).
 *   - TC flag can be also cleared by software sequence: a read operation to UART2_SR
 *     register (UART2_GetFlagStatus()) followed by a write operation to UART2_DR register
 *     (UART2_SendData8() or UART2_SendData9()).
 *   - TXE flag is cleared only by a write to the UART2_DR register (UART2_SendData8() or
 *     UART2_SendData9()).
 *   - SBK flag is cleared during the stop bit of break.
 * @retval None
 */

void UART2_ClearFlag(UART2_Flag_TypeDef UART2_FLAG)
{
    assert_param(IS_UART2_CLEAR_FLAG_OK(UART2_FLAG));

    /*< Clear the Receive Register Not Empty flag */
    if (UART2_FLAG == UART2_FLAG_RXNE)
    {
        UART2->SR = (u8)~(UART2_SR_RXNE);
    }
    /*< Clear the LIN Break Detection flag */
    else if (UART2_FLAG == UART2_FLAG_LBDF)
    {
        UART2->CR4 &= (u8)(~UART2_CR4_LBDF);
    }
    /*< Clear the LIN Header Detection Flag */
    else if (UART2_FLAG == UART2_FLAG_LHDF)
    {
        UART2->CR6 &= (u8)(~UART2_CR6_LHDF);
    }
    /*< Clear the LIN Synch Field flag */
    else
    {
        UART2->CR6 &= (u8)(~UART2_CR6_LSF);
    }

}

/**
  * @brief Checks whether the specified UART2 interrupt has occurred or not.
  * @par Full description:
  * Checks whether the specified UART2 interrupt has occurred or not.
  * @param[in] UART2_IT: Specifies the UART2 interrupt pending bit to check.
  * This parameter can be one of the following values:
  *   - UART2_IT_LBDF:  LIN Break detection interrupt
  *   - UART2_IT_TXE:  Tansmit Data Register empty interrupt
  *   - UART2_IT_TC:   Transmission complete interrupt
  *   - UART2_IT_RXNE: Receive Data register not empty interrupt
  *   - UART2_IT_IDLE: Idle line detection interrupt
  *   - UART2_IT_OR:  OverRun Error interrupt
  *   - UART2_IT_PE:   Parity Error interrupt
  * @retval
  * ITStatus The new state of UART2_IT (SET or RESET).
  */
ITStatus UART2_GetITStatus(UART2_IT_TypeDef UART2_IT)
{
    ITStatus pendingbitstatus = RESET;
    u8 itpos = 0;
    u8 itmask1 = 0;
    u8 itmask2 = 0;
    u8 enablestatus = 0;

    /* Check parameters */
    assert_param(IS_UART2_GET_IT_OK(UART2_IT));

    /* Get the UART2 IT index*/
    itpos = (u8)((u8)1 << (u8)((u8)UART2_IT & (u8)0x0F));
    /* Get the UART2 IT index*/
    itmask1 = (u8)((u8)UART2_IT >> (u8)4);
    /* Set the IT mask*/
    itmask2 = (u8)((u8)1 << itmask1);



    /* Check the status of the specified UART2 pending bit*/
    if (UART2_IT == UART2_IT_PE)
    {
        /* Get the UART2_ITPENDINGBIT enable bit status*/
        enablestatus = (u8)((u8)UART2->CR1 & itmask2);
        /* Check the status of the specified UART2 interrupt*/

        if (((UART2->SR & itpos) != (u8)0x00) && enablestatus)
        {
            /* Interrupt occurred*/
            pendingbitstatus = SET;
        }
        else
        {
            /* Interrupt not occurred*/
            pendingbitstatus = RESET;
        }
    }

    else if (UART2_IT == UART2_IT_LBDF)
    {
        /* Get the UART2_IT enable bit status*/
        enablestatus = (u8)((u8)UART2->CR4 & itmask2);
        /* Check the status of the specified UART2 interrupt*/
        if (((UART2->CR4 & itpos) != (u8)0x00) && enablestatus)
        {
            /* Interrupt occurred*/
            pendingbitstatus = SET;
        }
        else
        {
            /* Interrupt not occurred*/
            pendingbitstatus = RESET;
        }
    }
    else if (UART2_IT == UART2_IT_LHDF)
    {
        /* Get the UART2_IT enable bit status*/
        enablestatus = (u8)((u8)UART2->CR6 & itmask2);
        /* Check the status of the specified UART2 interrupt*/
        if (((UART2->CR6 & itpos) != (u8)0x00) && enablestatus)
        {
            /* Interrupt occurred*/
            pendingbitstatus = SET;
        }
        else
        {
            /* Interrupt not occurred*/
            pendingbitstatus = RESET;
        }
    }
    else
    {
        /* Get the UART2_IT enable bit status*/
        enablestatus = (u8)((u8)UART2->CR2 & itmask2);
        /* Check the status of the specified UART2 interrupt*/
        if (((UART2->SR & itpos) != (u8)0x00) && enablestatus)
        {
            /* Interrupt occurred*/
            pendingbitstatus = SET;
        }
        else
        {
            /* Interrupt not occurred*/
            pendingbitstatus = RESET;
        }
    }
    /* Return the UART2_IT status*/
    return  pendingbitstatus;
}

/**
 * @brief Clears the UART2 pending flags.
 * @par Full description:
 * Clears the UART2 pending bit.
 * @param[in] UART2_IT specifies the pending bit to clear
 * This parameter can be one of the following values:
 *   - UART2_IT_LBDF:  LIN Break detection interrupt
 *   - UART2_IT_LHDF:  LIN Header detection interrupt
 *   - UART2_IT_RXNE: Receive Data register not empty interrupt.
 *
 * @par Notes:
 *   - PE (Parity error), FE (Framing error), NE (Noise error), OR (OverRun error) and
 *     IDLE (Idle line detected) pending bits are cleared by software sequence: a read
 *     operation to UART2_SR register (UART2_GetITStatus()) followed by a read operation
 *     to UART2_DR register (UART2_ReceiveData8() or UART2_ReceiveData9() ).
 *   - RXNE pending bit can be also cleared by a read to the UART2_DR register
 *     (UART2_ReceiveData8() or UART2_ReceiveData9() ).
 *   - TC (Transmit complet) pending bit can be cleared by software sequence: a read
 *     operation to UART2_SR register (UART2_GetITStatus()) followed by a write operation
 *     to UART2_DR register (UART2_SendData8()or UART2_SendData9()).
 *   - TXE pending bit is cleared only by a write to the UART2_DR register
 *     (UART2_SendData8() or UART2_SendData9()).
 * @retval None
 */
void UART2_ClearITPendingBit(UART2_IT_TypeDef UART2_IT)
{
    assert_param(IS_UART2_CLEAR_IT_OK(UART2_IT));

    /*< Clear the Receive Register Not Empty pending bit */
    if (UART2_IT == UART2_IT_RXNE)
    {
        UART2->SR = (u8)~(UART2_SR_RXNE);
    }
    /*< Clear the LIN Break Detection pending bit */
    else if (UART2_IT == UART2_IT_LBDF)
    {
        UART2->CR4 &= (u8)~(UART2_CR4_LBDF);
    }
    /*< Clear the LIN Header Detection pending bit */
    else
    {
        UART2->CR6 &= (u8)(~UART2_CR6_LHDF);
    }
}
/**
  * @}
  */


/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
