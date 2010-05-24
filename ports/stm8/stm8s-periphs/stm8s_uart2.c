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

/**
* IAR EWSTM8: Ignore unused variable warning on dummy variable.
 */
#ifdef __IAR_SYSTEMS_ICC__
#pragma diag_suppress=Pe550
#endif


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
  * @}
  */


/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
