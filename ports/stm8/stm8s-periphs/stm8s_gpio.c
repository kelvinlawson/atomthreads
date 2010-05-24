/**
  ******************************************************************************
  * @file stm8s_gpio.c
  * @brief This file contains all the functions for the GPIO peripheral.
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
#include "stm8s_gpio.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* Public functions ----------------------------------------------------------*/

/**
  * @addtogroup GPIO_Public_Functions
  * @{
  */

/**
  * @brief Deinitializes the GPIOx peripheral registers to their default reset
  * values.
  * @param[in] GPIOx : Select the GPIO peripheral number (x = A to I).
  * @retval None
  */
void GPIO_DeInit(GPIO_TypeDef* GPIOx)
{
    GPIOx->ODR = GPIO_ODR_RESET_VALUE; /* Reset Output Data Register */
    GPIOx->DDR = GPIO_DDR_RESET_VALUE; /* Reset Data Direction Register */
    GPIOx->CR1 = GPIO_CR1_RESET_VALUE; /* Reset Control Register 1 */
    GPIOx->CR2 = GPIO_CR2_RESET_VALUE; /* Reset Control Register 2 */
}

/**
  * @brief Initializes the GPIOx according to the specified parameters.
  * @param[in] GPIOx : Select the GPIO peripheral number (x = A to I).
  * @param[in] GPIO_Pin : This parameter contains the pin number, it can be one or many members
  * of the @ref GPIO_Pin_TypeDef enumeration.
  * @param[in] GPIO_Mode : This parameter can be any of the @Ref GPIO_Mode_TypeDef enumeration.
  * @retval None
  */

void GPIO_Init(GPIO_TypeDef* GPIOx,
               GPIO_Pin_TypeDef GPIO_Pin,
               GPIO_Mode_TypeDef GPIO_Mode)
{
    /*----------------------*/
    /* Check the parameters */
    /*----------------------*/

    assert_param(IS_GPIO_MODE_OK(GPIO_Mode));
    assert_param(IS_GPIO_PIN_OK(GPIO_Pin));

    /*-----------------------------*/
    /* Input/Output mode selection */
    /*-----------------------------*/

    if ((((u8)(GPIO_Mode)) & (u8)0x80) != (u8)0x00) /* Output mode */
    {
        if ((((u8)(GPIO_Mode)) & (u8)0x10) != (u8)0x00) /* High level */
        {
            GPIOx->ODR |= (u8)GPIO_Pin;
        } else /* Low level */
        {
            GPIOx->ODR &= (u8)(~(GPIO_Pin));
        }
        /* Set Output mode */
        GPIOx->DDR |= (u8)GPIO_Pin;
    } else /* Input mode */
    {
        /* Set Input mode */
        GPIOx->DDR &= (u8)(~(GPIO_Pin));
    }

    /*------------------------------------------------------------------------*/
    /* Pull-Up/Float (Input) or Push-Pull/Open-Drain (Output) modes selection */
    /*------------------------------------------------------------------------*/

    if ((((u8)(GPIO_Mode)) & (u8)0x40) != (u8)0x00) /* Pull-Up or Push-Pull */
    {
        GPIOx->CR1 |= (u8)GPIO_Pin;
    } else /* Float or Open-Drain */
    {
        GPIOx->CR1 &= (u8)(~(GPIO_Pin));
    }

    /*-----------------------------------------------------*/
    /* Interrupt (Input) or Slope (Output) modes selection */
    /*-----------------------------------------------------*/

    if ((((u8)(GPIO_Mode)) & (u8)0x20) != (u8)0x00) /* Interrupt or Slow slope */
    {
        GPIOx->CR2 |= (u8)GPIO_Pin;
    } else /* No external interrupt or No slope control */
    {
        GPIOx->CR2 &= (u8)(~(GPIO_Pin));
    }

}


/**
  * @brief Writes reverse level to the specified GPIO pins.
  * @param[in] GPIOx : Select the GPIO peripheral number (x = A to I).
  * @param[in] PortPins : Specifies the pins to be reversed to the port output.
  * data register.
  * @retval None
  * @par Required preconditions:
  * The port must be configured in output mode.
  */
void GPIO_WriteReverse(GPIO_TypeDef* GPIOx, GPIO_Pin_TypeDef PortPins)
{
    GPIOx->ODR ^= (u8)PortPins;
}


/**
  * @}
  */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
