#include <stdio.h>
#include <stddef.h>

#include "stm8l15x.h"

#include "atom.h"
#include "uart.h"

/*
 * Initialize the UART to requested baudrate, tx/rx, 8N1.
 */
int uart_init(uint32_t baudrate)
{
  /* Enable USART clock */
  CLK_PeripheralClockConfig(CLK_Peripheral_USART1, ENABLE);

  /* Configure USART Tx as alternate function push-pull  (software pull up)*/
  GPIO_ExternalPullUpConfig(GPIOC, GPIO_Pin_3, ENABLE);

  /* Configure USART Rx as alternate function push-pull  (software pull up)*/
  GPIO_ExternalPullUpConfig(GPIOC, GPIO_Pin_2, ENABLE);

  /* USART configuration */
  USART_Init(USART1, baudrate, USART_WordLength_8b, USART_StopBits_1,
                   USART_Parity_No, (USART_Mode_TypeDef)(USART_Mode_Tx | USART_Mode_Rx));
  USART_ClearFlag(USART1, USART_FLAG_TC);

  /* Finished */
  return ATOM_OK;
}


/**
 * \b uart_putchar
 *
 * Write a char out via UART2
 *
 * @param[in] c Character to send
 *
 * @return Character sent
 */
char uart_putchar (char c)
{
        /* Convert \n to \r\n */
        if (c == '\n')
            putchar('\r');

        USART_ClearFlag(USART1,USART_FLAG_TC);
        /* Write a character to the USART */
        USART_SendData8(USART1, c);
        /* Loop until the end of transmission */
        while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);

    return (c);
}


/* COSMIC: Requires putchar() routine to override stdio */
#if defined(__CSMC__)
/**
 * \b putchar
 *
 * Retarget putchar() to use UART2
 *
 * @param[in] c Character to send
 *
 * @return Character sent
 */
char putchar (char c)
{
    return (uart_putchar(c));
}
#endif /* __CSMC__ */
