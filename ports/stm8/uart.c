#include <stdio.h>

#include "stm8s.h"

#include "atom.h"
#include "atommutex.h"
#include "uart.h"


/*
 * Semaphore for single-threaded access to UART device
 */
static ATOM_MUTEX uart_mutex;


/*
 * Initialize the UART to requested baudrate, tx/rx, 8N1.
 */
int uart_init(uint32_t baudrate)
{
  int status;
  
  /**
   * Set up UART2 for putting out debug messages.
   * This the UART used on STM8S Discovery, change if required.
   */
  UART2_DeInit();
  UART2_Init (baudrate, UART2_WORDLENGTH_8D, UART2_STOPBITS_1, UART2_PARITY_NO,
              UART2_SYNCMODE_CLOCK_DISABLE, UART2_MODE_TXRX_ENABLE);

  /* Create a mutex for single-threaded putchar() access */
  if (atomMutexCreate (&uart_mutex) != ATOM_OK)
  {
    status = -1;
  }
  else
  {
    status = 0;
  }

  /* Finished */
  return (status);
}


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
    /* Block on private access to the UART */
    if (atomMutexGet(&uart_mutex, 0) == ATOM_OK)
    {
        /* Convert \n to \r\n */
        if (c == '\n')
            putchar('\r');

        /* Write a character to the UART2 */
        UART2_SendData8(c);
      
        /* Loop until the end of transmission */
        while (UART2_GetFlagStatus(UART2_FLAG_TXE) == RESET)
            ;

    }

    return (c);
}

