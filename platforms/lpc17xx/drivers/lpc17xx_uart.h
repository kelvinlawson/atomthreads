/**************************************************************************//**
 * @file     lpc17xx_uart.h
 * @brief    Header file for lpc17xx_uart.c.
 * @version  1.0
 * @date     18. Nov. 2010
 *
 * @note
 * Copyright (C) 2010 NXP Semiconductors(NXP). All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * products. This software is supplied "AS IS" without any warranties.
 * NXP Semiconductors assumes no responsibility or liability for the
 * use of the software, conveys no license or title under any patent,
 * copyright, or mask work right to the product. NXP Semiconductors
 * reserves the right to make changes in the software without
 * notification. NXP Semiconductors also make no representation or
 * warranty that such application will be suitable for the specified
 * use without further testing or modification.
 ******************************************************************************/

#ifndef __LPC17xx_UART_H_
#define __LPC17xx_UART_H_

#include <stdint.h>

/* external functions */
void    LPC17xx_UART_PutChar (uint8_t);
uint8_t LPC17xx_UART_GetChar (void);
void  	LPC17xx_UART_Init(uint32_t baudrate);
//void  	LPC17xx_UART_Printf (const uint8_t *format, ...);
void LPC17xx_UART_PutString (uint8_t *str) ;

#endif // __LPC17xx_UART_H_

/* --------------------------------- End Of File ------------------------------ */
