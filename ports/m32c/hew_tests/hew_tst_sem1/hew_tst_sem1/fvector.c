/***********************************************************************/
/*                                                                     */
/*  FILE        :fvector.c                                             */
/*  DATE        :Thu, Feb 27, 2014                                     */
/*  DESCRIPTION :define the fixed vector table.                        */
/*  CPU GROUP   :80                                                    */
/*                                                                     */
/*  This file is generated by Renesas Project Generator (Ver.4.18).    */
/*  NOTE:THIS IS A TYPICAL EXAMPLE.                                    */
/***********************************************************************/

/********************************************************************
 *  M16C/M32C
 *  Copyright (C) 2004 (2010) Renesas Electronics Corporation.
 *  and Renesas Solutions Corporation. All rights reserved.  
 *
 *  resetprg.c : startup file
 *
 *  Function:initialize each function
 *
 * $Date: 2005/11/01 04:35:50 $
 * $Revision: 1.7 $
 ********************************************************************/
#include "vector.h"
#pragma sectaddress	fvector,ROMDATA Fvectaddr

////////////////////////////////////////////////////////////////////

#pragma interrupt/v _dummy_int	//udi
#pragma interrupt/v _dummy_int	//over_flow
#pragma interrupt/v _dummy_int	//brki
#pragma interrupt/v _dummy_int	//address_match
#pragma interrupt/v _dummy_int	//single_step
#pragma interrupt/v _dummy_int	//wdt
#pragma interrupt/v _dummy_int	//dbc
#pragma interrupt/v _dummy_int	//nmi
#pragma interrupt/v start	//reset


#pragma interrupt _dummy_int()
void _dummy_int(void){}

