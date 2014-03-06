/***********************************************************************/
/*                                                                     */
/*  FILE        :initsct.h                                             */
/*  DATE        :Thu, Feb 27, 2014                                     */
/*  DESCRIPTION :define the macro for initialization of sections.      */
/*  CPU GROUP   :80                                                    */
/*                                                                     */
/*  This file is generated by Renesas Project Generator (Ver.4.18).    */
/*  NOTE:THIS IS A TYPICAL EXAMPLE.                                    */
/***********************************************************************/
/*********************************************************************
*
* Device     : M32C/80,M16C/80/70
*
* File Name  : initsct.h
*
* Abstract   : define the macro for initialization of sections.(Do not modify)
*
* History    : 1.80  (2009-12-08)
*
* NOTE       : THIS IS A TYPICAL EXAMPLE.
*
* Copyright (C) 2009 (2010) Renesas Electronics Corporation.
*  and Renesas Solutions Corporation. All rights reserved.
*
*********************************************************************/

#pragma section program interrupt
#define	sclear(X,Y,Z)	_asm("	.initsct "X","Y","Z"\n"\
			"	mov.b	#00H,R0L\n"\
			"	mov.l	#(topof	"X") ,A1\n"\
			"	mov.w	#sizeof	"X",R3\n"\
			"	sstr.b");

#define sclear_f(X,Y,Z)	_asm("	.initsct "X","Y","Z"\n"\
			"	push.w	#(sizeof "X")>>16\n"\
			"	push.w	#(sizeof "X")&0ffffH\n"\
			"	pusha	(topof "X")\n"\
			"	.stk	8\n"\
			"	.glb	_bzero\n"\
			"	.call	_bzero,G\n"\
			"	jsr.a	_bzero\n"\
			"	add.l	#8H,sp\n"\
			"	.stk	-8");


#define scopy(X,Y,Z)	_asm("	.initsct "X","Y","Z"\n"\
			"	.initsct "X"I,rom"Y",noalign\n"\
			"	mov.l	#(topof "X"I),A0\n"\
			"	mov.l	#(topof "X"),A1\n"\
			"	mov.w	#sizeof	"X",R3\n"\
			"	smovf.b");

#define	scopy_f(X,Y,Z)	_asm("	.initsct "X","Y","Z"\n"\
			"	.initsct "X"I,rom"Y",noalign\n"\
			"	push.w	#(sizeof "X") >> 16\n"\
			"	push.w	#(sizeof "X") & 0ffffH\n"\
			"	pusha	(topof "X")\n"\
			"	pusha	(topof "X"I)\n"\
			"	.stk	12\n"\
			"	.glb	_bcopy\n"\
			"	.call	_bcopy,G\n"\
			"	jsr.a	_bcopy\n"\
			"	add.l	#8H,sp\n"\
			"	.stk	-12");

