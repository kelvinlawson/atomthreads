/*
 * Copyright (c) 2010, Atomthreads Project. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. No personal names or organizations' names associated with the
 *    Atomthreads project may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE ATOMTHREADS PROJECT AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE PROJECT OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __ATOMPORT_REGS_H_
#define __ATOMPORT_REGS_H_


#define zero             $0
#define at               $1
#define v0               $2
#define v1               $3
#define a0               $4
#define a1               $5
#define a2               $6
#define a3               $7
#define t0               $8
#define t1               $9
#define t2               $10
#define t3               $11
#define t4               $12
#define t5               $13
#define t6               $14
#define t7               $15
#define t8               $24
#define t9               $25
#define s0               $16
#define s1               $17
#define s2               $18
#define s3               $19
#define s4               $20
#define s5               $21
#define s6               $22
#define s7               $23
#define k0               $26
#define k1               $27
#define gp               $28
#define sp               $29
#define s8               $30
#define fp               $30
#define ra               $31

#define NUM_REGISTERS    32
#define WORD_SIZE        4

#define v0_IDX           0
#define v1_IDX           1
#define a0_IDX           2
#define a1_IDX           3
#define a2_IDX           4
#define a3_IDX           5
#define t0_IDX           6
#define t1_IDX           7
#define t2_IDX           8
#define t3_IDX           9
#define t4_IDX           10
#define t5_IDX           11
#define t6_IDX           12
#define t7_IDX           13
#define s0_IDX           14
#define s1_IDX           15
#define s2_IDX           16
#define s3_IDX           17
#define s4_IDX           18
#define s5_IDX           19
#define s6_IDX           20
#define s7_IDX           21
#define t8_IDX           22
#define t9_IDX           23
#define sp_IDX           24
#define gp_IDX           25
#define s8_IDX           26
#define ra_IDX           27
#define k0_IDX           28
#define k1_IDX           29
#define at_IDX           30
#define zero_IDX         31
#define cp0_epc_IDX      32
#define cp0_status_IDX   33
#define cp_cause_IDX     34

#define NUM_CTX_REGS     35

#define CP0_INDEX        $0
#define CP0_RANDOM       $1
#define CP0_ENTRYLO0     $2
#define CP0_ENTRYLO1     $3
#define CP0_CONTEXT      $4
#define CP0_PAGEMASK     $5
#define CP0_WIRED        $6
#define CP0_HWRENA       $7
#define CP0_BADVADDR     $8
#define CP0_COUNT        $9
#define CP0_ENTRYHI      $10
#define CP0_COMPARE      $11
#define CP0_STATUS       $12
#define CP0_INTCTL       $12,1
#define CP0_SRSCTL       $12,2
#define CP0_SRSMAP       $12,3
#define CP0_CAUSE        $13
#define CP0_EPC          $14
#define CP0_PRID         $15
#define CP0_EBASE        $15,1
#define CP0_CONFIG       $16
#define CP0_CONFIG1      $16,1
#define CP0_CONFIG2      $16,2
#define CP0_CONFIG3      $16,3
#define CP0_LLADDR       $17
#define CP0_WATCHLO      $18
#define CP0_WATCHHI      $19
#define CP0_DEBUG        $23
#define CP0_DEPC         $24
#define CP0_PERFCTL      $25,0
#define CP0_PERFCNT      $25,1
#define CP0_ECC          $26
#define CP0_CACHEERR     $27
#define CP0_TAGLO        $28
#define CP0_DATALO       $28,1
#define CP0_TAGHI        $29
#define CP0_DATAHI       $29,1
#define CP0_ERRORPC      $30

#endif /* __ATOMPORT_REGS_H_ */
