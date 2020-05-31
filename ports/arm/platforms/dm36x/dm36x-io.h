/*
 * Copyright (c) 2013, Kelvin Lawson. All rights reserved.
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

#ifndef __DM36X_IO_H__
#define __DM36X_IO_H__

#include "atomport.h"


/** Timer input clock speed: 24MHz */
#define TIMER_CLK       24000000


/*
 * IO Addresses for use with DM36x
 */

/** EDMA3 registers */
#define DM36X_EDMA3_CC_BASE         0x01C00000 /* EDMA3 CC registers */
#define DM36X_EDMA3_PARAM_BASE      0x01C04000 /* EDMA3 PaRAM base */
#define DM36X_EDMA3_TC0_BASE        0x01C10000 /* EDMA3 TC0 registers */
#define DM36X_EDMA3_TC1_BASE        0x01C10400 /* EDMA3 TC1 registers */
#define DM36X_EDMA3_TC2_BASE        0x01C10800 /* EDMA3 TC2 registers */
#define DM36X_EDMA3_TC3_BASE        0x01C10C00 /* EDMA3 TC3 registers */
/* EDMA3 CC global register offsets */
#define DM36X_EDMA3_DMAQNUM0        0x0240 /* Channel/queue mapping 0 */
#define DM36X_EDMA3_DMAQNUM1        0x0244 /* Channel/queue mapping 1 */
#define DM36X_EDMA3_DMAQNUM2        0x0248 /* Channel/queue mapping 2 */
#define DM36X_EDMA3_DMAQNUM3        0x024C /* Channel/queue mapping 3 */
#define DM36X_EDMA3_DMAQNUM4        0x0250 /* Channel/queue mapping 4 */
#define DM36X_EDMA3_DMAQNUM5        0x0254 /* Channel/queue mapping 5 */
#define DM36X_EDMA3_DMAQNUM6        0x0258 /* Channel/queue mapping 6 */
#define DM36X_EDMA3_DMAQNUM7        0x025C /* Channel/queue mapping 7 */
#define DM36X_EDMA3_QDMAQNUM        0x0260 /* QDMA queue mapping */
#define DM36X_EDMA3_QUEPRI          0x0284 /* Queue Priority */
#define DM36X_EDMA3_CC_EMR          0x0300 /* Event missed */
#define DM36X_EDMA3_CC_EMRH         0x0304 /* Event missed high */
#define DM36X_EDMA3_CC_EMCR         0x0308 /* Event missed clear */
#define DM36X_EDMA3_CC_EMCRH        0x030C /* Event missed clear high */
#define DM36X_EDMA3_CC_QEMR         0x0310 /* QDMA Event missed */
#define DM36X_EDMA3_CC_QEMCR        0x0314 /* QDMA Event missed clear */
#define DM36X_EDMA3_CC_CCERR        0x0318 /* EDMA3CC Error */
#define DM36X_EDMA3_CC_CCERRCLR     0x031C /* EDMA3CC Error clear */
#define DM36X_EDMA3_CC_EEVAL        0x0320 /* Error evaluate */
#define DM36X_EDMA3_CC_DRAE0        0x0340 /* Shadow region enable */
#define DM36X_EDMA3_CC_DRAEH0       0x0344 /* Shadow region enable */
#define DM36X_EDMA3_CC_QSTAT0       0x0600 /* Queue 0 status */
#define DM36X_EDMA3_CC_QSTAT1       0x0604 /* Queue 1 status */
#define DM36X_EDMA3_CC_QSTAT2       0x0608 /* Queue 2 status */
#define DM36X_EDMA3_CC_QSTAT3       0x060C /* Queue 3 status */
#define DM36X_EDMA3_CC_CCSTAT       0x0640 /* EDMA3CC Status */
/* EDMA3 CC global channel register offsets */
#define DM36X_EDMA3_CC_ER           0x1000 /* Event */
#define DM36X_EDMA3_CC_ERH          0x1004 /* Event high */
#define DM36X_EDMA3_CC_ECR          0x1008 /* Event clear */
#define DM36X_EDMA3_CC_ECRH         0x100C /* Event clear high */
#define DM36X_EDMA3_CC_ESR          0x1010 /* Event set */
#define DM36X_EDMA3_CC_ESRH         0x1014 /* Event set high */
#define DM36X_EDMA3_CC_CER          0x1018 /* Chained event */
#define DM36X_EDMA3_CC_CERH         0x101C /* Chained event high */
#define DM36X_EDMA3_CC_EER          0x1020 /* Event enable */
#define DM36X_EDMA3_CC_EERH         0x1024 /* Event enable high */
#define DM36X_EDMA3_CC_EECR         0x1028 /* Event enable clear */
#define DM36X_EDMA3_CC_EECRH        0x102C /* Event enable clear high */
#define DM36X_EDMA3_CC_EESR         0x1030 /* Event enable set */
#define DM36X_EDMA3_CC_EESRH        0x1034 /* Event enable set high */
#define DM36X_EDMA3_CC_SER          0x1038 /* Secondary event */
#define DM36X_EDMA3_CC_SERH         0x103C /* Secondary event high */
#define DM36X_EDMA3_CC_SECR         0x1040 /* Secondary event clear */
#define DM36X_EDMA3_CC_SECRH        0x1044 /* Secondary event clear high */
#define DM36X_EDMA3_CC_IER          0x1050 /* Interrupt enable */
#define DM36X_EDMA3_CC_IERH         0x1054 /* Interrupt enable high */
#define DM36X_EDMA3_CC_IECR         0x1058 /* Interrupt enable clear */
#define DM36X_EDMA3_CC_IECRH        0x105C /* Interrupt enable clear high */
#define DM36X_EDMA3_CC_IESR         0x1060 /* Interrupt enable set */
#define DM36X_EDMA3_CC_IESRH        0x1064 /* Interrupt enable set high */
#define DM36X_EDMA3_CC_IPR          0x1068 /* Interrupt pending */
#define DM36X_EDMA3_CC_IPRH         0x106C /* Interrupt pending high */
#define DM36X_EDMA3_CC_ICR          0x1070 /* Interrupt clear */
#define DM36X_EDMA3_CC_ICRH         0x1074 /* Interrupt clear high */
#define DM36X_EDMA3_CC_IEVAL        0x1078 /* Interrupt evaluate */
#define DM36X_EDMA3_CC_QER          0x1080 /* QDMA event */
#define DM36X_EDMA3_CC_QEER         0x1084 /* QDMA event enable */
#define DM36X_EDMA3_CC_QEECR        0x1088 /* QDMA event enable clear */
#define DM36X_EDMA3_CC_QEESR        0x108C /* QDMA event enable set */
#define DM36X_EDMA3_CC_QSER         0x1090 /* QDMA secondary event */
#define DM36X_EDMA3_CC_QSECR        0x1094 /* QDMA secondary event clear */
/* EDMA3 TC register offsets */
#define DM36X_EDMA3_TC_TCSTAT       0x0100 /* EDMA3 TC channel status */
#define DM36X_EDMA3_TC_ERRSTAT      0x0120 /* Error status */
#define DM36X_EDMA3_TC_ERREN        0x0124 /* Error enable */
#define DM36X_EDMA3_TC_ERRCLR       0x0128 /* Error clear */
#define DM36X_EDMA3_TC_ERRDET       0x012C /* Error details */
#define DM36X_EDMA3_TC_RDRATE       0x0140 /* Read rate */
/* EDMA3 channel mapping */
#define DM36X_EDMA3_CHAN_TIMER3_TEVT6         0
#define DM36X_EDMA3_CHAN_TIMER3_TEVT7         1
#define DM36X_EDMA3_CHAN_MCBSP_XEVT           2
#define DM36X_EDMA3_CHAN_MCBSP_REVT           3
#define DM36X_EDMA3_CHAN_VPSS_EVT1            4
#define DM36X_EDMA3_CHAN_VPSS_EVT2            5
#define DM36X_EDMA3_CHAN_VPSS_EVT3            6
#define DM36X_EDMA3_CHAN_VPSS_EVT4            7
#define DM36X_EDMA3_CHAN_TIMER2_TEVT4         8
#define DM36X_EDMA3_CHAN_TIMER2_TEVT5         9
#define DM36X_EDMA3_CHAN_SPI2XEVT             10
#define DM36X_EDMA3_CHAN_SPI2REVT             11
#define DM36X_EDMA3_CHAN_MJCP_IMX0INT         12
#define DM36X_EDMA3_CHAN_MJCP_SEQINT          13
#define DM36X_EDMA3_CHAN_SPI1XEVT             14
#define DM36X_EDMA3_CHAN_SPI1REVT             15
#define DM36X_EDMA3_CHAN_SPI0XEVT             16
#define DM36X_EDMA3_CHAN_SPI0REVT             17
#define DM36X_EDMA3_CHAN_URXEVT0              18
#define DM36X_EDMA3_CHAN_UTXEVT0              19
#define DM36X_EDMA3_CHAN_URXEVT1              20
#define DM36X_EDMA3_CHAN_UTXEVT1              21
#define DM36X_EDMA3_CHAN_TIMER4_TEVT8         22
#define DM36X_EDMA3_CHAN_TIMER4_TEVT9         23
#define DM36X_EDMA3_CHAN_RTOEVT               24
#define DM36X_EDMA3_CHAN_GPINT9               25
#define DM36X_EDMA3_CHAN_MMC0RXEVT            26
#define DM36X_EDMA3_CHAN_MMC0TXEVT            27
#define DM36X_EDMA3_CHAN_ICREVT               28
#define DM36X_EDMA3_CHAN_ICXEVT               29
#define DM36X_EDMA3_CHAN_MMC1RXEVT            30
#define DM36X_EDMA3_CHAN_MMC1TXEVT            31
#define DM36X_EDMA3_CHAN_GPINT0               32
#define DM36X_EDMA3_CHAN_GPINT1               33
#define DM36X_EDMA3_CHAN_GPINT2               34
#define DM36X_EDMA3_CHAN_GPINT3               35
#define DM36X_EDMA3_CHAN_GPINT4               36
#define DM36X_EDMA3_CHAN_GPINT5               37
#define DM36X_EDMA3_CHAN_GPINT6               38
#define DM36X_EDMA3_CHAN_GPINT7               39
#define DM36X_EDMA3_CHAN_GPINT10              40
#define DM36X_EDMA3_CHAN_GPINT11              41
#define DM36X_EDMA3_CHAN_GPINT12              42
#define DM36X_EDMA3_CHAN_GPINT13              43
#define DM36X_EDMA3_CHAN_GPINT14              44
#define DM36X_EDMA3_CHAN_GPINT15              45
#define DM36X_EDMA3_CHAN_ADINT                46
#define DM36X_EDMA3_CHAN_GPINT8               47
#define DM36X_EDMA3_CHAN_TIMER0_TEVT0         48
#define DM36X_EDMA3_CHAN_TIMER0_TEVT1         49
#define DM36X_EDMA3_CHAN_TIMER1_TEVT2         50
#define DM36X_EDMA3_CHAN_TIMER1_TEVT3         51
#define DM36X_EDMA3_CHAN_PWM0                 52
#define DM36X_EDMA3_CHAN_PWM1                 53
#define DM36X_EDMA3_CHAN_PWM2                 54
#define DM36X_EDMA3_CHAN_PWM3                 55
#define DM36X_EDMA3_CHAN_MJCP_VLDCINT         56
#define DM36X_EDMA3_CHAN_MJCP_BIMINT          57
#define DM36X_EDMA3_CHAN_MJCP_DCTINT          58
#define DM36X_EDMA3_CHAN_MJCP_QIQINT          59
#define DM36X_EDMA3_CHAN_MJCP_BPSINT          60
#define DM36X_EDMA3_CHAN_MJCP_VLDCERRINT      61
#define DM36X_EDMA3_CHAN_MJCP_RCNTINT         62
#define DM36X_EDMA3_CHAN_MJCP_COPCINT         63


/** System registers */
#define DM36X_SYSTEM_BASE       0x01C40000 /* System base registers */
#define DM36X_SYSTEM_PINMUX0        0x00
#define DM36X_SYSTEM_PINMUX1        0x04
#define DM36X_SYSTEM_PINMUX2        0x08
#define DM36X_SYSTEM_PINMUX3        0x0C
#define DM36X_SYSTEM_PINMUX4        0x10
#define DM36X_SYSTEM_ARM_INTMUX     0x18
#define DM36X_SYSTEM_EDMA_EVTMUX    0x1C
#define DM36X_SYSTEM_PERICLKCTL     0x48
#define DM36X_SYSTEM_PUPDCTL0       0x78
#define DM36X_SYSTEM_PUPDCTL1       0x7C
/* PINMUX0 register bitfields */
#define DM36X_PINMUX0_MMCSD0     24
#define DM36X_PINMUX0_GIO49      23
#define DM36X_PINMUX0_GIO48      22
#define DM36X_PINMUX0_GIO47      21
#define DM36X_PINMUX0_GIO46      20
#define DM36X_PINMUX0_GIO45      19
#define DM36X_PINMUX0_GIO44      18
#define DM36X_PINMUX0_GIO43      16
#define DM36X_PINMUX0_C_WE_FIELD 14
#define DM36X_PINMUX0_VD         13
#define DM36X_PINMUX0_HD         12
#define DM36X_PINMUX0_YIN0       11
#define DM36X_PINMUX0_YIN1       10
#define DM36X_PINMUX0_YIN2       9
#define DM36X_PINMUX0_YIN3       8
#define DM36X_PINMUX0_YIN4       6
#define DM36X_PINMUX0_YIN5       4
#define DM36X_PINMUX0_YIN6       2
#define DM36X_PINMUX0_YIN7       0
/* PINMUX1 register bitfields */
#define DM36X_PINMUX1_VCLK       22
#define DM36X_PINMUX1_EXTCLK     20
#define DM36X_PINMUX1_FIELD      18
#define DM36X_PINMUX1_LCD_OE     17
#define DM36X_PINMUX1_HVSYNC     16
#define DM36X_PINMUX1_COUT0      14
#define DM36X_PINMUX1_COUT1      12
#define DM36X_PINMUX1_COUT2      10
#define DM36X_PINMUX1_COUT3      8
#define DM36X_PINMUX1_COUT4      6
#define DM36X_PINMUX1_COUT5      4
#define DM36X_PINMUX1_COUT6      2
#define DM36X_PINMUX1_COUT7      0
/* PINMUX2 register bitfields */
#define DM36X_PINMUX2_EM_CLK     12
#define DM36X_PINMUX2_EM_ADV     11
#define DM36X_PINMUX2_EM_WAIT    10
#define DM36X_PINMUX2_EM_WE_OE   9
#define DM36X_PINMUX2_EM_CE1     8
#define DM36X_PINMUX2_EM_CE0     7
#define DM36X_PINMUX2_EM_D15_8   6
#define DM36X_PINMUX2_EM_A7      4
#define DM36X_PINMUX2_EM_A3      2
#define DM36X_PINMUX2_EM_AR      0
/* PINMUX3 register bitfields */
#define DM36X_PINMUX3_GIO26      31
#define DM36X_PINMUX3_GIO25      29
#define DM36X_PINMUX3_GIO24      28
#define DM36X_PINMUX3_GIO23      26
#define DM36X_PINMUX3_GIO22      25
#define DM36X_PINMUX3_GIO21      23
#define DM36X_PINMUX3_GIO20      21
#define DM36X_PINMUX3_GIO19      20
#define DM36X_PINMUX3_GIO18      19
#define DM36X_PINMUX3_GIO17      17
#define DM36X_PINMUX3_GIO16      15
#define DM36X_PINMUX3_GIO15      14
#define DM36X_PINMUX3_GIO14      13
#define DM36X_PINMUX3_GIO13      12
#define DM36X_PINMUX3_GIO12      11
#define DM36X_PINMUX3_GIO11      10
#define DM36X_PINMUX3_GIO10      9
#define DM36X_PINMUX3_GIO9       8
#define DM36X_PINMUX3_GIO8       7
#define DM36X_PINMUX3_GIO7       6
#define DM36X_PINMUX3_GIO6       5
#define DM36X_PINMUX3_GIO5       4
#define DM36X_PINMUX3_GIO4       3
#define DM36X_PINMUX3_GIO3       2
#define DM36X_PINMUX3_GIO2       1
#define DM36X_PINMUX3_GIO1       0
/* PINMUX4 register bitfields */
#define DM36X_PINMUX4_GIO42      30
#define DM36X_PINMUX4_GIO41      28
#define DM36X_PINMUX4_GIO40      26
#define DM36X_PINMUX4_GIO39      24
#define DM36X_PINMUX4_GIO38      22
#define DM36X_PINMUX4_GIO37      20
#define DM36X_PINMUX4_GIO36      18
#define DM36X_PINMUX4_GIO35      16
#define DM36X_PINMUX4_GIO34      14
#define DM36X_PINMUX4_GIO33      12
#define DM36X_PINMUX4_GIO32      10
#define DM36X_PINMUX4_GIO31      8
#define DM36X_PINMUX4_GIO30      6
#define DM36X_PINMUX4_GIO29      4
#define DM36X_PINMUX4_GIO28      2
#define DM36X_PINMUX4_GIO27      0


/** PLL registers */
#define DM36X_PLLC1_BASE         0x01C40800 /* PLLC1 base registers */
#define DM36X_PLLC2_BASE         0x01C40C00 /* PLLC2 base registers */
#define DM36X_PLLC_OCSEL         0x104
#define DM36X_PLLC_PLLM          0x110
#define DM36X_PLLC_PREDIV        0x114
#define DM36X_PLLC_PLLDIV1       0x118
#define DM36X_PLLC_PLLDIV2       0x11C
#define DM36X_PLLC_PLLDIV3       0x120
#define DM36X_PLLC_OSCDIV1       0x124
#define DM36X_PLLC_POSTDIV       0x128
#define DM36X_PLLC_CKEN          0x148
#define DM36X_PLLC_CKSTAT        0x14C
#define DM36X_PLLC_PLLDIV4       0x160
#define DM36X_PLLC_PLLDIV5       0x164
#define DM36X_PLLC_PLLDIV6       0x168
#define DM36X_PLLC_PLLDIV7       0x16C
#define DM36X_PLLC_PLLDIV8       0x170
#define DM36X_PLLC_PLLDIV9       0x174


/** Power and sleep controller registers */
#define DM36X_PSC_BASE          0x01C41000 /* PSC registers */
#define DM36X_PSC_EPCPR         0x070
#define DM36X_PSC_PTCMD         0x120
#define DM36X_PSC_PTSTAT        0x128
#define DM36X_PSC_PDSTAT        0x200
#define DM36X_PSC_PDCTL1        0x304
#define DM36X_PSC_MDSTAT_BASE   0x800
#define DM36X_PSC_MDCTL_BASE    0xA00
#define DM36X_PSC_MOD_SPI1      6
#define DM36X_PSC_MOD_MCBSP     8
#define DM36X_PSC_MOD_SPI2      11
#define DM36X_PSC_MOD_I2C       18
#define DM36X_PSC_MOD_SPI0      22
#define DM36X_PSC_MOD_SPI3      38
#define DM36X_PSC_MOD_SPI4      39


/** I2C registers */
#define DM36X_I2C_BASE          0x01C21000 /* I2C */
#define DM36X_I2C_ICOAR         0x00
#define DM36X_I2C_ICIMR         0x04
#define DM36X_I2C_ICSTR         0x08
#define DM36X_I2C_ICCLKL        0x0C
#define DM36X_I2C_ICCLKH        0x10
#define DM36X_I2C_ICCNT         0x14
#define DM36X_I2C_ICDRR         0x18
#define DM36X_I2C_ICSAR         0x1C
#define DM36X_I2C_ICDXR         0x20
#define DM36X_I2C_ICMDR         0x24
#define DM36X_I2C_ICIVR         0x28
#define DM36X_I2C_ICEMDR        0x2C
#define DM36X_I2C_ICPSC         0x30
#define DM36X_I2C_REVID1        0x34
#define DM36X_I2C_REVID2        0x38
#define DM36X_I2C_ICPFUNC       0x48
#define DM36X_I2C_ICPDIR        0x4C
#define DM36X_I2C_ICPDIN        0x50
#define DM36X_I2C_ICPDOUT       0x54
#define DM36X_I2C_ICPDSET       0x58
#define DM36X_I2C_ICPDCLR       0x5C
/** Register bitfields: ICMDR */
#define DM36X_I2C_ICMDR_NACKMOD     (1 << 15)
#define DM36X_I2C_ICMDR_FREE        (1 << 14)
#define DM36X_I2C_ICMDR_STT         (1 << 13)
#define DM36X_I2C_ICMDR_STP         (1 << 11)
#define DM36X_I2C_ICMDR_MST         (1 << 10)
#define DM36X_I2C_ICMDR_TRX         (1 << 9)
#define DM36X_I2C_ICMDR_XA          (1 << 8)
#define DM36X_I2C_ICMDR_RM          (1 << 7)
#define DM36X_I2C_ICMDR_DLB         (1 << 6)
#define DM36X_I2C_ICMDR_IRS         (1 << 5)
#define DM36X_I2C_ICMDR_STB         (1 << 4)
#define DM36X_I2C_ICMDR_FDF         (1 << 3)
#define DM36X_I2C_ICMDR_BC          (1 << 0)
/** Register bitfields: ICSTR */
#define DM36X_I2C_ICSTR_SDIR        (1 << 14)
#define DM36X_I2C_ICSTR_NACKSNT     (1 << 13)
#define DM36X_I2C_ICSTR_BB          (1 << 12)
#define DM36X_I2C_ICSTR_RSFULL      (1 << 11)
#define DM36X_I2C_ICSTR_XSMT        (1 << 10)
#define DM36X_I2C_ICSTR_AAS         (1 << 9)
#define DM36X_I2C_ICSTR_AD0         (1 << 8)
#define DM36X_I2C_ICSTR_SCD         (1 << 5)
#define DM36X_I2C_ICSTR_ICXRDY      (1 << 4)
#define DM36X_I2C_ICSTR_ICRRDY      (1 << 3)
#define DM36X_I2C_ICSTR_ARDY        (1 << 2)
#define DM36X_I2C_ICSTR_NACK        (1 << 1)
#define DM36X_I2C_ICSTR_AL          (1 << 0)


/** Timer registers */
#define DM36X_TIMER0_BASE       0x01C21400 /* TIMER0 */
#define DM36X_TIMER1_BASE       0x01C21800 /* TIMER1 */
#define DM36X_TIMER_PID12       0x00
#define DM36X_TIMER_EMUMGT      0x04
#define DM36X_TIMER_TIM12       0x10
#define DM36X_TIMER_TIM34       0x14
#define DM36X_TIMER_PRD12       0x18
#define DM36X_TIMER_PRD34       0x1C
#define DM36X_TIMER_TCR         0x20
#define DM36X_TIMER_TGCR        0x24
#define DM36X_TIMER_WDTCR       0x28
#define DM36X_TIMER_REL12       0x34
#define DM36X_TIMER_REL34       0x38
#define DM36X_TIMER_CAP12       0x3C
#define DM36X_TIMER_CAP34       0x40
#define DM36X_TIMER_INTCTL_STAT 0x44


/** Interrupt controller registers */
#define DM36X_INTC_BASE         0x01C48000 /* Interrupt controller */
#define DM36X_INTC_IRQ0         0x08
#define DM36X_INTC_IRQ1         0x0C
#define DM36X_INTC_FIQENTRY     0x10
#define DM36X_INTC_IRQENTRY     0x14
#define DM36X_INTC_EINT0        0x18
#define DM36X_INTC_EINT1        0x1C
#define DM36X_INTC_INTCTL       0x20
#define DM36X_INTC_EABASE       0x24
#define DM36X_INTC_PRI0         0x30
#define DM36X_INTC_PRI1         0x34
#define DM36X_INTC_PRI2         0x38
#define DM36X_INTC_PRI3         0x3C
#define DM36X_INTC_PRI4         0x40
#define DM36X_INTC_PRI5         0x44
#define DM36X_INTC_PRI6         0x48
#define DM36X_INTC_PRI7         0x4C
/** Interrupt controller vector offsets */
#define DM36X_INTC_VEC_VPSSINT0      0
#define DM36X_INTC_VEC_VPSSINT1      1
#define DM36X_INTC_VEC_VPSSINT2      2
#define DM36X_INTC_VEC_VPSSINT3      3
#define DM36X_INTC_VEC_VPSSINT4      4
#define DM36X_INTC_VEC_VPSSINT5      5
#define DM36X_INTC_VEC_VPSSINT6      6
#define DM36X_INTC_VEC_VPSSINT7      7
#define DM36X_INTC_VEC_VPSSINT8      8
#define DM36X_INTC_VEC_MJCP_SEQINT   9
#define DM36X_INTC_VEC_HDVICP_INT    10
#define DM36X_INTC_VEC_EDMA_CC_INT0  16
#define DM36X_INTC_VEC_SPI1INT0      17
#define DM36X_INTC_VEC_EDMA_CCERRINT 17
#define DM36X_INTC_VEC_SPI2INT0      19
#define DM36X_INTC_VEC_SDIO0INT      23
#define DM36X_INTC_VEC_MMC0INT       26
#define DM36X_INTC_VEC_MMC1INT       27
#define DM36X_INTC_VEC_SDIO1INT      31
#define DM36X_INTC_VEC_TINT0         32
#define DM36X_INTC_VEC_I2CINT        39
#define DM36X_INTC_VEC_UART0INT      40
#define DM36X_INTC_VEC_UART1INT      41
#define DM36X_INTC_VEC_SPI0INT0      42
#define DM36X_INTC_VEC_SPI3INT0      43
#define DM36X_INTC_VEC_GIO0          44
#define DM36X_INTC_VEC_GIO1          45
#define DM36X_INTC_VEC_GIO2          46
#define DM36X_INTC_VEC_GIO3          47
#define DM36X_INTC_VEC_GIO4          48
#define DM36X_INTC_VEC_GIO5          49
#define DM36X_INTC_VEC_GIO6          50
#define DM36X_INTC_VEC_GIO7          51
#define DM36X_INTC_VEC_GIO8          52
#define DM36X_INTC_VEC_GIO9          53
#define DM36X_INTC_VEC_GIO10         54
#define DM36X_INTC_VEC_GIO11         55
#define DM36X_INTC_VEC_GIO12         56
#define DM36X_INTC_VEC_GIO13         57
#define DM36X_INTC_VEC_GIO14         58
#define DM36X_INTC_VEC_GIO15         59
#define DM36X_INTC_MAX_VEC           63


/** UART registers */
#define DM36X_UART0_BASE        0x01C20000 /* UART0 */
#define DM36X_UART1_BASE        0x01D06000 /* UART1 */


/** SPI registers */
#define DM36X_SPI0_BASE         0x01C66000 /* SPI0 */
#define DM36X_SPI1_BASE         0x01C66800 /* SPI1 */
#define DM36X_SPI2_BASE         0x01C67800 /* SPI2 */
#define DM36X_SPI3_BASE         0x01C68000 /* SPI3 */
#define DM36X_SPI4_BASE         0x01C23000 /* SPI4 */
#define DM36X_SPI_SPIGCR0       0x00
#define DM36X_SPI_SPIGCR1       0x04
#define DM36X_SPI_SPIINT        0x08
#define DM36X_SPI_SPILVL        0x0C
#define DM36X_SPI_SPIFLG        0x10
#define DM36X_SPI_SPIPC0        0x14
#define DM36X_SPI_SPIPC2        0x1C
#define DM36X_SPI_SPIDAT1       0x3C
#define DM36X_SPI_SPIBUF        0x40
#define DM36X_SPI_SPIEMU        0x44
#define DM36X_SPI_SPIDELAY      0x48
#define DM36X_SPI_SPIDEF        0x4C
#define DM36X_SPI_SPIFMT0       0x50
#define DM36X_SPI_INTVECT0      0x60
#define DM36X_SPI_INTVECT1      0x64


/** GPIO registers */
#define DM36X_GPIO_BASE              0x01C67000
#define DM36X_GPIO_BINTEN            0x08
#define DM36X_GPIO_DIR01             0x10
#define DM36X_GPIO_OUT01             0x14
#define DM36X_GPIO_SET01             0x18
#define DM36X_GPIO_CLR01             0x1C
#define DM36X_GPIO_IN01              0x20
#define DM36X_GPIO_SET_RIS_TRIG01    0x24
#define DM36X_GPIO_CLR_RIS_TRIG01    0x28
#define DM36X_GPIO_SET_FAL_TRIG01    0x2C
#define DM36X_GPIO_CLR_FAL_TRIG01    0x30
#define DM36X_GPIO_INTSTAT01         0x34
#define DM36X_GPIO_DIR23             0x38
#define DM36X_GPIO_OUT23             0x3C
#define DM36X_GPIO_SET23             0x40
#define DM36X_GPIO_CLR23             0x44
#define DM36X_GPIO_IN23              0x48
#define DM36X_GPIO_DIR45             0x60
#define DM36X_GPIO_OUT45             0x64
#define DM36X_GPIO_SET45             0x68
#define DM36X_GPIO_CLR45             0x6C
#define DM36X_GPIO_IN45              0x70
#define DM36X_GPIO_DIR6              0x88
#define DM36X_GPIO_OUT6              0x8C
#define DM36X_GPIO_SET6              0x90
#define DM36X_GPIO_CLR6              0x94
#define DM36X_GPIO_IN6               0x98
#define DM36X_GPIO_SET_RIS_TRIG6     0x9C
#define DM36X_GPIO_CLR_RIS_TRIG6     0xA0
#define DM36X_GPIO_SET_FAL_TRIG6     0xA4
#define DM36X_GPIO_CLR_FAL_TRIG6     0xA8
#define DM36X_GPIO_INTSTAT6          0xAC


/** VPFE/VPBE registers */
#define DM36X_ISP_BASE          0x01C70000
#define DM36X_RSZ_BASE          0x01C70400
#define DM36X_IPIPE_BASE        0x01C70800
#define DM36X_ISIF_BASE         0x01C71000
#define DM36X_IPIPEIF_BASE      0x01C71200
#define DM36X_H3A_BASE          0x01C71400
#define DM36X_OSD_BASE          0x01C71C00
#define DM36X_VENC_BASE         0x01C71E00
#define DM36x_GAMMA_R_TBL_3     0x01C7A800
#define DM36x_GAMMA_G_TBL_3     0x01C7B000
#define DM36x_GAMMA_B_TBL_3     0x01C7B800


/** SD/MMC registers */
#define DM36X_SD1_BASE      0x01D00000 /* MMC/SD1 */
#define DM36X_SD0_BASE      0x01D11000 /* MMC/SD0 */
#define DM36X_SD_MMCCTL     0x00
#define DM36X_SD_MMCCLK     0x04
#define DM36X_SD_MMCST0     0x08
#define DM36X_SD_MMCST1     0x0C
#define DM36X_SD_MMCIM      0x10
#define DM36X_SD_MMCTOR     0x14
#define DM36X_SD_MMCTOD     0x18
#define DM36X_SD_MMCBLEN    0x1C
#define DM36X_SD_MMCNBLK    0x20
#define DM36X_SD_MMCNBLC    0x24
#define DM36X_SD_MMCDRR     0x28
#define DM36X_SD_MMCDXR     0x2C
#define DM36X_SD_MMCCMD     0x30
#define DM36X_SD_MMCARGHL   0x34
#define DM36X_SD_MMCRSP01   0x38
#define DM36X_SD_MMCRSP23   0x3C
#define DM36X_SD_MMCRSP45   0x40
#define DM36X_SD_MMCRSP67   0x44
#define DM36X_SD_MMCDRSP    0x48
#define DM36X_SD_MMCCIDX    0X50
#define DM36X_SD_SDIOCTL    0X64
#define DM36X_SD_SDIOST0    0X68
#define DM36X_SD_SDIOIEN    0X6C
#define DM36X_SD_SDIOIST    0X70
#define DM36X_SD_MMCFIFOCTL 0x74


/** McBSP registers */
#define DM36X_MCBSP_BASE    0x01D02000 /* McBSP */
#define DM36X_MCBSP_DRR     0x00
#define DM36X_MCBSP_DXR     0x04
#define DM36X_MCBSP_SPCR    0x08
#define DM36X_MCBSP_RCR     0x0C
#define DM36X_MCBSP_XCR     0x10
#define DM36X_MCBSP_SRGR    0x14
#define DM36X_MCBSP_MCR     0x18
#define DM36X_MCBSP_RCERE0  0x1C
#define DM36X_MCBSP_XCERE0  0x20
#define DM36X_MCBSP_PCR     0x24
#define DM36X_MCBSP_RCERE1  0x28
#define DM36X_MCBSP_XCERE1  0x2C
#define DM36X_MCBSP_RCERE2  0x30
#define DM36X_MCBSP_XCERE2  0x34
#define DM36X_MCBSP_RCERE3  0x38
#define DM36X_MCBSP_XCERE3  0x3C


/* Function prototypes */
extern int              low_level_init (void) ;


#endif /* __DM36X_IO_H__ */
