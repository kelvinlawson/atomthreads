/**
  ******************************************************************************
  * @file stm8s_tim1.h
  * @brief This file contains all functions prototype and macros for the TIM1 peripheral.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM8S_TIM1_H
#define __STM8S_TIM1_H

/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"

/* Exported types ------------------------------------------------------------*/



/** TIM1 Output Compare and PWM modes */

typedef enum
{
  TIM1_OCMODE_TIMING     = ((u8)0x00),
  TIM1_OCMODE_ACTIVE     = ((u8)0x10),
  TIM1_OCMODE_INACTIVE   = ((u8)0x20),
  TIM1_OCMODE_TOGGLE     = ((u8)0x30),
  TIM1_OCMODE_PWM1       = ((u8)0x60),
  TIM1_OCMODE_PWM2       = ((u8)0x70)
}TIM1_OCMode_TypeDef;

#define IS_TIM1_OC_MODE_OK(MODE) (((MODE) ==  TIM1_OCMODE_TIMING) || \
                                  ((MODE) == TIM1_OCMODE_ACTIVE) || \
                                  ((MODE) == TIM1_OCMODE_INACTIVE) || \
                                  ((MODE) == TIM1_OCMODE_TOGGLE)|| \
                                  ((MODE) == TIM1_OCMODE_PWM1) || \
                                  ((MODE) == TIM1_OCMODE_PWM2))

#define IS_TIM1_OCM_OK(MODE)(((MODE) ==  TIM1_OCMODE_TIMING) || \
                             ((MODE) == TIM1_OCMODE_ACTIVE) || \
                             ((MODE) == TIM1_OCMODE_INACTIVE) || \
                             ((MODE) == TIM1_OCMODE_TOGGLE)|| \
                             ((MODE) == TIM1_OCMODE_PWM1) || \
                             ((MODE) == TIM1_OCMODE_PWM2) || \
                             ((MODE) == TIM1_FORCEDACTION_ACTIVE) || \
                             ((MODE) == TIM1_FORCEDACTION_INACTIVE))

/** TIM1 One Pulse Mode */
typedef enum
{
  TIM1_OPMODE_SINGLE                 = ((u8)0x01),
  TIM1_OPMODE_REPETITIVE             = ((u8)0x00)
}TIM1_OPMode_TypeDef;

#define IS_TIM1_OPM_MODE_OK(MODE) (((MODE) == TIM1_OPMODE_SINGLE) || \
                                   ((MODE) == TIM1_OPMODE_REPETITIVE))

/** TIM1 Channel */

typedef enum
{
  TIM1_CHANNEL_1                     = ((u8)0x00),
  TIM1_CHANNEL_2                     = ((u8)0x01),
  TIM1_CHANNEL_3                     = ((u8)0x02),
  TIM1_CHANNEL_4                     = ((u8)0x03)
}TIM1_Channel_TypeDef;


#define IS_TIM1_CHANNEL_OK(CHANNEL) (((CHANNEL) == TIM1_CHANNEL_1) || \
                                     ((CHANNEL) == TIM1_CHANNEL_2) || \
                                     ((CHANNEL) == TIM1_CHANNEL_3) || \
                                     ((CHANNEL) == TIM1_CHANNEL_4))

#define IS_TIM1_PWMI_CHANNEL_OK(CHANNEL) (((CHANNEL) == TIM1_CHANNEL_1) || \
    ((CHANNEL) == TIM1_CHANNEL_2))

#define IS_TIM1_COMPLEMENTARY_CHANNEL_OK(CHANNEL) (((CHANNEL) == TIM1_CHANNEL_1) || \
    ((CHANNEL) == TIM1_CHANNEL_2) || \
    ((CHANNEL) == TIM1_CHANNEL_3))


/** TIM1 Counter Mode */
typedef enum
{
  TIM1_COUNTERMODE_UP                = ((u8)0x00),
  TIM1_COUNTERMODE_DOWN              = ((u8)0x10),
  TIM1_COUNTERMODE_CENTERALIGNED1    = ((u8)0x20),
  TIM1_COUNTERMODE_CENTERALIGNED2    = ((u8)0x40),
  TIM1_COUNTERMODE_CENTERALIGNED3    = ((u8)0x60)
}TIM1_CounterMode_TypeDef;

#define IS_TIM1_COUNTER_MODE_OK(MODE) (((MODE) == TIM1_COUNTERMODE_UP) || \
                                       ((MODE) == TIM1_COUNTERMODE_DOWN) || \
                                       ((MODE) == TIM1_COUNTERMODE_CENTERALIGNED1) || \
                                       ((MODE) == TIM1_COUNTERMODE_CENTERALIGNED2) || \
                                       ((MODE) == TIM1_COUNTERMODE_CENTERALIGNED3))

/** TIM1 Output Compare Polarity */
typedef enum
{
  TIM1_OCPOLARITY_HIGH               = ((u8)0x00),
  TIM1_OCPOLARITY_LOW                = ((u8)0x22)
}TIM1_OCPolarity_TypeDef;

#define IS_TIM1_OC_POLARITY_OK(POLARITY) (((POLARITY) == TIM1_OCPOLARITY_HIGH) || \
    ((POLARITY) == TIM1_OCPOLARITY_LOW))

/** TIM1 Output Compare N Polarity */
typedef enum
{
  TIM1_OCNPOLARITY_HIGH              = ((u8)0x00),
  TIM1_OCNPOLARITY_LOW               = ((u8)0x88)
}TIM1_OCNPolarity_TypeDef;

#define IS_TIM1_OCN_POLARITY_OK(POLARITY) (((POLARITY) == TIM1_OCNPOLARITY_HIGH) || \
    ((POLARITY) == TIM1_OCNPOLARITY_LOW))

/** TIM1 Output Compare states */
typedef enum
{
  TIM1_OUTPUTSTATE_DISABLE           = ((u8)0x00),
  TIM1_OUTPUTSTATE_ENABLE            = ((u8)0x11)
}TIM1_OutputState_TypeDef;

#define IS_TIM1_OUTPUT_STATE_OK(STATE) (((STATE) == TIM1_OUTPUTSTATE_DISABLE) || \
                                        ((STATE) == TIM1_OUTPUTSTATE_ENABLE))

/** TIM1 Output Compare N States */
typedef enum
{
  TIM1_OUTPUTNSTATE_DISABLE = ((u8)0x00),
  TIM1_OUTPUTNSTATE_ENABLE  = ((u8)0x44)
} TIM1_OutputNState_TypeDef;

#define IS_TIM1_OUTPUTN_STATE_OK(STATE) (((STATE) == TIM1_OUTPUTNSTATE_DISABLE) ||\
    ((STATE) == TIM1_OUTPUTNSTATE_ENABLE))

/** TIM1 Break Input enable/disable */
typedef enum
{
  TIM1_BREAK_ENABLE                  = ((u8)0x10),
  TIM1_BREAK_DISABLE                 = ((u8)0x00)
}TIM1_BreakState_TypeDef;
#define IS_TIM1_BREAK_STATE_OK(STATE) (((STATE) == TIM1_BREAK_ENABLE) || \
                                       ((STATE) == TIM1_BREAK_DISABLE))

/** TIM1 Break Polarity */
typedef enum
{
  TIM1_BREAKPOLARITY_LOW             = ((u8)0x00),
  TIM1_BREAKPOLARITY_HIGH            = ((u8)0x20)
}TIM1_BreakPolarity_TypeDef;
#define IS_TIM1_BREAK_POLARITY_OK(POLARITY) (((POLARITY) == TIM1_BREAKPOLARITY_LOW) || \
    ((POLARITY) == TIM1_BREAKPOLARITY_HIGH))

/** TIM1 AOE Bit Set/Reset */
typedef enum
{
  TIM1_AUTOMATICOUTPUT_ENABLE        = ((u8)0x40),
  TIM1_AUTOMATICOUTPUT_DISABLE       = ((u8)0x00)
}TIM1_AutomaticOutput_TypeDef;

#define IS_TIM1_AUTOMATIC_OUTPUT_STATE_OK(STATE) (((STATE) == TIM1_AUTOMATICOUTPUT_ENABLE) || \
    ((STATE) == TIM1_AUTOMATICOUTPUT_DISABLE))

/** TIM1 Lock levels */
typedef enum
{
  TIM1_LOCKLEVEL_OFF                 = ((u8)0x00),
  TIM1_LOCKLEVEL_1                   = ((u8)0x01),
  TIM1_LOCKLEVEL_2                   = ((u8)0x02),
  TIM1_LOCKLEVEL_3                   = ((u8)0x03)
}TIM1_LockLevel_TypeDef;

#define IS_TIM1_LOCK_LEVEL_OK(LEVEL) (((LEVEL) == TIM1_LOCKLEVEL_OFF) || \
                                      ((LEVEL) == TIM1_LOCKLEVEL_1) || \
                                      ((LEVEL) == TIM1_LOCKLEVEL_2) || \
                                      ((LEVEL) == TIM1_LOCKLEVEL_3))

/** TIM1 OSSI: Off-State Selection for Idle mode states */
typedef enum
{
  TIM1_OSSISTATE_ENABLE              = ((u8)0x04),
  TIM1_OSSISTATE_DISABLE             = ((u8)0x00)
}TIM1_OSSIState_TypeDef;

#define IS_TIM1_OSSI_STATE_OK(STATE) (((STATE) == TIM1_OSSISTATE_ENABLE) || \
                                      ((STATE) == TIM1_OSSISTATE_DISABLE))

/** TIM1 Output Compare Idle State */
typedef enum
{
  TIM1_OCIDLESTATE_SET               = ((u8)0x55),
  TIM1_OCIDLESTATE_RESET             = ((u8)0x00)
}TIM1_OCIdleState_TypeDef;

#define IS_TIM1_OCIDLE_STATE_OK(STATE) (((STATE) == TIM1_OCIDLESTATE_SET) || \
                                        ((STATE) == TIM1_OCIDLESTATE_RESET))

/** TIM1 Output Compare N Idle State */
typedef enum
{
  TIM1_OCNIDLESTATE_SET             = ((u8)0x2A),
  TIM1_OCNIDLESTATE_RESET           = ((u8)0x00)
}TIM1_OCNIdleState_TypeDef;

#define IS_TIM1_OCNIDLE_STATE_OK(STATE) (((STATE) == TIM1_OCNIDLESTATE_SET) || \
    ((STATE) == TIM1_OCNIDLESTATE_RESET))

/** TIM1 Input Capture Polarity */
typedef enum
{
  TIM1_ICPOLARITY_RISING            = ((u8)0x00),
  TIM1_ICPOLARITY_FALLING           = ((u8)0x01)
}TIM1_ICPolarity_TypeDef;

#define IS_TIM1_IC_POLARITY_OK(POLARITY) (((POLARITY) == TIM1_ICPOLARITY_RISING) || \
    ((POLARITY) == TIM1_ICPOLARITY_FALLING))

/** TIM1 Input Capture Selection */
typedef enum
{
  TIM1_ICSELECTION_DIRECTTI          = ((u8)0x01),
  TIM1_ICSELECTION_INDIRECTTI        = ((u8)0x02),
  TIM1_ICSELECTION_TRGI              = ((u8)0x03)
}TIM1_ICSelection_TypeDef;

#define IS_TIM1_IC_SELECTION_OK(SELECTION) (((SELECTION) == TIM1_ICSELECTION_DIRECTTI) || \
    ((SELECTION) == TIM1_ICSELECTION_INDIRECTTI) || \
    ((SELECTION) == TIM1_ICSELECTION_TRGI))

/** TIM1 Input Capture Prescaler */
typedef enum
{
  TIM1_ICPSC_DIV1                    = ((u8)0x00),
  TIM1_ICPSC_DIV2                    = ((u8)0x04),
  TIM1_ICPSC_DIV4                    = ((u8)0x08),
  TIM1_ICPSC_DIV8                    = ((u8)0x0C)
}TIM1_ICPSC_TypeDef;

#define IS_TIM1_IC_PRESCALER_OK(PRESCALER) (((PRESCALER) == TIM1_ICPSC_DIV1) || \
    ((PRESCALER) == TIM1_ICPSC_DIV2) || \
    ((PRESCALER) == TIM1_ICPSC_DIV4) || \
    ((PRESCALER) == TIM1_ICPSC_DIV8))

/** TIM1 Input Capture Filer Value */

#define IS_TIM1_IC_FILTER_OK(ICFILTER) (ICFILTER <= 0x0F)

/** TIM1 External Trigger Filer Value */
#define IS_TIM1_EXT_TRG_FILTER_OK(FILTER) (FILTER <= 0x0F)

/** TIM1 interrupt sources */
typedef enum
{
  TIM1_IT_UPDATE                     = ((u8)0x01),
  TIM1_IT_CC1                        = ((u8)0x02),
  TIM1_IT_CC2                        = ((u8)0x04),
  TIM1_IT_CC3                        = ((u8)0x08),
  TIM1_IT_CC4                        = ((u8)0x10),
  TIM1_IT_COM                        = ((u8)0x20),
  TIM1_IT_TRIGGER                    = ((u8)0x40),
  TIM1_IT_BREAK                      = ((u8)0x80)
}TIM1_IT_TypeDef;

#define IS_TIM1_IT_OK(IT) (IT != 0x00)

#define IS_TIM1_GET_IT_OK(IT) (((IT) == TIM1_IT_UPDATE) || \
                               ((IT) == TIM1_IT_CC1) || \
                               ((IT) == TIM1_IT_CC2) || \
                               ((IT) == TIM1_IT_CC3) || \
                               ((IT) == TIM1_IT_CC4) || \
                               ((IT) == TIM1_IT_COM) || \
                               ((IT) == TIM1_IT_TRIGGER) || \
                               ((IT) == TIM1_IT_BREAK))


/** TIM1 External Trigger Prescaler */
typedef enum
{
  TIM1_EXTTRGPSC_OFF                 = ((u8)0x00),
  TIM1_EXTTRGPSC_DIV2                = ((u8)0x10),
  TIM1_EXTTRGPSC_DIV4                = ((u8)0x20),
  TIM1_EXTTRGPSC_DIV8                = ((u8)0x30)
}TIM1_ExtTRGPSC_TypeDef;

#define IS_TIM1_EXT_PRESCALER_OK(PRESCALER) (((PRESCALER) == TIM1_EXTTRGPSC_OFF) || \
    ((PRESCALER) == TIM1_EXTTRGPSC_DIV2) || \
    ((PRESCALER) == TIM1_EXTTRGPSC_DIV4) || \
    ((PRESCALER) == TIM1_EXTTRGPSC_DIV8))

/** TIM1 Internal Trigger Selection */
typedef enum
{
  TIM1_TS_TIM6	                     = ((u8)0x00),  /*!< TRIG Input source =  TIM6 TRIG Output  */
	TIM1_TS_TIM5	                     = ((u8)0x30),  /*!< TRIG Input source =  TIM5 TRIG Output  */
  TIM1_TS_TI1F_ED                    = ((u8)0x40),
  TIM1_TS_TI1FP1                     = ((u8)0x50),
  TIM1_TS_TI2FP2                     = ((u8)0x60),
  TIM1_TS_ETRF                       = ((u8)0x70)
}TIM1_TS_TypeDef;

#define IS_TIM1_TRIGGER_SELECTION_OK(SELECTION) (((SELECTION) == TIM1_TS_TI1F_ED) || \
    ((SELECTION) == TIM1_TS_TI1FP1) || \
    ((SELECTION) == TIM1_TS_TI2FP2) || \
	  ((SELECTION) == TIM1_TS_ETRF) || \
    ((SELECTION) == TIM1_TS_TIM5) || \
    ((SELECTION) == TIM1_TS_TIM6))


#define IS_TIM1_TIX_TRIGGER_SELECTION_OK(SELECTION) (((SELECTION) == TIM1_TS_TI1F_ED) || \
    ((SELECTION) == TIM1_TS_TI1FP1) || \
    ((SELECTION) == TIM1_TS_TI2FP2))

/** TIM1 TIx External Clock Source */
typedef enum
{
  TIM1_TIXEXTERNALCLK1SOURCE_TI1ED   = ((u8)0x40),
  TIM1_TIXEXTERNALCLK1SOURCE_TI1     = ((u8)0x50),
  TIM1_TIXEXTERNALCLK1SOURCE_TI2     = ((u8)0x60)
}TIM1_TIxExternalCLK1Source_TypeDef;

#define IS_TIM1_TIXCLK_SOURCE_OK(SOURCE)  (((SOURCE) == TIM1_TIXEXTERNALCLK1SOURCE_TI1ED) || \
    ((SOURCE) == TIM1_TIXEXTERNALCLK1SOURCE_TI2) || \
    ((SOURCE) == TIM1_TIXEXTERNALCLK1SOURCE_TI1))

/** TIM1 External Trigger Polarity */
typedef enum
{
  TIM1_EXTTRGPOLARITY_INVERTED       = ((u8)0x80),
  TIM1_EXTTRGPOLARITY_NONINVERTED    = ((u8)0x00)
}TIM1_ExtTRGPolarity_TypeDef;

#define IS_TIM1_EXT_POLARITY_OK(POLARITY) (((POLARITY) == TIM1_EXTTRGPOLARITY_INVERTED) || \
    ((POLARITY) == TIM1_EXTTRGPOLARITY_NONINVERTED))

/** TIM1 Prescaler Reload Mode */
typedef enum
{
  TIM1_PSCRELOADMODE_UPDATE          = ((u8)0x00),
  TIM1_PSCRELOADMODE_IMMEDIATE       = ((u8)0x01)
}TIM1_PSCReloadMode_TypeDef;

#define IS_TIM1_PRESCALER_RELOAD_OK(RELOAD) (((RELOAD) == TIM1_PSCRELOADMODE_UPDATE) || \
    ((RELOAD) == TIM1_PSCRELOADMODE_IMMEDIATE))

/** TIM1 Encoder Mode */
typedef enum
{
  TIM1_ENCODERMODE_TI1               = ((u8)0x01),
  TIM1_ENCODERMODE_TI2               = ((u8)0x02),
  TIM1_ENCODERMODE_TI12              = ((u8)0x03)
}TIM1_EncoderMode_TypeDef;

#define IS_TIM1_ENCODER_MODE_OK(MODE) (((MODE) == TIM1_ENCODERMODE_TI1) || \
                                       ((MODE) == TIM1_ENCODERMODE_TI2) || \
                                       ((MODE) == TIM1_ENCODERMODE_TI12))

/** TIM1 Event Source */
typedef enum
{
  TIM1_EVENTSOURCE_UPDATE            = ((u8)0x01),
  TIM1_EVENTSOURCE_CC1               = ((u8)0x02),
  TIM1_EVENTSOURCE_CC2               = ((u8)0x04),
  TIM1_EVENTSOURCE_CC3               = ((u8)0x08),
  TIM1_EVENTSOURCE_CC4               = ((u8)0x10),
  TIM1_EVENTSOURCE_COM               = ((u8)0x20),
  TIM1_EVENTSOURCE_TRIGGER           = ((u8)0x40),
  TIM1_EVENTSOURCE_BREAK             = ((u8)0x80)
}TIM1_EventSource_TypeDef;

#define IS_TIM1_EVENT_SOURCE_OK(SOURCE) ((SOURCE) != 0x00)

/** TIM1 Update Source */
typedef enum
{
  TIM1_UPDATESOURCE_GLOBAL           = ((u8)0x00),
  TIM1_UPDATESOURCE_REGULAR          = ((u8)0x01)
}TIM1_UpdateSource_TypeDef;

#define IS_TIM1_UPDATE_SOURCE_OK(SOURCE) (((SOURCE) == TIM1_UPDATESOURCE_GLOBAL) || \
    ((SOURCE) == TIM1_UPDATESOURCE_REGULAR))

/** TIM1 Trigger Output Source */
typedef enum
{
  TIM1_TRGOSOURCE_RESET              = ((u8)0x00),
  TIM1_TRGOSOURCE_ENABLE             = ((u8)0x10),
  TIM1_TRGOSOURCE_UPDATE             = ((u8)0x20),
  TIM1_TRGOSource_OC1                = ((u8)0x30),
  TIM1_TRGOSOURCE_OC1REF             = ((u8)0x40),
  TIM1_TRGOSOURCE_OC2REF             = ((u8)0x50),
  TIM1_TRGOSOURCE_OC3REF             = ((u8)0x60)
}TIM1_TRGOSource_TypeDef;

#define IS_TIM1_TRGO_SOURCE_OK(SOURCE) (((SOURCE) == TIM1_TRGOSOURCE_RESET) || \
                                        ((SOURCE) == TIM1_TRGOSOURCE_ENABLE) || \
                                        ((SOURCE) == TIM1_TRGOSOURCE_UPDATE) || \
                                        ((SOURCE) == TIM1_TRGOSource_OC1)  || \
                                        ((SOURCE) == TIM1_TRGOSOURCE_OC1REF) || \
                                        ((SOURCE) == TIM1_TRGOSOURCE_OC2REF) || \
                                        ((SOURCE) == TIM1_TRGOSOURCE_OC3REF))

/** TIM1 Slave Mode */
typedef enum
{
  TIM1_SLAVEMODE_RESET               = ((u8)0x04),
  TIM1_SLAVEMODE_GATED               = ((u8)0x05),
  TIM1_SLAVEMODE_TRIGGER             = ((u8)0x06),
  TIM1_SLAVEMODE_EXTERNAL1           = ((u8)0x07)
}TIM1_SlaveMode_TypeDef;

#define IS_TIM1_SLAVE_MODE_OK(MODE) (((MODE) == TIM1_SLAVEMODE_RESET) || \
                                     ((MODE) == TIM1_SLAVEMODE_GATED) || \
                                     ((MODE) == TIM1_SLAVEMODE_TRIGGER) || \
                                     ((MODE) == TIM1_SLAVEMODE_EXTERNAL1))

/** TIM1 Flags */
typedef enum
{
  TIM1_FLAG_UPDATE                   = ((u16)0x0001),
  TIM1_FLAG_CC1                      = ((u16)0x0002),
  TIM1_FLAG_CC2                      = ((u16)0x0004),
  TIM1_FLAG_CC3                      = ((u16)0x0008),
  TIM1_FLAG_CC4                      = ((u16)0x0010),
  TIM1_FLAG_COM                      = ((u16)0x0020),
  TIM1_FLAG_TRIGGER                  = ((u16)0x0040),
  TIM1_FLAG_BREAK                    = ((u16)0x0080),
  TIM1_FLAG_CC1OF                    = ((u16)0x0200),
  TIM1_FLAG_CC2OF                    = ((u16)0x0400),
  TIM1_FLAG_CC3OF                    = ((u16)0x0800),
  TIM1_FLAG_CC4OF                    = ((u16)0x1000)
}TIM1_FLAG_TypeDef;

#define IS_TIM1_GET_FLAG_OK(FLAG) (((FLAG) == TIM1_FLAG_UPDATE) || \
                                   ((FLAG) == TIM1_FLAG_CC1) || \
                                   ((FLAG) == TIM1_FLAG_CC2) || \
                                   ((FLAG) == TIM1_FLAG_CC3) || \
                                   ((FLAG) == TIM1_FLAG_CC4) || \
                                   ((FLAG) == TIM1_FLAG_COM) || \
                                   ((FLAG) == TIM1_FLAG_TRIGGER) || \
                                   ((FLAG) == TIM1_FLAG_BREAK) || \
                                   ((FLAG) == TIM1_FLAG_CC1OF) || \
                                   ((FLAG) == TIM1_FLAG_CC2OF) || \
                                   ((FLAG) == TIM1_FLAG_CC3OF) || \
                                   ((FLAG) == TIM1_FLAG_CC4OF))

#define IS_TIM1_CLEAR_FLAG_OK(FLAG) ((((u16)FLAG & (u16)0xE100) == 0x0000) && (FLAG != 0x0000))

/** TIM1 Forced Action */
typedef enum
{
  TIM1_FORCEDACTION_ACTIVE           = ((u8)0x50),
  TIM1_FORCEDACTION_INACTIVE         = ((u8)0x40)
}TIM1_ForcedAction_TypeDef;

#define IS_TIM1_FORCED_ACTION_OK(ACTION) ((ACTION == TIM1_FORCEDACTION_ACTIVE) || \
    (ACTION == TIM1_FORCEDACTION_INACTIVE))
/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

/** @addtogroup TIM1_Exported_Functions
  * @{
  */

void TIM1_DeInit(void);
void TIM1_TimeBaseInit(u16 TIM1_Prescaler, TIM1_CounterMode_TypeDef TIM1_CounterMode, u16 TIM1_Period, u8 TIM1_RepetitionCounter);
void TIM1_OC1Init(TIM1_OCMode_TypeDef TIM1_OCMode, TIM1_OutputState_TypeDef TIM1_OutputState, TIM1_OutputNState_TypeDef TIM1_OutputNState, u16 TIM1_Pulse, TIM1_OCPolarity_TypeDef TIM1_OCPolarity, TIM1_OCNPolarity_TypeDef TIM1_OCNPolarity, TIM1_OCIdleState_TypeDef TIM1_OCIdleState, TIM1_OCNIdleState_TypeDef TIM1_OCNIdleState);
void TIM1_OC2Init(TIM1_OCMode_TypeDef TIM1_OCMode, TIM1_OutputState_TypeDef TIM1_OutputState, TIM1_OutputNState_TypeDef TIM1_OutputNState, u16 TIM1_Pulse, TIM1_OCPolarity_TypeDef TIM1_OCPolarity, TIM1_OCNPolarity_TypeDef TIM1_OCNPolarity, TIM1_OCIdleState_TypeDef TIM1_OCIdleState, TIM1_OCNIdleState_TypeDef TIM1_OCNIdleState);
void TIM1_OC3Init(TIM1_OCMode_TypeDef TIM1_OCMode, TIM1_OutputState_TypeDef TIM1_OutputState, TIM1_OutputNState_TypeDef TIM1_OutputNState, u16 TIM1_Pulse, TIM1_OCPolarity_TypeDef TIM1_OCPolarity, TIM1_OCNPolarity_TypeDef TIM1_OCNPolarity, TIM1_OCIdleState_TypeDef TIM1_OCIdleState, TIM1_OCNIdleState_TypeDef TIM1_OCNIdleState);
void TIM1_OC4Init(TIM1_OCMode_TypeDef TIM1_OCMode, TIM1_OutputState_TypeDef TIM1_OutputState, u16 TIM1_Pulse, TIM1_OCPolarity_TypeDef TIM1_OCPolarity, TIM1_OCIdleState_TypeDef TIM1_OCIdleState);
void TIM1_BDTRConfig(TIM1_OSSIState_TypeDef TIM1_OSSIState, TIM1_LockLevel_TypeDef TIM1_LockLevel, u8 TIM1_DeadTime, TIM1_BreakState_TypeDef TIM1_Break, TIM1_BreakPolarity_TypeDef TIM1_BreakPolarity, TIM1_AutomaticOutput_TypeDef TIM1_AutomaticOutput);
void TIM1_ICInit(TIM1_Channel_TypeDef TIM1_Channel, TIM1_ICPolarity_TypeDef TIM1_ICPolarity, TIM1_ICSelection_TypeDef TIM1_ICSelection, TIM1_ICPSC_TypeDef TIM1_ICPrescaler, u8 TIM1_ICFilter);
void TIM1_PWMIConfig(TIM1_Channel_TypeDef TIM1_Channel, TIM1_ICPolarity_TypeDef TIM1_ICPolarity, TIM1_ICSelection_TypeDef TIM1_ICSelection, TIM1_ICPSC_TypeDef TIM1_ICPrescaler, u8 TIM1_ICFilter);
void TIM1_Cmd(FunctionalState NewState);
void TIM1_CtrlPWMOutputs(FunctionalState Newstate);
void TIM1_ITConfig(TIM1_IT_TypeDef TIM1_IT, FunctionalState NewState);
void TIM1_InternalClockConfig(void);
void TIM1_ETRClockMode1Config(TIM1_ExtTRGPSC_TypeDef TIM1_ExtTRGPrescaler, TIM1_ExtTRGPolarity_TypeDef TIM1_ExtTRGPolarity, u8 ExtTRGFilter);
void TIM1_ETRClockMode2Config(TIM1_ExtTRGPSC_TypeDef TIM1_ExtTRGPrescaler, TIM1_ExtTRGPolarity_TypeDef TIM1_ExtTRGPolarity, u8 ExtTRGFilter);
void TIM1_ETRConfig(TIM1_ExtTRGPSC_TypeDef TIM1_ExtTRGPrescaler, TIM1_ExtTRGPolarity_TypeDef TIM1_ExtTRGPolarity, u8 ExtTRGFilter);
void TIM1_TIxExternalClockConfig(TIM1_TIxExternalCLK1Source_TypeDef TIM1_TIxExternalCLKSource, TIM1_ICPolarity_TypeDef TIM1_ICPolarity, u8 ICFilter);
void TIM1_SelectInputTrigger(TIM1_TS_TypeDef TIM1_InputTriggerSource);
void TIM1_UpdateDisableConfig(FunctionalState Newstate);
void TIM1_UpdateRequestConfig(TIM1_UpdateSource_TypeDef TIM1_UpdateSource);
void TIM1_SelectHallSensor(FunctionalState Newstate);
void TIM1_SelectOnePulseMode(TIM1_OPMode_TypeDef TIM1_OPMode);
void TIM1_SelectOutputTrigger(TIM1_TRGOSource_TypeDef TIM1_TRGOSource);
void TIM1_SelectSlaveMode(TIM1_SlaveMode_TypeDef TIM1_SlaveMode);
void TIM1_SelectMasterSlaveMode(FunctionalState NewState);
void TIM1_EncoderInterfaceConfig(TIM1_EncoderMode_TypeDef TIM1_EncoderMode, TIM1_ICPolarity_TypeDef TIM1_IC1Polarity, TIM1_ICPolarity_TypeDef TIM1_IC2Polarity);
void TIM1_PrescalerConfig(u16 Prescaler, TIM1_PSCReloadMode_TypeDef TIM1_PSCReloadMode);
void TIM1_CounterModeConfig(TIM1_CounterMode_TypeDef TIM1_CounterMode);
void TIM1_ForcedOC1Config(TIM1_ForcedAction_TypeDef TIM1_ForcedAction);
void TIM1_ForcedOC2Config(TIM1_ForcedAction_TypeDef TIM1_ForcedAction);
void TIM1_ForcedOC3Config(TIM1_ForcedAction_TypeDef TIM1_ForcedAction);
void TIM1_ForcedOC4Config(TIM1_ForcedAction_TypeDef TIM1_ForcedAction);
void TIM1_ARRPreloadConfig(FunctionalState Newstate);
void TIM1_SelectCOM(FunctionalState Newstate);
void TIM1_CCPreloadControl(FunctionalState Newstate);
void TIM1_OC1PreloadConfig(FunctionalState Newstate);
void TIM1_OC2PreloadConfig(FunctionalState Newstate);
void TIM1_OC3PreloadConfig(FunctionalState Newstate);
void TIM1_OC4PreloadConfig(FunctionalState Newstate);
void TIM1_OC1FastConfig(FunctionalState Newstate);
void TIM1_OC2FastConfig(FunctionalState Newstate);
void TIM1_OC3FastConfig(FunctionalState Newstate);
void TIM1_OC4FastConfig(FunctionalState Newstate);
void TIM1_GenerateEvent(TIM1_EventSource_TypeDef TIM1_EventSource);
void TIM1_OC1PolarityConfig(TIM1_OCPolarity_TypeDef TIM1_OCPolarity);
void TIM1_OC1NPolarityConfig(TIM1_OCNPolarity_TypeDef TIM1_OCNPolarity);
void TIM1_OC2PolarityConfig(TIM1_OCPolarity_TypeDef TIM1_OCPolarity);
void TIM1_OC2NPolarityConfig(TIM1_OCNPolarity_TypeDef TIM1_OCNPolarity);
void TIM1_OC3PolarityConfig(TIM1_OCPolarity_TypeDef TIM1_OCPolarity);
void TIM1_OC3NPolarityConfig(TIM1_OCNPolarity_TypeDef TIM1_OCNPolarity);
void TIM1_OC4PolarityConfig(TIM1_OCPolarity_TypeDef TIM1_OCPolarity);
void TIM1_CCxCmd(TIM1_Channel_TypeDef TIM1_Channel, FunctionalState Newstate);
void TIM1_CCxNCmd(TIM1_Channel_TypeDef TIM1_Channel, FunctionalState Newstate);
void TIM1_SelectOCxM(TIM1_Channel_TypeDef TIM1_Channel, TIM1_OCMode_TypeDef TIM1_OCMode);
void TIM1_SetCounter(u16 Counter);
void TIM1_SetAutoreload(u16 Autoreload);
void TIM1_SetCompare1(u16 Compare1);
void TIM1_SetCompare2(u16 Compare2);
void TIM1_SetCompare3(u16 Compare3);
void TIM1_SetCompare4(u16 Compare4);
void TIM1_SetIC1Prescaler(TIM1_ICPSC_TypeDef TIM1_IC1Prescaler);
void TIM1_SetIC2Prescaler(TIM1_ICPSC_TypeDef TIM1_IC2Prescaler);
void TIM1_SetIC3Prescaler(TIM1_ICPSC_TypeDef TIM1_IC3Prescaler);
void TIM1_SetIC4Prescaler(TIM1_ICPSC_TypeDef TIM1_IC4Prescaler);
u16 TIM1_GetCapture1(void);
u16 TIM1_GetCapture2(void);
u16 TIM1_GetCapture3(void);
u16 TIM1_GetCapture4(void);
u16 TIM1_GetCounter(void);
u16 TIM1_GetPrescaler(void);
FlagStatus TIM1_GetFlagStatus(TIM1_FLAG_TypeDef TIM1_FLAG);
void TIM1_ClearFlag(TIM1_FLAG_TypeDef TIM1_FLAG);
ITStatus TIM1_GetITStatus(TIM1_IT_TypeDef TIM1_IT);
void TIM1_ClearITPendingBit(TIM1_IT_TypeDef TIM1_IT);

/**
  * @}
  */

#endif /* __STM8S_TIM1_H */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
