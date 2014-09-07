/**************************************************************************//**
 * @file lesense_lcsense_accumulated.c
 * @brief LESENSE LCSENSE code example
 * @author Silicon Labs
 * @version 1.07
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Labs has no
 * obligation to support this Software. Silicon Labs is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Silicon Labs will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include "em_device.h"
#include "em_acmp.h"
#include "em_cmu.h"
#include "em_dac.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_int.h"
#include "em_lesense.h"
#include "em_pcnt.h"
#include "em_prs.h"
#include "em_rtc.h"
#include "lesense_lcsense_calibration.h"

/**************************************************************************//**
 * Macro definitions
 *****************************************************************************/
/* LESENSE */
#define LCSENSE_CH             7
#define LCSENSE_CH_PORT        gpioPortC
#define LCSENSE_CH_PIN         7
#define LCSENSE_SCAN_FREQ      20

/* ACMP */
#define ACMP_NEG_REF           acmpChannelVDD
#define ACMP_VDD_SCALE         0x0D                         /* reference for the LC sensor to be
                                                             * close to the DAC voltage
                                                             * This was calibrated using a scope
                                                             * and probing both the LC sensor and
                                                             * the ACMP output */

/* USER LED */
#define LED_GPIO_PORT          gpioPortE
#define LED_GPIO_PIN           2
#define USER_LED_SECONDS_ON    3

/* RTC */
#define RTC_FREQ               32768
#define RTC_COMP_VALUE         (RTC_FREQ * USER_LED_SECONDS_ON)

/* DAC */
#define DAC_FREQ               500000
#define DAC_CHANNEL            1
#define DAC_DATA               800

/* PRS */
#define PRS_CHANNEL            0

/* PCNT */
#define PCNT_TOP               4

/**************************************************************************//**
 * Interrupt handlers prototypes
 *****************************************************************************/
void PCNT0_IRQHandler(void);
void RTC_IRQHandler(void);

/**************************************************************************//**
 * Functions prototypes
 *****************************************************************************/
void setupCMU(void);
void writeDataDAC(DAC_TypeDef *dac, unsigned int value, unsigned int ch);
void setupDAC(void);
void setupACMP(void);
void setupLESENSE(void);
void setupGPIO(void);
void setupPRS(void);
void setupPCNT(void);
void setupRTC(void);

/**************************************************************************//**
 * Interrupt handlers
 *****************************************************************************/
/**************************************************************************//**
 * @brief PCNT_IRQHandler
 * Interrupt Service Routine for Pulse Counter Interrupt Line
 *****************************************************************************/
void PCNT0_IRQHandler(void)
{
  /* Clear interrupt flag */
  PCNT_IntClear(PCNT0, PCNT_IEN_OF);

  /* Disable RTC first to reset counter */
  RTC_Enable(false);
  /* Set compare value */
  RTC_CompareSet(0, RTC_COMP_VALUE);
  /* Enable RTC */
  RTC_Enable(true);

  /* Turn on user led */
  GPIO_PinOutSet(LED_GPIO_PORT, LED_GPIO_PIN);
}

/**************************************************************************//**
 * @brief RTC_IRQHandler
 * Interrupt Service Routine for RTC Interrupt Line
 *****************************************************************************/
void RTC_IRQHandler(void)
{
  /* Clear interrupt flag */
  RTC_IntClear(RTC_IFS_COMP0);
  /* Disable RTC */
  RTC_Enable(false);

  /* Turn off user led */
  GPIO_PinOutClear(LED_GPIO_PORT, LED_GPIO_PIN);
}

/**************************************************************************//**
 * Functions
 *****************************************************************************/
/**************************************************************************//**
 * @brief  Enable clocks for all the peripherals to be used
 *****************************************************************************/
void setupCMU(void)
{
  /* Ensure core frequency has been updated */
  SystemCoreClockUpdate();

  /* DAC */
  CMU_ClockEnable(cmuClock_DAC0, true);

  /* ACMP */
  CMU_ClockEnable(cmuClock_ACMP0, true);

  /* GPIO */
  CMU_ClockEnable(cmuClock_GPIO, true);

  /* Low energy peripherals
   * LESENSE
   * LFRCO clock must be enables prior to enabling
   * clock for the low energy peripherals */
  CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFRCO);
  CMU_ClockEnable(cmuClock_CORELE, true);
  CMU_ClockEnable(cmuClock_LESENSE, true);

  /* RTC */
  CMU_ClockEnable(cmuClock_RTC, true);

  /* PRS */
  CMU_ClockEnable(cmuClock_PRS, true);

  /* PCNT */
  CMU_ClockEnable(cmuClock_PCNT0, true);
}

/**************************************************************************//**
 * @brief  Sets up the DAC
 *****************************************************************************/
void setupDAC(void)
{
  /* Configuration structure for the DAC */
  static const DAC_Init_TypeDef dacInit =
  {
    .refresh      = dacRefresh8,
    .reference    = dacRefVDD,
    .outMode      = dacOutputPin,
    .convMode     = dacConvModeContinuous,
    .prescale     = 0,
    .lpEnable     = false,
    .ch0ResetPre  = false,
    .outEnablePRS = false,
    .sineEnable   = false,
    .diff         = false
  };

  /* Initialize DAC */
  DAC_Init(DAC0, &dacInit);

  /* Set data for DAC channel 0 */
  writeDataDAC(DAC0, (unsigned int) DAC_DATA, DAC_CHANNEL);
}

/**************************************************************************//**
 * @brief  Write DAC conversion value
 *****************************************************************************/
void writeDataDAC(DAC_TypeDef *dac, unsigned int value, unsigned int ch)
{
  /* Write data output value to the correct register. */
  if (!ch)
  {
    /* Write data to DAC ch 0 */
    dac->CH0DATA = value;
  }
  else
  {
    /* Write data to DAC ch 1 */
    dac->CH1DATA = value;
  }
}

/**************************************************************************//**
 * @brief  Sets up the ACMP
 *****************************************************************************/
void setupACMP(void)
{
  /* Configuration structure for ACMP */
  static const ACMP_Init_TypeDef acmpInit =
  {
    .fullBias                 = false,
    .halfBias                 = true,
    .biasProg                 = 0xE,
    .interruptOnFallingEdge   = false,
    .interruptOnRisingEdge    = false,
    .warmTime                 = acmpWarmTime4,
    .hysteresisLevel          = acmpHysteresisLevel0,
    .inactiveValue            = false,
    .lowPowerReferenceEnabled = false,
    .vddLevel                 = ACMP_VDD_SCALE,
    .enable                   = false
  };

  /* Initialize ACMP */
  ACMP_Init(ACMP0, &acmpInit);

  /* Select Vdd as negative reference
   * Positive reference is controlled by LESENSE */
  ACMP_ChannelSet(ACMP0, acmpChannelVDD, acmpChannel7);
}

/**************************************************************************//**
 * @brief  Sets up the LESENSE
 *****************************************************************************/
void setupLESENSE(void)
{
  static const LESENSE_Init_TypeDef initLesense =
  {
    .coreCtrl         =
    {
      .scanStart    = lesenseScanStartPeriodic,
      .prsSel       = lesensePRSCh0,
      .scanConfSel  = lesenseScanConfDirMap,
      .invACMP0     = false,
      .invACMP1     = false,
      .dualSample   = false,
      .storeScanRes = false,
      .bufOverWr    = true,
      .bufTrigLevel = lesenseBufTrigHalf,
      .wakeupOnDMA  = lesenseDMAWakeUpDisable,
      .biasMode     = lesenseBiasModeDutyCycle,
      .debugRun     = false
    },

    .timeCtrl         =
    {
      .startDelay     = 0
    },

    .perCtrl          =
    {
      .dacCh0Data     = lesenseDACIfData,
      .dacCh0ConvMode = lesenseDACConvModeSampleOff,
      .dacCh0OutMode  = lesenseDACOutModeDisable,
      .dacCh1Data     = lesenseDACIfData,
      .dacCh1ConvMode = lesenseDACConvModeSampleOff,
      .dacCh1OutMode  = lesenseDACOutModePin,
      .dacPresc       = 31,
      .dacRef         = lesenseDACRefVdd,
      .acmp0Mode      = lesenseACMPModeMux,
      .acmp1Mode      = lesenseACMPModeDisable,
      .warmupMode     = lesenseWarmupModeNormal
    },

    .decCtrl          =
    {
      .decInput  = lesenseDecInputSensorSt,
      .initState = 0,
      .chkState  = false,
      .intMap    = false,
      .hystPRS0  = false,
      .hystPRS1  = false,
      .hystPRS2  = false,
      .hystIRQ   = false,
      .prsCount  = true,
      .prsChSel0 = lesensePRSCh0,
      .prsChSel1 = lesensePRSCh1,
      .prsChSel2 = lesensePRSCh2,
      .prsChSel3 = lesensePRSCh3
    }
  };

  /* Channel configuration */
  static const LESENSE_ChDesc_TypeDef initLesenseCh7 =
  {
    .enaScanCh     = true,
    .enaPin        = true,
    .enaInt        = false,
    .chPinExMode   = lesenseChPinExLow,
    .chPinIdleMode = lesenseChPinIdleDis,
    .useAltEx      = false,
    .shiftRes      = false,
    .invRes        = false,
    .storeCntRes   = true,
    .exClk         = lesenseClkHF,
    .sampleClk     = lesenseClkLF,
    .exTime        = 0x07,
    .sampleDelay   = 0x01,
    .measDelay     = 0x00,
    .acmpThres     = 0x00,
    .sampleMode    = lesenseSampleModeCounter,
    .intMode       = lesenseSetIntPosEdge,
    .cntThres      = 0x0000,
    .compMode      = lesenseCompModeLess
  };

  /* Initialize LESENSE interface with RESET. */
  LESENSE_Init(&initLesense, true);

  /* Configure channel 7 */
  LESENSE_ChannelConfig(&initLesenseCh7, LCSENSE_CH);

  /* Set scan frequency */
  LESENSE_ScanFreqSet(0, LCSENSE_SCAN_FREQ);

  /* Set clock divisor for LF clock. */
  LESENSE_ClkDivSet(lesenseClkLF, lesenseClkDiv_2);
  /* Set clock divisor for HF clock. */
  LESENSE_ClkDivSet(lesenseClkHF, lesenseClkDiv_1);

  /* Calibrate LC sensor */
  lesenseCalibrateLC(LCSENSE_CH);

  /* Start scan. */
  LESENSE_ScanStart();
}

/**************************************************************************//**
 * @brief  Sets up the GPIO
 *****************************************************************************/
void setupGPIO(void)
{
  /* Configure excitation/measure pin as push pull */
  GPIO_PinModeSet(LCSENSE_CH_PORT, LCSENSE_CH_PIN, gpioModePushPull, 0);

  /* Configure user led as output */
  GPIO_PinModeSet(LED_GPIO_PORT, LED_GPIO_PIN, gpioModePushPull, 0);
}

/**************************************************************************//**
 * @brief  Sets up the PRS
 *****************************************************************************/
void setupPRS(void)
{
  /* PRS channel 0 configuration */
  /* LESENSE SCANRES bit 7 will be used as PRS signal */
  PRS_SourceAsyncSignalSet(PRS_CHANNEL, PRS_CH_CTRL_SOURCESEL_LESENSEL, PRS_CH_CTRL_SIGSEL_LESENSESCANRES7);
}

/**************************************************************************//**
 * @brief  Sets up the PCNT
 *****************************************************************************/
void setupPCNT(void)
{
  /* PCNT configuration structure */
  static const PCNT_Init_TypeDef initPCNT =
  {
    .mode        = pcntModeOvsSingle,
    .counter     = 0,
    .top         = PCNT_TOP,
    .negEdge     = false,
    .countDown   = false,
    .filter      = false,
    .hyst        = false,
    .s1CntDir    = false,
    .cntEvent    = pcntCntEventUp,
    .auxCntEvent = pcntCntEventNone,
    .s0PRS       = pcntPRSCh0,
    .s1PRS       = pcntPRSCh0
  };


  /* Initialize PCNT. */
  PCNT_Init(PCNT0, &initPCNT);
  /* Enable PRS input S0 in PCNT. */
  PCNT_PRSInputEnable(PCNT0, pcntPRSInputS0, true);

  /* Enable the PCNT peripheral. */
  PCNT_Enable(PCNT0, pcntModeOvsSingle);

  /* Enable the PCNT overflow interrupt. */
  PCNT_IntEnable(PCNT0, PCNT_IEN_OF);

  /* Enable the PCNT vector in NVIC */
  NVIC_EnableIRQ(PCNT0_IRQn);
}

/**************************************************************************//**
 * @brief  Sets up the RTC
 *****************************************************************************/
void setupRTC(void)
{
  /* RTC configuration */
  static const RTC_Init_TypeDef rtcInit =
  {
    .enable   = false,
    .debugRun = false,
    .comp0Top = true
  };

  RTC_Init(&rtcInit);

  /* Enable interrupts */
  RTC_IntEnable(RTC_IFS_COMP0);
  /* Enable RTC vector in NVIC */
  NVIC_EnableIRQ(RTC_IRQn);
}
/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
  /* Disable global interrupts */
  INT_Disable();

  /* Enable clocks for used peripherals */
  setupCMU();

  /* Setup the DAC */
  setupDAC();

  /* Setup the ACMP */
  setupACMP();

  /* Setup the GPIO */
  setupGPIO();

  /* Setup the PRS */
  setupPRS();

  /* Setup the PCNT */
  setupPCNT();

  /* Setup the RTC */
  setupRTC();

  /* setup lesense */
  setupLESENSE();

  /* Enable global interrupts */
  INT_Enable();

  while (1)
  {
    /* Enter EM2. */
    EMU_EnterEM2(true);
  }
}
