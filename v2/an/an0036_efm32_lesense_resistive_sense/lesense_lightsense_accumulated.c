/**************************************************************************//**
 * @file lesense_lightsense_accumulated.c
 * @brief LESENSE LIGHTSENSE code example
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
#include "em_emu.h"
#include "em_prs.h"
#include "em_pcnt.h"
#include "em_gpio.h"
#include "em_int.h"
#include "em_lesense.h"
#include "em_rtc.h"

/**************************************************************************//**
 * Macro definitions
 *****************************************************************************/
/* Default configuration for alternate excitation channel. */
#define LESENSE_LIGHTSENSE_ALTEX_DIS_CH_CONF  \
  {                                           \
    false,                                    \
    lesenseAltExPinIdleDis,                   \
    false                                     \
  }

/* ACMP */
#define ACMP_NEG_REF           acmpChannelVDD
#define ACMP_VDD_SCALE         0x0D                         /* reference for the LC sensor to be
                                                             * close to the DAC voltage
                                                             * This was calibrated using a scope
                                                             * and probing both the LC sensor and
                                                             * the ACMP output */

/* USER LED */
#define USER_LED_SECONDS_ON    3

/* RTC */
#define RTC_FREQ               32768
#define RTC_COMP_VALUE         (RTC_FREQ * USER_LED_SECONDS_ON)

/* PRS */
#define PRS_CHANNEL            0

/* PCNT */
#define PCNT_TOP               4

#if defined ( STK3700 )
	#define LED_GPIO_PORT gpioPortE
	#define LED_GPIO_PIN  2
	/* LESENSE */
	#define LIGHTSENSE_CH             6
	#define LIGHTSENSE_EXCITE_PORT    gpioPortD
	#define LIGHTSENSE_EXCITE_PIN     6
	#define LIGHTSENSE_SENSOR_PORT    gpioPortC
	#define LIGHTSENSE_SENSOR_PIN     6
	#define LCSENSE_SCAN_FREQ         20
	/* PRS */
	#define PRS_SIGSEL_CHANNEL  	    PRS_CH_CTRL_SIGSEL_LESENSESCANRES6
#elif defined ( STK3300 )
	#define LED_GPIO_PORT gpioPortD
	#define LED_GPIO_PIN  7
	/* LESENSE */
	#define LIGHTSENSE_CH             4
	#define LIGHTSENSE_EXCITE_PORT    gpioPortD
	#define LIGHTSENSE_EXCITE_PIN     6
	#define LIGHTSENSE_SENSOR_PORT    gpioPortC
	#define LIGHTSENSE_SENSOR_PIN     4
	#define LCSENSE_SCAN_FREQ         20
	/* PRS */
	#define PRS_SIGSEL_CHANNEL  	    PRS_CH_CTRL_SIGSEL_LESENSESCANRES4
#else
  #error "undefined KIT"
#endif

/**************************************************************************//**
 * Interrupt handlers prototypes
 *****************************************************************************/
void PCNT0_IRQHandler(void);
void RTC_IRQHandler(void);

/**************************************************************************//**
 * Functions prototypes
 *****************************************************************************/
void setupCMU(void);
void setupACMP(void);
void setupLESENSE(void);
void setupGPIO(void);
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

  /* ACMP */
  CMU_ClockEnable(cmuClock_ACMP0, true);

  /* GPIO */
  CMU_ClockEnable(cmuClock_GPIO, true);

/* Low energy peripherals
 *   LESENSE
 *   LFRCO clock must be enables prior to enabling
 *   clock for the low energy peripherals */
  CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFRCO);
  CMU_ClockEnable(cmuClock_CORELE, true);
  CMU_ClockEnable(cmuClock_LESENSE, true);

  /* RTC */
  CMU_ClockEnable(cmuClock_RTC, true);

  /* PRS */
  CMU_ClockEnable(cmuClock_PRS, true);

  /* PCNT */
  CMU_ClockEnable(cmuClock_PCNT0, true);

  /* Disable clock source for LFB clock. */
  CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_Disabled);
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
    .biasProg                 = 0x0,
    .interruptOnFallingEdge   = false,
    .interruptOnRisingEdge    = false,
    .warmTime                 = acmpWarmTime512,
    .hysteresisLevel          = acmpHysteresisLevel5,
    .inactiveValue            = false,
    .lowPowerReferenceEnabled = false,
    .vddLevel                 = 0x00,
    .enable                   = false
  };

  /* Initialize ACMP */
  ACMP_Init(ACMP0, &acmpInit);
  /* Disable ACMP0 out to a pin. */
  ACMP_GPIOSetup(ACMP0, 0, false, false);
  /* Set up ACMP negSel to VDD, posSel is controlled by LESENSE. */
  ACMP_ChannelSet(ACMP0, acmpChannelVDD, acmpChannel0);
  /* LESENSE controls ACMP thus ACMP_Enable(ACMP0) should NOT be called in order
   * to ensure lower current consumption. */
}

/**************************************************************************//**
 * @brief  Sets up the LESENSE
 *****************************************************************************/
void setupLESENSE(void)
{
  /* LESENSE configuration structure */
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
      .dacCh0ConvMode = lesenseDACConvModeDisable,
      .dacCh0OutMode  = lesenseDACOutModeDisable,
      .dacCh1Data     = lesenseDACIfData,
      .dacCh1ConvMode = lesenseDACConvModeDisable,
      .dacCh1OutMode  = lesenseDACOutModeDisable,
      .dacPresc       = 0,
      .dacRef         = lesenseDACRefBandGap,
      .acmp0Mode      = lesenseACMPModeMuxThres,
      .acmp1Mode      = lesenseACMPModeMuxThres,
      .warmupMode     = lesenseWarmupModeNormal
    },

    .decCtrl          =
    {
      .decInput  = lesenseDecInputSensorSt,
      .initState = 0,
      .chkState  = false,
      .intMap    = true,
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
  static const LESENSE_ChDesc_TypeDef initLesenseCh =
  {
    .enaScanCh     = true,
    .enaPin        = false,
    .enaInt        = false,
    .chPinExMode   = lesenseChPinExHigh,
    .chPinIdleMode = lesenseChPinIdleDis,
    .useAltEx      = true,
    .shiftRes      = false,
    .invRes        = false,
    .storeCntRes   = true,
    .exClk         = lesenseClkLF,
    .sampleClk     = lesenseClkLF,
    .exTime        = 0x01,
    .sampleDelay   = 0x01,
    .measDelay     = 0x00,
    .acmpThres     = 0x38,
    .sampleMode    = lesenseSampleModeACMP,
    .intMode       = lesenseSetIntNegEdge,
    .cntThres      = 0x0000,
    .compMode      = lesenseCompModeLess
  };

  /* Alternate excitation channels configuration */
  static const LESENSE_ConfAltEx_TypeDef initAltEx =
  {
    .altExMap = lesenseAltExMapALTEX,
    .AltEx[0] =
    {
      .enablePin = true,
      .idleConf  = lesenseAltExPinIdleDis,
      .alwaysEx  = true
    },
    .AltEx[1] = LESENSE_LIGHTSENSE_ALTEX_DIS_CH_CONF,
    .AltEx[2] = LESENSE_LIGHTSENSE_ALTEX_DIS_CH_CONF,
    .AltEx[3] = LESENSE_LIGHTSENSE_ALTEX_DIS_CH_CONF,
    .AltEx[4] = LESENSE_LIGHTSENSE_ALTEX_DIS_CH_CONF,
    .AltEx[5] = LESENSE_LIGHTSENSE_ALTEX_DIS_CH_CONF,
    .AltEx[6] = LESENSE_LIGHTSENSE_ALTEX_DIS_CH_CONF,
    .AltEx[7] = LESENSE_LIGHTSENSE_ALTEX_DIS_CH_CONF
  };
  /* Initialize LESENSE interface _with_ RESET. */
  LESENSE_Init(&initLesense, true);

  /* Configure LESENSE channel */
  LESENSE_ChannelConfig(&initLesenseCh, LIGHTSENSE_CH);

  /* Configure alternate excitation channels */
  LESENSE_AltExConfig(&initAltEx);

  /* Set scan frequency */
  LESENSE_ScanFreqSet(0, LCSENSE_SCAN_FREQ);

  /* Set clock divisor for LF clock. */
  LESENSE_ClkDivSet(lesenseClkLF, lesenseClkDiv_2);

  /* Start scan. */
  LESENSE_ScanStart();
}

/**************************************************************************//**
 * @brief  Sets up the GPIO
 *****************************************************************************/
void setupGPIO(void)
{
  /* Configure the drive strength of the ports for the light sensor. */
  GPIO_DriveModeSet(LIGHTSENSE_EXCITE_PORT, gpioDriveModeStandard);
  GPIO_DriveModeSet(LIGHTSENSE_SENSOR_PORT, gpioDriveModeStandard);

  /* Initialize the 2 GPIO pins of the light sensor setup. */
  GPIO_PinModeSet(LIGHTSENSE_EXCITE_PORT, LIGHTSENSE_EXCITE_PIN, gpioModePushPull, 0);
  GPIO_PinModeSet(LIGHTSENSE_SENSOR_PORT, LIGHTSENSE_SENSOR_PIN, gpioModeDisabled, 0);

  /* Configure user led as output */
  GPIO_PinModeSet(LED_GPIO_PORT, LED_GPIO_PIN, gpioModePushPull, 0);
}

/**************************************************************************//**
 * @brief  Sets up the PRS
 *****************************************************************************/
void setupPRS(void)
{
  /* PRS channel 0 configuration */
  /* LESENSE SCANRES bit PRS_SIGSEL_CHANNEL will be used as PRS signal */
  PRS_SourceAsyncSignalSet(PRS_CHANNEL, PRS_CH_CTRL_SOURCESEL_LESENSEL, PRS_SIGSEL_CHANNEL);
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

  /* Set compare value */
  RTC_CompareSet(0, RTC_COMP_VALUE);

  RTC_IntEnable(RTC_IFS_COMP0);
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

