/**************************************************************************//**
 * @file lesense_lightsense_single.c
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
#include "em_dac.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_int.h"
#include "em_lesense.h"
#include "em_rtc.h"

/**************************************************************************//**
 * Macro definitions
 *****************************************************************************/


/* Default configuration for alternate excitation channel. */
#define LESENSE_LIGHTSENSE_ALTEX_DIS_CH_CONF                                    \
  {                                                                             \
    false,                  /* Alternate excitation enabled.*/                  \
    lesenseAltExPinIdleDis, /* Alternate excitation pin is disabled in idle. */ \
    false                   /* Excite only for corresponding channel. */        \
  }

/* ACMP */
#define ACMP_NEG_REF           acmpChannelVDD
#define ACMP_THRESHOLD         0x38                         /* Reference value for the lightsensor.
                                                             * Value works well in office light 
                                                             * conditions. Might need adjustment 
                                                             * for other conditions. */

/* USER LED, how long should it stay on after sensor triggers. */
#define USER_LED_SECONDS_ON    2

/* RTC */
#define RTC_FREQ               32768
#define RTC_COMP_VALUE         (RTC_FREQ * USER_LED_SECONDS_ON)

#if defined ( STK3700 )
  /* GPIO */
	#define LED_GPIO_PORT gpioPortE
	#define LED_GPIO_PIN  2
	/* LESENSE */
	#define LIGHTSENSE_CH             6
	#define LIGHTSENSE_EXCITE_PORT    gpioPortD
	#define LIGHTSENSE_EXCITE_PIN     6
	#define LIGHTSENSE_SENSOR_PORT    gpioPortC
	#define LIGHTSENSE_SENSOR_PIN     6
	#define LCSENSE_SCAN_FREQ         20
  #define LIGHTSENSE_INTERRUPT      LESENSE_IF_CH6
#elif defined ( STK3300 )
  /* GPIO */
	#define LED_GPIO_PORT gpioPortD
	#define LED_GPIO_PIN  7
	/* LESENSE */
	#define LIGHTSENSE_CH             4
	#define LIGHTSENSE_EXCITE_PORT    gpioPortD
	#define LIGHTSENSE_EXCITE_PIN     6
	#define LIGHTSENSE_SENSOR_PORT    gpioPortC
	#define LIGHTSENSE_SENSOR_PIN     4
	#define LCSENSE_SCAN_FREQ         20
  #define LIGHTSENSE_INTERRUPT      LESENSE_IF_CH4
#else
  #error "undefined KIT"
#endif

/**************************************************************************//**
 * Interrupt handlers prototypes
 *****************************************************************************/
void LESENSE_IRQHandler(void);
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
 * @brief LESENSE_IRQHandler
 * Interrupt Service Routine for LESENSE Interrupt Line
 *****************************************************************************/
void LESENSE_IRQHandler(void)
{
  /* Clear interrupt flag */
  LESENSE_IntClear(LIGHTSENSE_INTERRUPT);

  /* Disable RTC first to reset counter */
  RTC_Enable(false);
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
    .fullBias                 = false, /* The lightsensor is slow acting, */
    .halfBias                 = true,  /* comparator bias current can be set to lowest setting.*/
    .biasProg                 = 0x0,   /* Analog comparator will still be fast enough */
    .interruptOnFallingEdge   = false, /* No comparator interrupt, lesense will issue interrupts. */
    .interruptOnRisingEdge    = false, 
    .warmTime                 = acmpWarmTime512, /* Not applicable, lesense controls this. */
    .hysteresisLevel          = acmpHysteresisLevel5, /* Some hysteresis will prevent excessive toggling. */
    .inactiveValue            = false, /* Not applicable, lesense controls this. */
    .lowPowerReferenceEnabled = false, /* Can be enabled for even lower power. */
    .vddLevel                 = 0x00,  /* Not applicable, lesense controls this through .acmpThres value. */
    .enable                   = false  /* Not applicable, lesense controls this. */
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
    { /* LESENSE configured for periodic scan. */
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
      .biasMode     = lesenseBiasModeDutyCycle, /* Lesense should duty cycle comparator and related references etc. */
      .debugRun     = false
    },

    .timeCtrl         =
    {
      .startDelay     = 0 /* No start delay needed for this application. */
    },

    .perCtrl          =
    {  /* DAC is not needed for this application. */
      .dacCh0Data     = lesenseDACIfData,
      .dacCh0ConvMode = lesenseDACConvModeDisable,
      .dacCh0OutMode  = lesenseDACOutModeDisable,
      .dacCh1Data     = lesenseDACIfData,
      .dacCh1ConvMode = lesenseDACConvModeDisable,
      .dacCh1OutMode  = lesenseDACOutModeDisable,
      .dacPresc       = 0,
      .dacRef         = lesenseDACRefBandGap,
      .acmp0Mode      = lesenseACMPModeMuxThres, /* Allow LESENSE to control ACMP mux and reference threshold. */
      .acmp1Mode      = lesenseACMPModeMuxThres,
      .warmupMode     = lesenseWarmupModeNormal  /* Normal mode means LESENSE is allowed to dutycycle comparator and reference. */
    },

    .decCtrl          =
    { /* Decoder or statemachine not used in this code example. */
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
  /* Only one channel is configured for the lightsense application. */
  static const LESENSE_ChDesc_TypeDef initLesenseCh =
  {
    .enaScanCh     = true, 
    .enaPin        = false,                 /* Pin is input, no enabling needed. Separate pin is exciting the sensor. */
    .enaInt        = true,                  /* Enable interrupt for this channel. */
    .chPinExMode   = lesenseChPinExHigh,    /* Excite by pullin pin high. */
    .chPinIdleMode = lesenseChPinIdleDis,   /* During Idle, excite pin should be disabled (tri-stated). */
    .useAltEx      = true,                  /* Use alternate excite pin. */
    .shiftRes      = false,                 /* Not applicable, only for decoder operation. */
    .invRes        = false,                 /* No need to invert result. */
    .storeCntRes   = true,                  /* Not applicable, don't care really. */
    .exClk         = lesenseClkLF,          /* Using low frequency clock for timing the excitation. */
    .sampleClk     = lesenseClkLF,          /* Using low frequency clock for timing the sample instant. */
    .exTime        = 0x01,                  /* 1 LFclk cycle is enough excitation time, this depends on response time of light sensor. */
    .sampleDelay   = 0x01,                  /* Sampling should happen when excitation ends, it it happens earlier, excitation time might as well be reduced. */
    .measDelay     = 0x00,                  /* Not used here, basically only used for applications which uses the counting feature. */
    .acmpThres     = ACMP_THRESHOLD,        /* This is the analog comparator threshold setting, determines when the acmp triggers. */
    .sampleMode    = lesenseSampleModeACMP, /* Sampling acmp, not counting. */
    .intMode       = lesenseSetIntNegEdge,  /* Interrupt when voltage falls below threshold. */
    .cntThres      = 0x0000,                /* Not applicable. */
    .compMode      = lesenseCompModeLess    /* Not applicable. */
  };

  /* Alternate excitation channels configuration. */
  /* The lightsensor is excited by alternate excite channel 0. */
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

  /* Enable interrupt in NVIC. */
  NVIC_EnableIRQ(LESENSE_IRQn);

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


