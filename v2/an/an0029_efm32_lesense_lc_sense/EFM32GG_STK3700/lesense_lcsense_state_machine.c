/**************************************************************************//**
 * @file lesense_lcsense_state_machine.c
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
#include "em_lcd.h"
#include "em_pcnt.h"
#include "em_prs.h"
#include "em_int.h"
#include "em_lesense.h"
#include "em_rtc.h"
#include "segmentlcd.h"

/**************************************************************************//**
 * Macro definitions
 *****************************************************************************/
/* LESENSE */
#define LCSENSE_CH7              7
#define LCSENSE_CH7_PORT         gpioPortC
#define LCSENSE_CH7_PIN          7
#define LCSENSE_SCAN_FREQ        20
#define LCSENSE_CH11             11
#define LCSENSE_CH11_PORT        gpioPortC
#define LCSENSE_CH11_PIN         11
#define LCSENSE_STATE0           0
#define LCSENSE_STATE1           1
#define LCSENSE_STATE2           2
#define LCSENSE_STATE3           3
#define LCSENSE_ERROR_STRING     "ERROR"
#define INITIAL_STATE            0
/* ACMP */
#define ACMP_VDD_LEVEL_MAX       63
#define ACMP_NEG_REF             acmpChannelVDD
#define ACMP_LOC                 0
#define ACMP_VDD_SCALE           0x0D                      /* reference for the LC sensor to be
                                                            * close to the DAC voltage */

/* LCD */
#define LCD_SECONDS_ON           3

/* RTC */
#define RTC_FREQ                 32768
#define RTC_COMP_VALUE           (RTC_FREQ * LCD_SECONDS_ON)

/* DAC */
#define DAC_FREQ                 500000
#define DAC_CHANNEL              1
#define DAC_DATA                 800

/* PRS */
#define PRS_CHANNEL0             0
#define PRS_CHANNEL1             1

/* PCNT */
#define PCNT_TOP                 0xFFFF
#define PCNT_DIRCHANGE_STRING    "DIRCHNG"

/**************************************************************************//**
 * Interrupt handlers prototypes
 *****************************************************************************/
void LESENSE_IRQHandler(void);
void PCNT0_IRQHandler(void);
void RTC_IRQHandler(void);

/**************************************************************************//**
 * Functions prototypes
 *****************************************************************************/
void setupCMU(void);
void setupDAC(void);
void writeDataDAC(DAC_TypeDef *dac, unsigned int value, unsigned int ch);
void setupACMP(void);
void setupLESENSE(void);
void setupGPIO(void);
void setupLCD(void);
void setupRTC(void);
void setupPCNT(void);
void setupPRS(void);

/**************************************************************************//**
 * Interrupt handlers
 *****************************************************************************/
/**************************************************************************//**
 * @brief LESENSE_IRQHandler
 * Interrupt Service Routine for LESENSE Interrupt Line
 *****************************************************************************/
void LESENSE_IRQHandler(void)
{
  /* Read interrupts flags */
  uint32_t intFlags = LESENSE_IntGet();
  
  /* Clear interrupts */
  LESENSE_IntClear(LESENSE_IFC_DEC | LESENSE_IFC_DECERR);

  /* Enable clock for LCD. */
  CMU_ClockEnable(cmuClock_LCD, true);
  /* Wait until SYNCBUSY_CTRL flag is cleared. */
  LCD_SyncBusyDelay(LCD_SYNCBUSY_CTRL);
  /* Enable LCD. */
  LCD_Enable(true);

  /* If there is a decoder error stop trap execution in this loop */
  if (intFlags & LESENSE_IF_DECERR)
  {
    /* DECODER ERROR */
    SegmentLCD_Write(LCSENSE_ERROR_STRING);
    while (1) ;
  }

  /* Write PCNT counter number the LCD */
  SegmentLCD_Number(PCNT_CounterGet(PCNT0));

  /* Disable RTC first to reset counter */
  RTC_Enable(false);
  /* Set compare value */
  RTC_CompareSet(0, RTC_COMP_VALUE);
  /* Enable RTC */
  RTC_Enable(true);
}

/**************************************************************************//**
 * @brief PCNT0_IRQHandler
 * Interrupt Service Routine for PCNT0 Interrupt Line
 *****************************************************************************/
void PCNT0_IRQHandler(void)
{
  PCNT_IntGet(PCNT0);
  PCNT_IntClear(PCNT0, PCNT_IEN_DIRCNG);

  /* Enable clock for LCD. */
  CMU_ClockEnable(cmuClock_LCD, true);
  /* Wait until SYNCBUSY_CTRL flag is cleared. */
  LCD_SyncBusyDelay(LCD_SYNCBUSY_CTRL);
  /* Enable LCD. */
  LCD_Enable(true);
  /* Write PCNT counter number the LCD */
  SegmentLCD_Write(PCNT_DIRCHANGE_STRING);

  /* Disable RTC first to reset counter */
  RTC_Enable(false);
  /* Set compare value */
  RTC_CompareSet(0, RTC_COMP_VALUE);
  /* Enable RTC */
  RTC_Enable(true);
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
  /* Turn off all segments */
  SegmentLCD_AllOff();
  /* Disable LCD to avoid excessive current consumption */
  LCD_Enable(false);
  /* Wait until SYNCBUSY_CTRL flag is cleared. */
  LCD_SyncBusyDelay(LCD_SYNCBUSY_CTRL);
  /* Disable clock for LCD. */
  CMU_ClockEnable(cmuClock_LCD, false);
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
  CMU_ClockEnable(cmuClock_ACMP1, true);

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

  /* LCD */
  CMU_ClockEnable(cmuClock_LCD, true);

  /* PCNT */
  CMU_ClockEnable(cmuClock_PCNT0, true);

  /* PCNT */
  CMU_ClockEnable(cmuClock_PRS, true);
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
	
	/* 	Set OPA1SHORT	
			Setting this will cause the output of one sensor to be connected to the DAC output 
			while the other sensor is measured, and vice versa. This is necessary to avoid cross 
			coupling when two LC sensors are placed close */
	DAC0->OPACTRL |= DAC_OPACTRL_OPA1SHORT;

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
  /* There is no default configuration for this */
  static const ACMP_Init_TypeDef acmpInit =
  {
    .fullBias                 = false,
    .halfBias                 = true,
    .biasProg                 = 0xF,
    .interruptOnFallingEdge   = false,
    .interruptOnRisingEdge    = false,
    .warmTime                 = acmpWarmTime4,
    .hysteresisLevel          = acmpHysteresisLevel0,
    .inactiveValue            = false,
    .lowPowerReferenceEnabled = false,
    .vddLevel                 = ACMP_VDD_SCALE,
    .enable                   = false
  };

  /* Initialize ACMPs */
  ACMP_Init(ACMP0, &acmpInit);
  ACMP_Init(ACMP1, &acmpInit);

  /* Select Vdd as negative reference
   * Positive reference is controlled by LESENSE */
  ACMP_ChannelSet(ACMP0, acmpChannelVDD, acmpChannel7);
  ACMP_ChannelSet(ACMP1, acmpChannelVDD, acmpChannel3);
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
      .dacCh1OutMode  = lesenseDACOutModePinADCACMP,
      .dacPresc       = 31,
      .dacRef         = lesenseDACRefVdd,
      .acmp0Mode      = lesenseACMPModeMux,
      .acmp1Mode      = lesenseACMPModeMux,
      .warmupMode     = lesenseWarmupModeDAC
    },

    .decCtrl          =
    {
      .decInput  = lesenseDecInputSensorSt,
      .initState = 0,
      .chkState  = true,
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
  static const LESENSE_ChDesc_TypeDef initLesenseCh =
  {
    .enaScanCh     = true,
    .enaPin        = true,
    .enaInt        = false,
    .chPinExMode   = lesenseChPinExLow,
    .chPinIdleMode = lesenseChPinIdleDis,
    .useAltEx      = false,
    .shiftRes      = true,
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
  LESENSE_ChannelConfig(&initLesenseCh, LCSENSE_CH7);

  /* Configure channel 11 */
  LESENSE_ChannelConfig(&initLesenseCh, LCSENSE_CH11);

  /* Set scan frequency for 20Hz */
  LESENSE_ScanFreqSet(0, LCSENSE_SCAN_FREQ);

  /* Set clock divisor for LF clock. */
  LESENSE_ClkDivSet(lesenseClkLF, lesenseClkDiv_2);
  /* Set clock divisor for HF clock. */
  LESENSE_ClkDivSet(lesenseClkHF, lesenseClkDiv_1);

  /* CALIBRATION */
  /* Calibration is done differently here because
   * channel 11 is also connected to one of the pads
   * which is increases the capacitance to ground.
   * Because of this 16 scans are not enough to charge
   * this capacitor in parallel with the DAC capacitor
   * and the calibration routine cannot be used.
   * However this is an STK related constraint, there
   * are no more LESENSE channels available so one of
   * the capsense channels had to be used for this example.
   * The user can use the calibration routine in the other
   * examples on his own setup. */
  /* Start scan */
  LESENSE_ScanStart();

  /* Wait cycle so that the DAC cap + capsense
   * cap are fully charged and the readings are reliable */
  volatile int i;
  for (i = 0; i < 1000000; i++) ;

  /* The result buffer alternates between channel 7 and 11, and since
   * 16 is divisible by two we know that the even buffer index hold
   * channel 7 results and odd buffer indes hold channel 11 results.
   * These can then be used as counter threshold */
  LESENSE_ChannelThresSet(LCSENSE_CH7, 0, LESENSE_ScanResultDataBufferGet(0));
  LESENSE_ChannelThresSet(LCSENSE_CH11, 0, LESENSE_ScanResultDataBufferGet(1));

  /* Configure decoder */
  /* Interrupts will be enabled for transitions between
   * states 0 and 3 to show the pulse counter value */
  /* Configuration structure for state 0 */
  LESENSE_DecStDesc_TypeDef decConf =
  {
    .chainDesc = false,
    .confA     = 
    {
      .compVal   = 0x02,
      .compMask  = 0x0,
      .nextState = LCSENSE_STATE1,
      .prsAct    = lesenseTransActNone,
      .setInt    = false
    },
    .confB       = 
    {
      .compVal   = 0x01,
      .compMask  = 0x0,
      .nextState = LCSENSE_STATE3,
      .prsAct    = lesenseTransActDown,
      .setInt    = true
    }
  };
  /* Configure state 0 */
  LESENSE_DecoderStateConfig(&decConf, LCSENSE_STATE0);

  /* Change necessary structure fields for state 1 */
  decConf.confA.compVal   = 0x03;
  decConf.confA.nextState = LCSENSE_STATE2;
  decConf.confA.prsAct    = lesenseTransActNone;
  decConf.confA.setInt    = false;
  decConf.confB.compVal   = 0x00;
  decConf.confB.nextState = LCSENSE_STATE0;
  decConf.confB.prsAct    = lesenseTransActNone;
  decConf.confB.setInt    = false;
  /* Configure state 1 */
  LESENSE_DecoderStateConfig(&decConf, LCSENSE_STATE1);

  /* Change necessary structure fields for state 2 */
  decConf.confA.compVal   = 0x01;
  decConf.confA.nextState = LCSENSE_STATE3;
  decConf.confA.prsAct    = lesenseTransActNone;
  decConf.confA.setInt    = false;
  decConf.confB.compVal   = 0x02;
  decConf.confB.nextState = LCSENSE_STATE1;
  decConf.confB.prsAct    = lesenseTransActNone;
  decConf.confB.setInt    = false;
  /* Configure state 2 */
  LESENSE_DecoderStateConfig(&decConf, LCSENSE_STATE2);

  /* Change necessary structure fields for state 3 */
  decConf.confA.compVal   = 0x00;
  decConf.confA.nextState = LCSENSE_STATE0;
  decConf.confA.prsAct    = lesenseTransActUp;
  decConf.confA.setInt    = true;
  decConf.confB.compVal   = 0x03;
  decConf.confB.nextState = LCSENSE_STATE2;
  decConf.confB.prsAct    = lesenseTransActNone;
  decConf.confB.setInt    = false;
  /* Configure state 3 */
  LESENSE_DecoderStateConfig(&decConf, LCSENSE_STATE3);

  /* Set initial decoder state to 0 */
  LESENSE_DecoderStateSet(INITIAL_STATE);

  /* Start Scan */
  LESENSE_ScanStart();

  /* Waiting for buffer to be full before starting decoder
   * The first result of scan is zero because the DAC level isn't
   * Vdd/2 yet so it will cause a false trigger */
  while (!(LESENSE->STATUS & LESENSE_STATUS_BUFFULL)) ;

  /* Clear all pending interrupts to avoid going to the interrupt
   * routine */
  LESENSE_IntClear(0x7FFFFF);

  /* Enable Decoder and Decoder Error interrupts */
  LESENSE_IntEnable(LESENSE_IFS_DEC | LESENSE_IFS_DECERR);

  /* Enable interrupt in NVIC. */
  NVIC_EnableIRQ(LESENSE_IRQn);

  /* Star Decoder */
  LESENSE_DecoderStart();
}

/**************************************************************************//**
 * @brief  Sets up the LCD
 *****************************************************************************/
void setupLCD(void)
{
  /* Initialize LCD without voltage boost */
  SegmentLCD_Init(false);

  /* Shut down LCD in order to save energy */
  /* Turn off all segments */
  SegmentLCD_AllOff();
  /* Disable LCD to avoid excessive current consumption */
  LCD_Enable(false);
  /* Wait until SYNCBUSY_CTRL flag is cleared. */
  LCD_SyncBusyDelay(LCD_SYNCBUSY_CTRL);
}
/**************************************************************************//**
 * @brief  Sets up the GPIO
 *****************************************************************************/
void setupGPIO(void)
{
  /* Configure measuring pin for channel 7 as push pull */
  GPIO_PinModeSet(LCSENSE_CH7_PORT, LCSENSE_CH7_PIN, gpioModePushPull, 0);
  /* Configure measuring pin for channel 11 as push pull */
  GPIO_PinModeSet(LCSENSE_CH11_PORT, LCSENSE_CH11_PIN, gpioModePushPull, 0);
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

  RTC_IntEnable(RTC_IFS_COMP0);
  NVIC_EnableIRQ(RTC_IRQn);
}

/**************************************************************************//**
 * @brief  Sets up the PCNT
 *****************************************************************************/
void setupPCNT(void)
{
  /* PCNT configuration constant table. */
  static const PCNT_Init_TypeDef initPCNT =
  {
    .mode        = pcntModeOvsSingle,  /* Oversampling, single mode. */
    .counter     = 0,                 /* Counter value has been initialized to 0. */
    .top         = PCNT_TOP,           /* Counter top value. */
    .negEdge     = false,              /* Use positive edge. */
    .countDown   = false,              /* Up-counting. */
    .filter      = false,              /* Filter disabled. */
    .hyst        = false,              /* Hysteresis disabled. */
    .s1CntDir    = true,               /* Counter direction is given by S1. */
    .cntEvent    = pcntCntEventBoth,   /* Regular counter counts up on upcount events. */
    .auxCntEvent = pcntCntEventNone,   /* Auxiliary counter doesn't respond to events. */
    .s0PRS       = pcntPRSCh0,         /* PRS channel 0 selected as S0IN. */
    .s1PRS       = pcntPRSCh1          /* PRS channel 1 selected as S1IN. */
  };


  /* Initialize PCNT. */
  PCNT_Init(PCNT0, &initPCNT);

  /* Enable PRS input S0 in PCNT. */
  PCNT_PRSInputEnable(PCNT0, pcntPRSInputS0, true);

  /* Enable PRS input S0 in PCNT. */
  PCNT_PRSInputEnable(PCNT0, pcntPRSInputS1, true);

  /* Clear all pending interrupts */
  PCNT_IntClear(PCNT0, 0xFFFF);

  /* Enable the PCNT overflow interrupt. */
  PCNT_IntEnable(PCNT0, PCNT_IEN_DIRCNG);

  /* Enable the PCNT vector in NVIC */
  NVIC_EnableIRQ(PCNT0_IRQn);
}

/**************************************************************************//**
 * @brief  Sets up the PRS
 *****************************************************************************/
void setupPRS(void)
{
  /* PRS channel 0 configuration. */
  PRS_SourceAsyncSignalSet(PRS_CHANNEL0, PRS_CH_CTRL_SOURCESEL_LESENSED, PRS_CH_CTRL_SIGSEL_LESENSEDEC0);
  /* PRS channel 0 configuration. */
  PRS_SourceAsyncSignalSet(PRS_CHANNEL1, PRS_CH_CTRL_SOURCESEL_LESENSED, PRS_CH_CTRL_SIGSEL_LESENSEDEC1);
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

  /* Setup the RTC */
  setupRTC();

  /* Setup LCD */
  setupLCD();

  /* Setup PCNT */
  setupPCNT();

  /* Setup PRS */
  setupPRS();

  /* Setup lesense */
  setupLESENSE();

  /* Enable global interrupts */
  INT_Enable();

  while (1)
  {
    /* Enter EM2. */
    EMU_EnterEM2(true);
  }
}
