/**************************************************************************//**
 * @file photointerrupter_lesense.c
 * @brief IR Sensor code example
 * @author Silicon Labs
 * @version 1.04
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
#include "em_gpio.h"
#include "em_int.h"
#include "em_lesense.h"
#include "em_rtc.h"
#include "segmentlcd.h"

/**************************************************************************//**
 * Macro definitions
 *****************************************************************************/
/* LESENSE */

/* How many times per second the sensor scans for objects.
Energy consumption increases linearly with frequency. */
#define OBJECT_SCAN_FREQ         5

/* RTC */
#define RTC_FREQ               32768

/* How often to calibrate the analog comparator threshold value */
#define SENSOR_CALIB_PERIOD_SEC    0 

/* How many levels below the actual threshold should ACOMPTHRES be set to */
#define SENSOR_SENSITIVITY     1

#define CH4_EXCITE_PORT           gpioPortD
#define CH4_EXCITE_PIN            6
#define CH4_SENSOR_PORT           gpioPortC
#define CH4_SENSOR_PIN            0

#define CH5_EXCITE_PORT           gpioPortD
#define CH5_EXCITE_PIN            7
#define CH5_SENSOR_PORT           gpioPortC
#define CH5_SENSOR_PIN            1

#define STATE_0                   0
#define STATE_1                   1

volatile uint32_t proximityCounter = 0;

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
void sensorCalib(void);

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
  LESENSE_IntClear(LESENSE_IFS_DEC);
  
  SegmentLCD_Number(++proximityCounter);
}

/**************************************************************************//**
 * @brief RTC_IRQHandler
 * Interrupt Service Routine for RTC Interrupt Line
 *****************************************************************************/
void RTC_IRQHandler(void)
{
  uint32_t flags;
  flags = RTC_IntGet();  

  /* if comp0 interrupt flag is set */
  if(((flags & _RTC_IFS_COMP0_MASK)& RTC->IEN) == RTC_IFS_COMP0)
  {
      sensorCalib();
  }

  /* Clear interrupt flag */
  RTC_IntClear(flags); 
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

  /* Low energy peripherals */
  CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFRCO);
  CMU_ClockEnable(cmuClock_CORELE, true);
  CMU_ClockEnable(cmuClock_LESENSE, true);

  /* RTC */
  CMU_ClockEnable(cmuClock_RTC, true);
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
    .halfBias                 = false,                
    .biasProg                 = 0x6,                  /* Experiment with higher value if sensor doesn't work */
    .interruptOnFallingEdge   = false,
    .interruptOnRisingEdge    = false,
    .warmTime                 = acmpWarmTime256,      /* Warmuptime should be longer than 10 micro s */
    .hysteresisLevel          = acmpHysteresisLevel0, 
    .inactiveValue            = false,
    .lowPowerReferenceEnabled = false,
    .vddLevel                 = 0x00,                 /* Controlled by LESENSE */
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
    .coreCtrl         =  LESENSE_CORECTRL_DESC_DEFAULT,
    .timeCtrl         =  LESENSE_TIMECTRL_DESC_DEFAULT,
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
      .acmp0Mode      = lesenseACMPModeMuxThres, /* LESENSE controls the threshold value (VDDLEVEL) of ACMP0 */
      .acmp1Mode      = lesenseACMPModeMuxThres, /* LESENSE controls the threshold value (VDDLEVEL) of ACMP1 */
      .warmupMode     = lesenseWarmupModeNormal  /* The analog comparators are shut down when LESENSE is idle */
    },
    .decCtrl          = LESENSE_DECCTRL_DESC_DEFAULT
  };

  /* Channel configuration */
  static const LESENSE_ChDesc_TypeDef initLesenseCh =
  {
    .enaScanCh     = true,                    /* Enable scan on this channel */
    .enaPin        = false,
    .enaInt        = false, 
    .chPinExMode   = lesenseChPinExHigh,      /* Pin is high when excitating */
    .chPinIdleMode = lesenseChPinIdleDis,     /* Pin idle when channel idle  */
    .useAltEx      = true,                    /* Use alternative excitation pin  */
    .shiftRes      = true,                    /* Result is shifted into the decoder register */
    .invRes        = false,
    .storeCntRes   = true,
    .exClk         = lesenseClkHF,            /* Use HF clock for excitation timing */
    .sampleClk     = lesenseClkHF,            /* Use HF clock for sample timing */
    .exTime        = 0x1F,                    /* Clk cycles to excite emitter and decoder */
    .sampleDelay   = 0x1F,                    /* Clc cycles to wait before sampling comparator */
    .measDelay     = 0x00, 
    .acmpThres     = 0x0F,                    /* Initial comperator threshold  */
    .sampleMode    = lesenseSampleModeACMP,
    .intMode       = lesenseSetIntNone,
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
      .alwaysEx  = true /* Connected to IR sensor */
    },
    .AltEx[1] = 
    {
      .enablePin = true,
      .idleConf  = lesenseAltExPinIdleDis,
      .alwaysEx  = false /* Connected to IR LED */
    }
  };

  /* Initialize LESENSE interface _with_ RESET. */
  LESENSE_Init(&initLesense, true);
   
  /* Configure channels */
  LESENSE_ChannelConfig(&initLesenseCh, 0);
  LESENSE_ChannelConfig(&initLesenseCh, 1); 
  
  /* Configure alternate excitation channels */
  LESENSE_AltExConfig(&initAltEx);
  
  
 
  /* State machine Photointerrupter setup */
  
  LESENSE_DecStDesc_TypeDef decConf =
  {
    .chainDesc = false,
    .confA     = 
    {
      .compVal   = 0x3, /* Trigger transition when scan result = compVal */
      .compMask  = 0xC, /* Mask out the upper two bits */
      .nextState = STATE_1,
      .prsAct    = lesenseTransActNone,
      .setInt    = true
    },
    .confB       = 
    {
      .compVal   = 0x3,
      .compMask  = 0xC,
      .nextState = STATE_1,
      .prsAct    = lesenseTransActDown,
      .setInt    = true
    }
  };
  
  /* Configure state 0 */
  LESENSE_DecoderStateConfig(&decConf, STATE_0);
  
  /* Configure state 1 */
  decConf.confA.nextState = STATE_0;
  decConf.confB.nextState = STATE_0;
  
  decConf.confA.compVal = 0x1;
  decConf.confB.compVal = 0x1;
  
  decConf.confA.setInt  = false;	/* Only interrupt when transition from state 0 to 1 */
  decConf.confB.setInt  = false;
    
  LESENSE_DecoderStateConfig(&decConf, STATE_1);
  
  /* Set initial decoder state to STATE_0 */
  LESENSE_DecoderStateSet(STATE_0);

  /* State machine Photointerrupter end */
  
   
  
  /* Start Scan */
  LESENSE_ScanStart();


  /* Clear all pending interrupts to avoid going to the interrupt routine */
  LESENSE_IntClear(_LESENSE_IF_MASK);

  /* Enable Decoder interrupts */
  LESENSE_IntEnable(LESENSE_IFS_DEC);
  
  /* Set scan frequency */
  LESENSE_ScanFreqSet(0, OBJECT_SCAN_FREQ);

  /* Set clock divisor for LF clock. */
  LESENSE_ClkDivSet(lesenseClkLF, lesenseClkDiv_1);
  
  LESENSE_ClkDivSet(lesenseClkHF , lesenseClkDiv_8);

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
  GPIO_DriveModeSet(CH4_EXCITE_PORT, gpioDriveModeHigh); 
  GPIO_DriveModeSet(CH4_SENSOR_PORT, gpioDriveModeStandard);
   
  GPIO_PinModeSet(CH4_EXCITE_PORT, CH4_EXCITE_PIN, gpioModePushPull, 0);
  GPIO_PinModeSet(CH4_SENSOR_PORT, CH4_SENSOR_PIN, gpioModeDisabled, 0);
   
  GPIO_DriveModeSet(CH5_EXCITE_PORT, gpioDriveModeStandard); 
  GPIO_DriveModeSet(CH5_SENSOR_PORT, gpioDriveModeStandard);
  
  GPIO_PinModeSet(CH5_EXCITE_PORT, CH5_EXCITE_PIN, gpioModePushPull, 0);
  GPIO_PinModeSet(CH5_SENSOR_PORT, CH5_SENSOR_PIN, gpioModeDisabled, 0);
}

/**************************************************************************//**
 * @brief  Sets up the RTC
 *****************************************************************************/
void setupRTC(void)
{
  /* SENSOR_CALIB_PERIOD_SEC == 0, do not setup RTC */
  if(SENSOR_CALIB_PERIOD_SEC > 0)
  {
    /* RTC configuration */
    static const RTC_Init_TypeDef rtcInit =
    {
      .enable   = true,
      .debugRun = false,
      .comp0Top = true
    };
  
    RTC_Init(&rtcInit);
  
    /* Set compare value for sensor calibration */
  
    RTC_CompareSet(0, SENSOR_CALIB_PERIOD_SEC*RTC_FREQ);
    RTC_IntEnable(RTC_IFS_COMP0);
    NVIC_EnableIRQ(RTC_IRQn);
  }
}


/**************************************************************************//**
 * @brief  Sensor calibration function
 *****************************************************************************/
void sensorCalib()
{
  /* Disable all interrupts */
  INT_Disable();
 
  uint32_t i;
  uint32_t sensorRes = 0;
  uint8_t acmpthres = 0;
  uint8_t thresholdFound = 0;
  uint8_t searchDir = 1;
  
  /* Disable state machine during calibration */
  LESENSE->DECCTRL |= LESENSE_DECCTRL_DISABLE;
  
  /* Stop scanning of lesense */
  LESENSE_ScanStop();

  /* Set LESENSE to one shot mode */
  LESENSE->CTRL |= LESENSE_CTRL_SCANMODE_ONESHOT;
  
   /* Clear LESENSE interrupt */
  LESENSE_IntClear(LESENSE_IF_SCANCOMPLETE);
   
  /* Get current ACMP threshold value */
  acmpthres = (LESENSE->CH[0].INTERACT & _LESENSE_CH_INTERACT_ACMPTHRES_MASK) >> _LESENSE_CH_INTERACT_ACMPTHRES_SHIFT;
  
  /* Start one LESENSE SCAN */ 
  LESENSE_ScanStart();
  
  /* While scan not complete */
  while(!(LESENSE->IF & _LESENSE_IF_SCANCOMPLETE_MASK));
  LESENSE_IntClear(LESENSE_IF_SCANCOMPLETE);
  
  /* Get result of the sensor scan */   
  sensorRes = LESENSE_ScanResultGet();
  
  
  /* If ACMPTHRES input is above sensor input */
  if((sensorRes & 0x1) == 0x0) 
  {
    searchDir = 0; /* Decrease ACMPTHRES to find threshold0 */
  }

  while(!thresholdFound) 
  {
    if(searchDir)
    {
      acmpthres++;
    }
    else
    {
      acmpthres--;
    }
    
    /* If calibration fails */
    if((acmpthres == 0) || (acmpthres > 63))
    {
      break;
    } 
    
    /* Set comparator threshold value for channel 0 to acmpthres */
    LESENSE->CH[0].INTERACT = ( LESENSE->CH[0].INTERACT & ~_LESENSE_CH_INTERACT_ACMPTHRES_MASK )
        | (acmpthres << _LESENSE_CH_INTERACT_ACMPTHRES_SHIFT);
    
    /* Give acmp time to stabilize VDDLEVEL */
    for(i=0; i<10000; i++); 
    
    /* Start LESENSE SCAN */
    LESENSE_ScanStart();
    
    /* While scan not complete */
    while(!(LESENSE->IF & _LESENSE_IF_SCANCOMPLETE_MASK));
    LESENSE_IntClear(LESENSE_IF_SCANCOMPLETE);
   
    /* Get result of the sensor scan */
    sensorRes = LESENSE_ScanResultGet();   
    
    switch(searchDir)
    {  
	/* If ACMPTHRES input is above sensor input */
    case 0:
      if((sensorRes & 0x1) == 0x1)
      {
        thresholdFound = 1;
		/* Check for underflow */
        if(acmpthres < SENSOR_SENSITIVITY)
        {
          acmpthres = 0;
        }
        else
        {
          acmpthres-= SENSOR_SENSITIVITY;
        }              
      }
      break;
	/* If ACMPTHRES input is below sensor input */
    case 1:
      if((sensorRes & 0x1) == 0x0)
      {
        thresholdFound = 1;
		/* Check for underflow */  
        if(acmpthres < (1 + SENSOR_SENSITIVITY))
        {
          acmpthres = 0;
        }
        else
        {
		  /* Set sensor threshold to be atleast 1 */
          acmpthres-= (1 + SENSOR_SENSITIVITY);
        }
      }          
      break;
    }
  }

  /* Set comparator threshold value for channel 0 to acmpthres */
  LESENSE->CH[0].INTERACT = ( LESENSE->CH[0].INTERACT & ~_LESENSE_CH_INTERACT_ACMPTHRES_MASK )
                                | (acmpthres << _LESENSE_CH_INTERACT_ACMPTHRES_SHIFT); 

  /* Set comparator threshold value for channel 1 to acmpthres */
  LESENSE->CH[1].INTERACT = ( LESENSE->CH[1].INTERACT & ~_LESENSE_CH_INTERACT_ACMPTHRES_MASK )
                                | (acmpthres << _LESENSE_CH_INTERACT_ACMPTHRES_SHIFT);  

  /* Enable periodic LESENSE scans */
  LESENSE->CTRL &= ~LESENSE_CTRL_SCANMODE_ONESHOT;
  
    /* Start scan. */
  LESENSE_ScanStart();

  /* Let LESENSE finish one scan before continuing */
  while(!(LESENSE->IF & _LESENSE_IF_SCANCOMPLETE_MASK));
  
  /* Reenable state machine */
  LESENSE->DECCTRL &= ~_LESENSE_DECCTRL_DISABLE_MASK;
  
  /* Enable all interrupts and clear pending interrupts */
  LESENSE_IntClear(_LESENSE_IF_MASK);
  
  /* Enable all interrupts */
  INT_Enable();
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

  /* Init LCD driver */
  SegmentLCD_Init(false);
 
   /* Setup the GPIO */
  setupGPIO();
  
  /* Setup the ACMP */
  setupACMP();

  /* Setup the RTC */
  setupRTC();

  /* setup lesense */
  setupLESENSE();

  /* Calibrate ACMP threshold */
  sensorCalib();

  /* Display number of times sensor is triggered */
  SegmentLCD_Number(proximityCounter);
  
  /* Enable global interrupts */
  INT_Enable();
 
  while (1)
  {
    /* Enter EM2. LESENSE will wake up the CPU. */
    EMU_EnterEM2(true);
  }
}

