/***************************************************************************//**
 * @file lightsensefft.c
 * @brief FFT transform example
 * @details
 *   Use ADC in order to capture and analyse input from the
 *   light sensor on the STK. Runs floating point FFT algorithm from the CMSIS
 *   DSP Library, and estimate the frequency of the most luminous light source
 *   using sinc interpolation. The main point with this example is to show the
 *   use of the CMSIS DSP library and the floating point capability of the CPU.
 *
 * @par Usage
 *   Connect the light sensor output to the ADC input by shorting pins
 *   15 and 14 on the EXP_HEADER of the STK.
 *   Direct various light sources to the light sensor. Expect no specific
 *   frequency from daylight or from a flashlight. Mains powered incandescent
 *   bulbs should give twice the mains frequency. Using another STK running the
 *   "blink" example modified to various blink rates is an excellent signal
 *   source. The frequency bandwidth is approximately 10-500 Hz.
 *   The frequency shows in the 4 digit numerical display upper right on
 *   the LCD. The LCD also displays the number of CPU cycles used to do
 *   the FFT transform.
 *
 * @author Silicon Labs
 * @version 1.04
 *******************************************************************************
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

#include "em_common.h"
#include "em_emu.h"
#include "em_cmu.h"
#include "em_chip.h"
#include "em_adc.h"
#include "em_gpio.h"
#include "em_rtc.h"
#include "em_acmp.h"
#include "em_lesense.h"
#include "segmentlcd.h"
#include "arm_math.h"
#include "math.h"

/*
 * Light sensor sampling:
 * ----------------------
 *
 *
 *                                | Light sensor
 *                                V
 *              +-----+       +------+       +--------+
 *              | RTC |--+--->| ADC0 |------>| Buffer |
 *              +-----+       +------+       +--------+ 
 *
 *
 * 1. RTC is set to overflow appr. SAMPLE_RATE times per
 *    second, triggering a wakeup from EM2 with Wait For Event instead
 *    of interrupt to save some interrupt latency cycles.
 *
 * 2. After 512 samples, the sample buffer is processed with fft to find the frequency.
 *
 * 3. The example also uses lesense to stay asleep when the lightlevel is too low. 
 *
 * Energy mode usage:
 * ------------------
 *
 * Due to the relatively low sampling rate, we can go to EM2 between adc samples.
 * See AN0021 ADC application note for more information about this.
 */

/**
 * Number of samples processed at a time. This number has to be equal to one
 * of the accepted input sizes of the rfft transform of the CMSIS DSP library.
 * Increasing it gives better resolution in the frequency, but also a longer
 * sampling time.
 */
#define BUFFER_SAMPLES                  512

/** (Approximate) sample rate used for sampling data. */
#define SAMPLE_RATE                     (1024)

/** The GPIO pin used to power the light sensor. */
#define EXCITE_PIN    gpioPortD,6

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
/* LESENSE Pin config */
#define LIGHTSENSE_CH             6
#define LIGHTSENSE_EXCITE_PORT    gpioPortD
#define LIGHTSENSE_EXCITE_PIN     6
#define LIGHTSENSE_SENSOR_PORT    gpioPortC
#define LIGHTSENSE_SENSOR_PIN     6
#define LCSENSE_SCAN_FREQ         5
#define LIGHTSENSE_INTERRUPT      LESENSE_IF_CH6

/** Buffer of uint16_t sample values ready to be FFT-ed. */
static uint16_t lightToFFTBuffer[BUFFER_SAMPLES];

/** Buffer of float samples ready for FFT. */
static float32_t floatBuf[BUFFER_SAMPLES];

/** Complex (interleaved) output from FFT. */
static float32_t fftOutputComplex[BUFFER_SAMPLES * 2];

/** Magnitude of complex numbers in FFT output. */
static float32_t fftOutputMag[BUFFER_SAMPLES];

/** Flag used to indicate whether data is ready for processing */
static volatile bool dataReadyForFFT;
/** Indicate whether we are currently processing data through FFT */
static volatile bool processingFFT;

/** Instance structures for float32_t RFFT */
static arm_rfft_instance_f32 rfft_instance;
/** Instance structure for float32_t CFFT used by the RFFT */
static arm_cfft_radix4_instance_f32 cfft_instance;

/**************************************************************************//**
 * Interrupt handlers prototypes
 *****************************************************************************/
void LESENSE_IRQHandler(void);

/**************************************************************************//**
 * Functions prototypes
 *****************************************************************************/
void setupCMU(void);
void setupACMP(void);
void setupLESENSE(void);

/**************************************************************************//**
 * @brief LESENSE_IRQHandler
 * Interrupt Service Routine for LESENSE Interrupt Line
 *****************************************************************************/
void LESENSE_IRQHandler(void)
{
  /* Clear interrupt flag */
  LESENSE_IntClear(LIGHTSENSE_INTERRUPT);

}

/***************************************************************************//**
 * @brief Enables LFACLK and selects osc as clock source for RTC
 ******************************************************************************/
void RTC_Setup(CMU_Select_TypeDef osc)
{
  RTC_Init_TypeDef init;

  /* Ensure LE modules are accessible */
  CMU_ClockEnable(cmuClock_CORELE, true);

  /* Enable osc as LFACLK in CMU (will also enable oscillator if not enabled) */
  CMU_ClockSelectSet(cmuClock_LFA, osc);

  /* Division prescaler to decrease consumption. */
  CMU_ClockDivSet(cmuClock_RTC, cmuClkDiv_32);

  /* Enable clock to RTC module */
  CMU_ClockEnable(cmuClock_RTC, true);

  init.enable   = false;
  init.debugRun = false;
  init.comp0Top = true; /* Count only to top before wrapping */
  RTC_Init(&init);
  
  /* RTC clock divider is 32 which gives 1024 ticks per second.  */
  RTC_CompareSet(0, ((1024 * SAMPLE_RATE) / 1000000)-1);
    
  /* Enable interrupt generation from RTC0, needed for WFE (wait for event). */
  /* Notice that enabling the interrupt in the NVIC is not needed. */
  RTC_IntEnable(RTC_IF_COMP0);
}

/**************************************************************************//**
 * @brief  Enable clocks for all the peripherals to be used
 *****************************************************************************/
void setupCMU(void)
{
  /* Ensure core frequency has been updated */
  SystemCoreClockUpdate();

  /* Set the clock frequency to 11MHz so the ADC can run on the undivided HFCLK */
  CMU_HFRCOBandSet(cmuHFRCOBand_11MHz);  
  
  /* ACMP */
  CMU_ClockEnable(cmuClock_ACMP0, true);

  /* GPIO */
  CMU_ClockEnable(cmuClock_GPIO, true);

  /* ADC */
  CMU_ClockEnable(cmuClock_ADC0, true);
  
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
    .intMode       = lesenseSetIntLevel,  /* Interrupt when voltage goes above threshold. */
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
}

/**************************************************************************//**
 * @brief Configure ADC for 12 bit mode, sample channel 0 with Vdd as reference 
 * and use shortest acquisition time.
 *****************************************************************************/
static void ADC_Config(void)
{
  
  CMU_ClockEnable(cmuClock_ADC0, true);
  
  ADC_Init_TypeDef       init       = ADC_INIT_DEFAULT;
  ADC_InitSingle_TypeDef singleInit = ADC_INITSINGLE_DEFAULT;

  /* Init common settings for both single conversion and scan mode- */
  /* Set timebase to 10, this gives 11 cycles which equals 1us at 11 MHz. */
  init.timebase = 10;

  /* Set ADC clock prescaler to 0, we are using 11MHz HFRCO, which results in HFPERCLK < 13MHz- */
  init.prescale = 0;

  ADC_Init(ADC0, &init);

  /* Init for single conversion use, measure channel 0 with Vdd as reference. */
  /* Using Vdd as reference removes the 5us warmup time for the bandgap reference. */
  singleInit.reference  = adcRefVDD;
  singleInit.input      = adcSingleInpCh5;
  
  /* Resolution can be set lower for even more energy efficient operation. */
  singleInit.resolution = adcRes8Bit;

  /* Assuming we are mesuring a low impedance source we can safely use the shortest */
  /* acquisition time. */
  singleInit.acqTime = adcAcqTime1;

  ADC_InitSingle(ADC0, &singleInit);
  
  /* Enable ADC Interrupt when Single Conversion Complete. */
  /* This is necessary for WFE (wait for event) to work. */
  /* Notice that enabling the interrupt in the NVIC is not needed. */  
  ADC0->IEN = ADC_IEN_SINGLE;
}

/**************************************************************************//**
 * @brief A separate function for taking all the samples is preferred since 
 * the whole idea is to stay in EM2 between samples. If other code is added, 
 * it might be more energy efficient to configure the ADC to use DMA while 
 * the cpu can do other work. 
 *****************************************************************************/
void doAdcSampling(uint16_t* buffer)
{  
  uint16_t sample_count = 0;
  
  /* Enable RTC, this can be enabled all the time as well if needed. */
  RTC_Enable(true);
  
  while(sample_count < BUFFER_SAMPLES)
  {    
    /* Enable deep sleep to enter EM2 between samples. */
    SCB->SCR = SCB_SCR_SEVONPEND_Msk | SCB_SCR_SLEEPDEEP_Msk;
    
    /* Go to sleep while waiting for RTC event (set by RTC_IRQ pending bit) */
    /* Since IRQ is not enabled in the NVIC, no ISR will be entered */
    __WFE();

    /* Start ADC conversion as soon as we wake up. */
    ADC_Start(ADC0, adcStartSingle);
    
    /* Clear the interrupt flag */
    RTC_IntClear(RTC_IF_COMP0);
    
    /* Clear pending RTC IRQ */
    NVIC_ClearPendingIRQ(RTC_IRQn);

    /* Wait while conversion is active in EM1, should be almost finished since it */
    /* takes 13 cycles + warmup (1us), and it was started a while ago. */
    /* Disable deep sleep so we wait in EM1 for conversion to finish. */
    SCB->SCR = SCB_SCR_SEVONPEND_Msk; 
    __WFE();
    
    /* Clear the interrupt flag */
    ADC_IntClear(ADC0, ADC_IF_SINGLE);
    
    /* Clear pending IRQ */
    NVIC_ClearPendingIRQ(ADC0_IRQn);
    
    /* Get ADC result */
    buffer[sample_count++] = ADC_DataSingleGet(ADC0);
   
  }
  
  RTC_Enable(false);
}

/***************************************************************************//**
* @brief
*   Process the sampled data through FFT.
*******************************************************************************/
void ProcessFFT(void)
{
  uint16_t        *inBuf;
  int32_t         value;
  int             i;

  inBuf = lightToFFTBuffer;

  /*
   * Convert to float values.
   */
  for (i = 0; i < BUFFER_SAMPLES; ++i)
  {
    value = (int32_t)*inBuf++;
    floatBuf[i] = (float32_t)value;
  }

  /* Process the data through the RFFT module, resulting complex output is
   * stored in fftOutputComplex
   */
  arm_rfft_f32(&rfft_instance, floatBuf, fftOutputComplex);

  /* Compute the magnitude of all the resulting complex numbers */
  arm_cmplx_mag_f32(fftOutputComplex,
                    fftOutputMag,
                    BUFFER_SAMPLES);
}

/***************************************************************************//**
* @brief
*   Find the maximal bin and estimate the frequency using sinc interpolation.
* @return
*   Frequency of maximal peak
*******************************************************************************/
float32_t GetFreq(void)
{
  float32_t maxVal;
  uint32_t maxIndex;

  /* Real and imag components of maximal bin and bins on each side */
  float32_t rz_p, iz_p, rz_n, iz_n, rz_0, iz_0;
  /* Small correction to the "index" of the maximal bin */
  float32_t deltaIndex;
  /* Real and imag components of the intermediate result */
  float32_t a, b, c, d;

  #define START_INDEX 4
  /* Find the biggest bin, disregarding the first bins because of DC offset and
   * low frequency noise.
   */
  arm_max_f32(&fftOutputMag[START_INDEX],
              BUFFER_SAMPLES / 2 - START_INDEX,
              &maxVal,
              &maxIndex);

  maxIndex += START_INDEX;

  /* Perform sinc() interpolation using the two bins on each side of the
   * maximal bin. For more information see page 113 of
   * http://tmo.jpl.nasa.gov/progress_report/42-118/118I.pdf
   */

  /* z_{peak} */
  rz_0 = fftOutputComplex[maxIndex * 2];
  iz_0 = fftOutputComplex[maxIndex * 2 + 1];

  /* z_{peak+1} */
  rz_p = fftOutputComplex[maxIndex * 2 + 2];
  iz_p = fftOutputComplex[maxIndex * 2 + 2 + 1];

  /* z_{peak-1} */
  rz_n = fftOutputComplex[maxIndex * 2 - 2];
  iz_n = fftOutputComplex[maxIndex * 2 - 2 + 1];

  /* z_{peak+1} - z_{peak-1} */
  a = rz_p - rz_n;
  b = iz_p - iz_n;
  /* z_{peak+1} + z_{peak-1} - 2*z_{peak} */
  c = rz_p + rz_n - (float32_t)2.0 * rz_0;
  d = iz_p + iz_n - (float32_t)2.0 * iz_0;

  /* Re (z_{peak+1} - z_{peak-1}) / (z_{peak+1} + z_{peak-1} - 2*z_{peak}) */
  deltaIndex = (a*c + b*d) / (c*c + d*d);

  return ((float32_t)maxIndex + deltaIndex)
          * (float32_t)SAMPLE_RATE
          / (float32_t)BUFFER_SAMPLES;
}

/***************************************************************************//**
* @brief
*   Main function. Setup ADC, FFT, clocks, PRS, DMA, Timer,
*   and process FFT forever.
*******************************************************************************/
int main(void)
{
  uint32_t    time;
  arm_status  status;

  /* Chip errata */
  CHIP_Init();

  /* Enable clocks for used peripherals */
  setupCMU();  
  
  /* Setup the ACMP */
  setupACMP();
  
  /* Setup the GPIO */
  setupGPIO();
  
  /* setup lesense */
  setupLESENSE();
  
  /* Enable LCD without voltage boost */
  SegmentLCD_Init(false);
  SegmentLCD_Symbol(LCD_SYMBOL_GECKO, 1);
  SegmentLCD_Symbol(LCD_SYMBOL_EFM32, 1);

  /* Initialize the CFFT/CIFFT module */
  status = arm_rfft_init_f32(&rfft_instance,
                             &cfft_instance,
                             BUFFER_SAMPLES,
                             0,  /* forward transform */
                             1); /* normal, not bitreversed, order */

  if (status != ARM_MATH_SUCCESS) {
    /* Error initializing RFFT module. */
    SegmentLCD_Write(" Error ");
    while (1) ;
  }
  
  /* Configure RTC to use LFXO as clock source */
  RTC_Setup(cmuSelect_LFXO);
    
  /* Configure ADC */
  ADC_Config();

  /* Enable DWT */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  /* Make sure CYCCNT is running */
  DWT->CTRL |= 1;
  
  while (1)
  {
      /* Power the light sensor with GPIO. */
      GPIO_PinModeSet( EXCITE_PIN, gpioModePushPull, 1);
      
      /* Do sampling. */
      doAdcSampling(lightToFFTBuffer);
      
      /* Power off the light sensor. */
      GPIO_PinModeSet( EXCITE_PIN, gpioModeDisabled, 0);
      
      /* Do FFT, measure number of cpu cycles used. */
      time = DWT->CYCCNT;
      ProcessFFT();
      time = DWT->CYCCNT - time;

      /* Display dominant frequency. */
      SegmentLCD_Number( (int)GetFreq() );

      /* Display cpu cycle count used to do FFT. */
      SegmentLCD_LowerNumber( (int)time );
      
      /* Check last ADC value to determine if lightlevel is too low. */
      /* Go to sleep with lesense enabled if ADC reading is below 10. */
      if(lightToFFTBuffer[BUFFER_SAMPLES-1] < 10)
      {
        
        /* Write to LCD that lightlevel is too low. */
        SegmentLCD_NumberOff();
        SegmentLCD_Write("DARK");
        
        /* Set gpio in pushpull for lesense operation. */
        GPIO_PinModeSet(LIGHTSENSE_EXCITE_PORT, LIGHTSENSE_EXCITE_PIN, gpioModePushPull, 0);
        
        LESENSE->ROUTE = LESENSE_ROUTE_ALTEX0PEN; 
        /* Start scan. */
        LESENSE_ScanStart();
        
        /* Enable deep sleep to enter EM2. */
        SCB->SCR = SCB_SCR_SEVONPEND_Msk | SCB_SCR_SLEEPDEEP_Msk;
        /* Go to sleep while waiting for LESENSE event */
        /* Since IRQ is not enabled in the NVIC, no ISR will be entered */
        __WFE();

        /* Clear interrupt flag */
        LESENSE_IntClear(LIGHTSENSE_INTERRUPT);
    
        /* Clear pending RTC IRQ */
        NVIC_ClearPendingIRQ(LESENSE_IRQn);    
        
        LESENSE_ScanStop();
        LESENSE->ROUTE &= ~LESENSE_ROUTE_ALTEX0PEN;        
        
      }
      
  }
}
