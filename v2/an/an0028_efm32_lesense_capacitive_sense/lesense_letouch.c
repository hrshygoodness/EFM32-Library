/**************************************************************************//**
 * @file lesense_letouch.c
 * @brief LESENSE CAPACITIVE TOUCH code example
 * @author Silicon Labs
 * @version 2.05
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

#include "em_device.h"
#include "em_cmu.h"
#include "em_acmp.h"
#include "em_gpio.h"
#include "em_rtc.h"
#include "em_lesense.h"
#include "em_int.h"
#include "lesense_letouch.h"
#include "lesense_letouch_config.h"


/* RTC nominal frequency */
#define RTC_FREQ               32768

/* LESENSE number of channels possible to use, should be 16 */
#define NUM_LESENSE_CHANNELS    16

/* GPIO Port for analog comparators */
#define LESENSE_CH_PORT         gpioPortC

static volatile uint16_t calibration_value[NUM_LESENSE_CHANNELS][NUMBER_OF_CALIBRATION_VALUES];
static volatile uint16_t buttons_pressed;
static volatile uint16_t channel_max_value[NUM_LESENSE_CHANNELS];
static volatile uint16_t channel_min_value[NUM_LESENSE_CHANNELS];

static uint16_t channels_used_mask;
static uint8_t num_channels_used;
static float channel_threshold_percent[NUM_LESENSE_CHANNELS];

/* Function prototypes */
static void LETOUCH_setupACMP(void);
static void LETOUCH_setupLESENSE(void);
static void LETOUCH_setupGPIO(void);
static void LETOUCH_setupRTC(void);
static void LETOUCH_setupCMU(void);

static uint16_t GetMaxValue(volatile uint16_t* A, uint16_t N);
static uint16_t GetMinValue(volatile uint16_t* A, uint16_t N);

/**************************************************************************//**
 * Insertion sort, can be used to implement median filter instead of just using max-value 
 * in calibration function.
*****************************************************************************/
/* static void InsertionSort(uint16_t* A, uint16_t N); */


/**************************************************************************//**
 * @brief Initializes the capacative touch system with LESENSE.
 *
 * @param[in] sensitivity
 *   An array of floats, indication of threshold level, in percent,
 *   of nominal count value.
 *
 *****************************************************************************/
void LETOUCH_Init(float sensitivity[]){

  uint8_t i;
  channels_used_mask = 0;
  num_channels_used = 0;
  
  /* Initialize channels used mask and threshold array for each channel */
  /* Uses the sensitivity array to deduce which channels to enable and how */
  /* many channels that are enabled */

  for(i = 0; i < NUM_LESENSE_CHANNELS; i++){
    
    /* Init min and max values for each channel */
    channel_max_value[i] = 0;
    channel_min_value[i] = 0xffff;
    
    /* Add to channels used mask if sensitivity is not zero */
    if(sensitivity[i] != 0.0){
      channel_threshold_percent[i] = sensitivity[i];
      channels_used_mask |= (1 << i);
      num_channels_used++;
    }
  }
      
  /* Disable interrupts while initializing */
  INT_Disable();

  /* Setup CMU. */
  LETOUCH_setupCMU();
  /* Setup GPIO. */
  LETOUCH_setupGPIO();
  /* Setup ACMP. */
  LETOUCH_setupACMP();
  /* Setup LESENSE. */
  LETOUCH_setupLESENSE();
  /* Do initial calibration "N_calibration_values * 10" times to make sure */
  /* it settles on values after potential startup transients */
  for(i = 0; i < NUMBER_OF_CALIBRATION_VALUES * 10; i++){
    LESENSE_ScanStart();
    LETOUCH_Calibration();
  }
  /* Setup RTC for calibration interrupt */
  LETOUCH_setupRTC();
  /* Initialization done, enable interrupts globally. */
  INT_Enable();

}

/***************************************************************************//**
 * @brief
 *   Get the buttons pressed variable, one bit for each channel pressed 
 *   or'ed together.
 *
 * @return
 *   The touch buttons/pads that are in touched state is or'ed together and
 *   returned.
 ******************************************************************************/
uint16_t LETOUCH_GetChannelsTouched(void){
  
  return buttons_pressed;
}

/***************************************************************************//**
 * @brief
 *   Get the maximum value registered for a given channel.
 *
 * @param[in] channel
 *   The channel to get maximum value for
 *
 * @return
 *   The maximum value registered for the given channel.
 ******************************************************************************/
uint16_t LETOUCH_GetChannelMaxValue(uint8_t channel){

  return channel_max_value[channel];
}

/***************************************************************************//**
 * @brief
 *   Get the minimum value registered for a given channel.
 *
 * @param[in] channel
 *   The channel to get minimum value for
 *
 * @return
 *   The minimum value registered for the given channel.
 ******************************************************************************/
uint16_t LETOUCH_GetChannelMinValue(uint8_t channel){
 
  return channel_min_value[channel];
}

/**************************************************************************//**
 * @brief  Enable clocks for all the peripherals to be used
 *****************************************************************************/
static void LETOUCH_setupCMU( void )
{
  /* Ensure core frequency has been updated */
  SystemCoreClockUpdate();

  /* ACMP */
  CMU_ClockEnable(cmuClock_ACMP0, true);
  CMU_ClockEnable(cmuClock_ACMP1, true);
  
  /* GPIO */
  CMU_ClockEnable(cmuClock_GPIO, true);

 /* Low energy peripherals
 *   LESENSE
 *   LFXO clock must be enabled prior to enabling
 *   clock for the low energy peripherals */
  CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);
  CMU_ClockEnable(cmuClock_CORELE, true);
  CMU_ClockEnable(cmuClock_LESENSE, true);

  /* RTC */
  CMU_ClockEnable(cmuClock_RTC, true);

  /* Disable clock source for LFB clock */
  CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_Disabled);
}

/**************************************************************************//**
 * @brief  Sets up the ACMP
 *****************************************************************************/
static void LETOUCH_setupACMP( void )
{
  /* Configuration structure for ACMP */
  /* See application note document for description of the different settings. */
  static const ACMP_CapsenseInit_TypeDef acmpInit =
  {
    .fullBias                 = true,            //Configured according to application note
    .halfBias                 = true,            //Configured according to application note 
    .biasProg                 = 0x5,             //Configured according to application note
    .warmTime                 = acmpWarmTime512, //LESENSE uses a fixed warmup time
    .hysteresisLevel          = acmpHysteresisLevel5, //Configured according to application note
    .resistor                 = acmpResistor0,   //Configured according to application note   
    .lowPowerReferenceEnabled = false,           //LP-reference can introduce glitches with captouch
    .vddLevel                 = 0x30,            //Configured according to application note 
    .enable                   = false            //LESENSE enables the ACMP
  };
  
  /* Initialize ACMP in capsense mode*/
  ACMP_CapsenseInit(ACMP0, &acmpInit);
  ACMP_CapsenseInit(ACMP1, &acmpInit);

}

/**************************************************************************//**
 * @brief  Sets up the LESENSE
 *****************************************************************************/
static void LETOUCH_setupLESENSE( void )
{
  uint8_t i;
  
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
      .startDelay     = 0x0
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
      .acmp0Mode      = lesenseACMPModeMux,   // only acmp mux controlled by lesense
      .acmp1Mode      = lesenseACMPModeMux,   // only acmp mux controlled by lesense
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
      .prsCount  = false,
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
    .enaInt        = true,
    .chPinExMode   = lesenseChPinExDis,
    .chPinIdleMode = lesenseChPinIdleDis,
    .useAltEx      = false,
    .shiftRes      = false,
    .invRes        = false,
    .storeCntRes   = true,
    .exClk         = lesenseClkLF,
    .sampleClk     = lesenseClkLF,
    .exTime        = 0x0,
    .sampleDelay   = SAMPLE_DELAY,
    .measDelay     = 0x0,
    .acmpThres     = 0x0,                   // don't care, configured by ACMPInit
    .sampleMode    = lesenseSampleModeCounter,
    .intMode       = lesenseSetIntLevel,
    .cntThres      = 0x0,                   // Configured later by calibration function
    .compMode      = lesenseCompModeLess
  };

  /* Initialize LESENSE interface _with_ RESET. */
  LESENSE_Init(&initLesense, true);

  /* Configure channels */
  for(i = 0; i < NUM_LESENSE_CHANNELS; i++){
    if((channels_used_mask >> i) & 0x1){
      LESENSE_ChannelConfig(&initLesenseCh, i);
    }
  }
    
  /* Set scan frequency */
  LESENSE_ScanFreqSet(0, LESENSE_SCAN_FREQUENCY);

  /* Set clock divisor for LF clock. */
  LESENSE_ClkDivSet(lesenseClkLF, lesenseClkDiv_1);

  /* Enable interrupt in NVIC. */
  NVIC_EnableIRQ(LESENSE_IRQn);

  /* Start scan. */
  LESENSE_ScanStart();
}

/**************************************************************************//**
 * @brief  Sets up the GPIO
 *****************************************************************************/
static void LETOUCH_setupGPIO( void )
{
  unsigned int i;
  
  /* Set GPIO pin mode to disabled for all active pins */
  for(i = 0; i < NUM_LESENSE_CHANNELS; i++){
    if((channels_used_mask >> i) & 0x1){
      GPIO_PinModeSet(LESENSE_CH_PORT, i, gpioModeDisabled, 0);
    }
  }
}

/**************************************************************************//**
 * @brief  Sets up the RTC
 *****************************************************************************/
void LETOUCH_setupRTC( void )
{
  /* RTC configuration */
  static const RTC_Init_TypeDef rtcInit =
  {
    .enable   = true,
    .debugRun = false,
    .comp0Top = true
  };

  RTC_Init(&rtcInit);

  /* Set the RTC calibration interrupt compare value */
  /* calibration interval defined in lesense_letouch_config.h */
  RTC_CompareSet( 0, CALIBRATION_INTERVAL * RTC_FREQ );

  RTC_IntEnable(RTC_IFS_COMP0);
  NVIC_EnableIRQ(RTC_IRQn);
}

/**************************************************************************//**
 * Calibration function 
*****************************************************************************/
void LETOUCH_Calibration( void ){
  int i,k;
  uint16_t nominal_count;
  static uint8_t calibration_value_index = 0;
  
  /* Wait for current scan to finish */
  while(LESENSE->STATUS & LESENSE_STATUS_SCANACTIVE); 
  
  /* Get position for first channel data in count buffer from lesense write pointer */
  k = ((LESENSE->PTR & _LESENSE_PTR_WR_MASK) >> _LESENSE_PTR_WR_SHIFT);
  
  /* Handle circular buffer wraparound */
  if(k >= num_channels_used){
    k = k - num_channels_used;
  }
  else{
    k = k - num_channels_used + NUM_LESENSE_CHANNELS;
  }
  
  /* Fill calibration values array with buffer values */
  for(i = 0; i < NUM_LESENSE_CHANNELS; i++){
    if((channels_used_mask >> i) & 0x1){
      calibration_value[i][calibration_value_index] = LESENSE_ScanResultDataBufferGet(k++);
    }
  }
 
  /* Wrap around calibration_values_index */
  calibration_value_index++;
  if(calibration_value_index >= NUMBER_OF_CALIBRATION_VALUES){
    calibration_value_index = 0;
  }
  
  /* Calculate max/min-value for each channel and set threshold */
  for(i = 0; i < NUM_LESENSE_CHANNELS; i++){
    if((channels_used_mask >> i) & 0x1){
      channel_max_value[i] = GetMaxValue(calibration_value[i], NUMBER_OF_CALIBRATION_VALUES);
      channel_min_value[i] = GetMinValue(calibration_value[i], NUMBER_OF_CALIBRATION_VALUES);
      
      nominal_count = channel_max_value[i];
      LESENSE_ChannelThresSet(i, 0x0,(uint16_t) (nominal_count - ((nominal_count * channel_threshold_percent[i])/100.0)) ); 
    }
  }
}

/**************************************************************************//**
 * Insertion sort, can be used to implement median filter instead of just using max-value 
 * in calibration function.
*****************************************************************************/
/*
static void InsertionSort(uint16_t* A, uint16_t N){
  int i,j;
  uint16_t temp;
    
  for(i=1; i<N; i++)
  {
    temp = A[i];
    j = i-1;
    while(temp<A[j] && j>=0)
    {
      A[j+1] = A[j];
      j = j-1;
    }
    A[j+1] = temp;
  } 
}
*/

/**************************************************************************//**
 * Returns maximum value in input array of size N 
*****************************************************************************/
static uint16_t GetMaxValue(volatile uint16_t* A, uint16_t N){
  int i;
  uint16_t max = 0;
    
  for(i=0; i<N; i++)
  {
    if(max < A[i]){
      max = A[i];
    }
  } 
  return max;
}

/**************************************************************************//**
 * Returns minimum value in input array of size N 
*****************************************************************************/
static uint16_t GetMinValue(volatile uint16_t* A, uint16_t N){
  int i;
  uint16_t min = 0xffff;
    
  for(i=0; i<N; i++)
  {
    if(A[i] < min){
      min = A[i];
    }
  } 
  return min;
}

/**************************************************************************//**
 * Interrupt handlers
 *****************************************************************************/

/**************************************************************************//**
 * @brief RTC_IRQHandler
 * Interrupt Service Routine for RTC, used for the calibration function
 *****************************************************************************/
void RTC_IRQHandler( void )
{   
  /* Clear interrupt flag */
  RTC_IntClear(RTC_IFS_COMP0);
  
  LETOUCH_Calibration();
  
  /* Reset counter */
  RTC_CounterReset();
}

/**************************************************************************//**
 * @brief LESENSE_IRQHandler
 * Interrupt Service Routine for LESENSE Interrupt Line
 *****************************************************************************/
void LESENSE_IRQHandler( void )
{
  uint8_t channel, i, valid_touch;
  uint32_t interrupt_flags, tmp, channels_enabled;
  uint16_t threshold_value;
  
  /* Get interrupt flag */
  interrupt_flags = LESENSE_IntGet();
  /* Clear interrupt flag */
  LESENSE_IntClear(interrupt_flags);
  
  /* Interrupt handles only one channel at a time */ 
  /* therefore only first active channel found is handled by the interrupt. */
  for(channel = 0; channel < NUM_LESENSE_CHANNELS; channel++){
    if( (interrupt_flags >> channel) & 0x1 ){
      break;
    }
  }
  
  /* To filter out possible false touches, the suspected channel is measured several times */
  /* All samples should be below threshold to trigger an actual touch. */

  /* Disable other channels. */
  channels_enabled = LESENSE->CHEN;
  LESENSE->CHEN = 1 << channel;
  
   /* Evaluate VALIDATE_CNT results for touched channel. */
  valid_touch = 1;
  for(i = 0;i<VALIDATE_CNT;i++){
    /* Start new scan and wait while active. */
    LESENSE_ScanStart();
    while(LESENSE->STATUS & LESENSE_STATUS_SCANACTIVE);
    
    tmp = LESENSE->SCANRES;
    if((tmp & (1 << channel)) == 0){
      valid_touch = 0;
    }
  }  
    
  /* Enable all channels again. */
  LESENSE->CHEN = channels_enabled;
  
  
  if(valid_touch){
    /* If logic was switched clear button flag and set logic back, else set button flag and invert logic. */
    if(LESENSE->CH[channel].EVAL & LESENSE_CH_EVAL_COMP){
      buttons_pressed &= ~(1 << channel);
      LESENSE->CH[channel].EVAL &= ~LESENSE_CH_EVAL_COMP;
      
      threshold_value = LESENSE->CH[channel].EVAL & (_LESENSE_CH_EVAL_COMPTHRES_MASK);
      /* Change threshold value 1 LSB for hysteresis. */
      threshold_value -= 1;
      LESENSE_ChannelThresSet(channel, 0, threshold_value);
    }
    else{
      buttons_pressed |= (1 << channel);
      LESENSE->CH[channel].EVAL |= LESENSE_CH_EVAL_COMP;
      
      threshold_value = LESENSE->CH[channel].EVAL & (_LESENSE_CH_EVAL_COMPTHRES_MASK);
      /* Change threshold value 1 LSB for hysteresis. */
      threshold_value += 1;
      LESENSE_ChannelThresSet(channel, 0, threshold_value);
    }
  }
  
  /* Need to reset RTC counter so we don't get new calibration event right after buttons are pushed/released. */
  RTC_CounterReset();
  
}



