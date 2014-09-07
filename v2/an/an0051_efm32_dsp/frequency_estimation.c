/***************************************************************************//**
 * @file frequency_estimation.c
 * @brief Frequency estimation and Guitar tuner example for EFM32GG_DK3750
 *   Use ADC/TIMER/DMA/PRS in order to capture and analyze the audio input
 * @details
 *   signal. Run a real FFT algorithm from the CMSIS DSP Library, and estimate
 *   the frequency of the loudest tone using sinc interpolation. The main point
 *   with this example is to show the use of the CMSIS DSP library.
 *
 * @par Usage
 *   Push the AEM button on the kit until "EFM" shows in the upper right
 *   corner of the TFT display to activate the TFT and start sending tones
 *   to audio in, and the frequency should show up on the TFT
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

#include "bsp.h"
#include "bsp_trace.h"
#include "em_emu.h"
#include "em_cmu.h"
#include "em_adc.h"
#include "em_prs.h"
#include "em_timer.h"
#include "em_dma.h"
#include "dmactrl.h"
#include "rtcdrv.h"
#include "arm_math.h"
#include "glib/glib.h"
#include "tftamapped.h"

/*
 * Audio in/out handling:
 * ----------------------
 *
 *             EFM32 DVK Guitar tuner implementation overview
 *
 *                                | Audio in
 *                                V
 * +------+     +-----+       +------+       +-----+         +--------+
 * |TIMER0|---->| PRS |--+--->| ADC0 |------>| DMA |-------->| Buffer |
 * +------+     +-----+       +------+       +-----+         +--------+         
 *
 *
 * 1. TIMER0 is set to overflow appr. GUITAR_AUDIO_SAMPLE_RATE times per
 *    seconds, triggering a pulse to the PRS module on each overflow.
 *
 * 2. ADC0 is configured to trigger a scan sequence (left+right channels) when
 *    PRS pulse occurs.
 *
 * 3. The DMA is using a ping-pong transfer type to receive sampled data from the
 *    ADC0 into a buffer (right and left channel interleaved). When the specified
 *    number of samples has been received, the DMA will trigger an interrupt and
 *    switch sampling into an alternate buffer. The DMA interrupt handler only
 *    refreshes the use of the current buffer, and transfers the data into an
 *    array for the main function to process through FFT.
 *
 * Energy mode usage:
 * ------------------
 *
 * Due to the relatively high sampling rate, we need to keep the ADC warmed,
 * and only EM1 may be used when idle.
 */

/**
 * Number of samples for each channel processed at a time. This number has to be
 * equal to one of the accepted input sizes of the rfft transform of the CMSIS
 * DSP library. Increasing it gives better resolution in the frequency, but also
 * a longer sampling time.
 */
#define GUITAR_AUDIO_BUFFER_SAMPLES    512

/** (Approximate) sample rate used for processing audio data. */
#define GUITAR_AUDIO_SAMPLE_RATE      1024

/** DMA channel used for audio in scan sequence (both right and left channel) */
#define GUITAR_DMA_AUDIO_IN              0

/** PRS channel used by TIMER to trigger ADC activity. */
#define GUITAR_PRS_CHANNEL               0

/** Primary audio in buffer, holding both left and right channel (interleaved) */
static uint16_t guitarAudioInBuffer1[GUITAR_AUDIO_BUFFER_SAMPLES * 2];
/** Alternate audio in buffer, holding both left and right channel (interleaved) */
static uint16_t guitarAudioInBuffer2[GUITAR_AUDIO_BUFFER_SAMPLES * 2];

/** Buffer of uint16_t audio input values ready to be FFT-ed */
static uint16_t audioToFFTBuffer[GUITAR_AUDIO_BUFFER_SAMPLES * 2];

/** Buffer of righ-left-averaged float samples ready for FFT */
static float32_t floatBuf1[GUITAR_AUDIO_BUFFER_SAMPLES];

/** Complex (interleaved) output from FFT */
static float32_t fftOutputComplex[GUITAR_AUDIO_BUFFER_SAMPLES * 2];

/** Magnitude of complex numbers in FFT output */
static float32_t fftOutputMag[GUITAR_AUDIO_BUFFER_SAMPLES];

/** Callback config for audio-in DMA handling, must remain 'live' */
static DMA_CB_TypeDef cbInData;

/*
 * Counters used to monitor data processing, they should be equal. If
 * guitarMonProcessCount falls behind, it is an indication that the
 * system is unable to process incoming data with sufficient speed.
 */

/** Count number of interrupts on input buffer filled. */
static uint32_t guitarMonInCount;
/** Count number of times buffer has been processed. */
static uint32_t guitarMonProcessCount;

/** Flag used to indicate whether data is ready for processing */
static volatile bool dataReadyForFFT;
/** Indicate whether we are currently processing data through FFT */
static volatile bool processingFFT;

/** Instance structure for float32_t RFFT */
static arm_rfft_instance_f32 rfft_instance;
/** Instance structure for float32_t CFFT used by the RFFT */
static arm_cfft_radix4_instance_f32 cfft_instance;

/** Graphics context */
static GLIB_Context_t gc;

/** Counts 1 ms timeTicks */
static volatile uint32_t msTicks;

/**************************************************************************//**
 * @brief SysTick_Handler
 * Interrupt Service Routine for system tick counter
 *****************************************************************************/
void SysTick_Handler(void)
{
  msTicks++;       /* increment counter necessary in Delay()*/
}

/***************************************************************************//**
* @brief
*   Callback invoked from DMA interrupt handler when DMA transfer has filled
*   an audio in buffer. (I.e. received from ADC.)
*******************************************************************************/
static void guitarDMAInCb(unsigned int channel, bool primary, void *user)
{
  (void) user; /* Unused parameter */
  
  uint16_t *inBuf;
  
  inBuf = primary ? guitarAudioInBuffer1 : guitarAudioInBuffer2;
  
  /* Copy the recieved samples unless we are currently processing */
  if (!processingFFT)
  {
    /* Two channels, and each sample is 2 bytes */
    memcpy(audioToFFTBuffer, inBuf, GUITAR_AUDIO_BUFFER_SAMPLES * 2 * 2);
    dataReadyForFFT = true;
  }
  
  /* Refresh DMA for using this buffer. DMA ping-pong will */
  /* halt if buffer not refreshed in time. */
  DMA_RefreshPingPong(channel,
                      primary,
                      false,
                      NULL,
                      NULL,
                      (GUITAR_AUDIO_BUFFER_SAMPLES * 2) - 1,
                      false);

  guitarMonInCount++;
}

/***************************************************************************//**
* @brief
*   Configure ADC and DMA for this application.
*******************************************************************************/
static void guitarADCConfig(void)
{
  DMA_CfgDescr_TypeDef   descrCfg;
  DMA_CfgChannel_TypeDef chnlCfg;
  ADC_Init_TypeDef       init     = ADC_INIT_DEFAULT;
  ADC_InitScan_TypeDef   scanInit = ADC_INITSCAN_DEFAULT;

  /* Configure DMA usage by ADC */

  cbInData.cbFunc  = guitarDMAInCb;
  cbInData.userPtr = NULL;

  /* Set up high pri channel with callback with request from ADC scan complete. */ 
  chnlCfg.highPri   = true; 
  chnlCfg.enableInt = true;
  chnlCfg.select    = DMAREQ_ADC0_SCAN;
  chnlCfg.cb        = &cbInData;
  DMA_CfgChannel(GUITAR_DMA_AUDIO_IN, &chnlCfg);

  /* Configure datasize, increment and primary, alternate structure for pingpong. */
  descrCfg.dstInc  = dmaDataInc2;
  descrCfg.srcInc  = dmaDataIncNone;
  descrCfg.size    = dmaDataSize2;
  descrCfg.arbRate = dmaArbitrate1;
  descrCfg.hprot   = 0;
  DMA_CfgDescr(GUITAR_DMA_AUDIO_IN, true, &descrCfg);
  DMA_CfgDescr(GUITAR_DMA_AUDIO_IN, false, &descrCfg);

  DMA_ActivatePingPong(GUITAR_DMA_AUDIO_IN,
                       false,
                       guitarAudioInBuffer1,
                       (void *)((uint32_t) &(ADC0->SCANDATA)),
                       (GUITAR_AUDIO_BUFFER_SAMPLES * 2) - 1,
                       guitarAudioInBuffer2,
                       (void *)((uint32_t) &(ADC0->SCANDATA)),
                       (GUITAR_AUDIO_BUFFER_SAMPLES * 2) - 1);

  /* Configure ADC */

  /* Keep warm due to "high" frequency sampling */
  init.warmUpMode = adcWarmupKeepADCWarm;
  /* Init common issues for both single conversion and scan mode */
  /* 0 means, figure out the base frequency based on current cmu config. */
  init.timebase = ADC_TimebaseCalc(0); 
  /* Calculate necessary prescaler to get 4MHz adc clock with current cmu configuration. */
  init.prescale = ADC_PrescaleCalc(4000000, 0); 
  /* Sample potentiometer by tailgating in order to not disturb fixed rate */
  /* audio sampling. */
  init.tailgate = true;
  ADC_Init(ADC0, &init);

  /* Init for scan sequence use (audio in right/left channels). */
  scanInit.prsSel    = adcPRSSELCh0;
  scanInit.prsEnable = true;
  scanInit.reference = adcRefVDD;
  scanInit.input     = ADC_SCANCTRL_INPUTMASK_CH6 | ADC_SCANCTRL_INPUTMASK_CH7;
  ADC_InitScan(ADC0, &scanInit);
}

/***************************************************************************//**
* @brief
*   Configure PRS usage for this application.
*
* @param[in] prsChannel
*   PRS channel to use.
*******************************************************************************/
static void guitarPRSConfig(unsigned int prsChannel)
{
  /* Set channel to 0. */
  PRS_LevelSet(0, 1 << (prsChannel + _PRS_SWLEVEL_CH0LEVEL_SHIFT));
  /* Configure PRS channel with positive edge on Timer0 overflow as source. */
  /* Will result in a 1 HFclk pulse on the PRS channel, which can trigger the ADC. */
  PRS_SourceSignalSet(prsChannel,
                      PRS_CH_CTRL_SOURCESEL_TIMER0,
                      PRS_CH_CTRL_SIGSEL_TIMER0OF,
                      prsEdgePos);
}

/***************************************************************************//**
* @brief
*   Process the sampled audio data through FFT.
*******************************************************************************/
void processFFT(void)
{
  uint16_t        *inBuf;
  int32_t         right;
  int32_t         left;
  int             i;
  
  guitarMonProcessCount++;

  inBuf = audioToFFTBuffer;

  /* 
   * Average the left and right channels into one combined floating point buffer
   */
  for (i = 0; i < GUITAR_AUDIO_BUFFER_SAMPLES; ++i)
  {
    right = (int32_t) *inBuf++;
    left  = (int32_t) *inBuf++;

    floatBuf1[i] = (float32_t)( (float32_t)right + (float32_t)left) / 2.0f;
  }
  
  /* Process the data through the RFFT module, resulting complex output is
   * stored in fftOutputComplex
   */
  arm_rfft_f32(&rfft_instance, floatBuf1, fftOutputComplex);
  
  /* Compute the magnitude of all the resulting complex numbers */
  arm_cmplx_mag_f32(fftOutputComplex,
                    fftOutputMag,
                    GUITAR_AUDIO_BUFFER_SAMPLES);
}

/***************************************************************************//**
* @brief
*   Find the maximal bin and estimate the frequency using sinc interpolation.
* @return
*   Frequency of maximal peak
*******************************************************************************/
float32_t getFreq(void)
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
  /* Find the biggest bin, disregarding the four first because of DC offset and
   * low frequency noise.
   */
  arm_max_f32(
              &fftOutputMag[START_INDEX],
              GUITAR_AUDIO_BUFFER_SAMPLES / 2 - START_INDEX,
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
          * (float32_t)GUITAR_AUDIO_SAMPLE_RATE
          / (float32_t)GUITAR_AUDIO_BUFFER_SAMPLES;
}

/***************************************************************************//**
* @brief
*   Main function. Setup ADC, FFT, clocks, PRS, DMA, Timer,
*   and process FFT forever.
*******************************************************************************/
int main(void)
{
  arm_status status;
  char buf[20];
  bool redraw = false;
  int glibStatus;
  
  DMA_Init_TypeDef   dmaInit;
  TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;

  /* Initialize DVK board register access */
  BSP_Init(BSP_INIT_DEFAULT);

  /* If first word of user data page is non-zero, enable eA Profiler trace */
  BSP_TraceEtmSetup();

  /* Connect audio in to ADC */
  BSP_PeripheralAccess(BSP_AUDIO_IN, true);

  /* Wait a while in order to let signal from audio-in stabilize after */
  /* enabling audio-in peripheral. */
  RTCDRV_Trigger(1000, NULL);
  EMU_EnterEM2(true);
  
  /* Initialize the CFFT/CIFFT module */
  status = arm_rfft_init_f32(&rfft_instance,
                             &cfft_instance,
                             GUITAR_AUDIO_BUFFER_SAMPLES,
                             0,  /* forward transform */
                             1); /* normal, not bitreversed, order */
  
  if (status != ARM_MATH_SUCCESS) {
    /* Error initializing RFFT module. */
    while (1) ;
  }
  
  dataReadyForFFT = false;
  processingFFT   = false;
  
  /* Use the HFXO. We still only manage to process about
   * a third the buffers through FFT
   */
  CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);
  
  /* Setup SysTick Timer for 1 msec interrupts  */
  if (SysTick_Config(CMU_ClockFreqGet(cmuClock_CORE) / 1000))
  {
    while (1) ;
  }

  /* Enable clocks required */
  CMU_ClockEnable(cmuClock_HFPER, true);
  CMU_ClockEnable(cmuClock_ADC0, true);
  CMU_ClockEnable(cmuClock_PRS, true);
  CMU_ClockEnable(cmuClock_DMA, true);
  CMU_ClockEnable(cmuClock_TIMER0, true);

  NVIC_SetPriority(DMA_IRQn, 0); /* Highest priority */

  /* Configure peripheral reflex system used by TIMER to trigger ADC/DAC */
  guitarPRSConfig(GUITAR_PRS_CHANNEL);

  /* Configure general DMA issues */
  dmaInit.hprot        = 0;
  dmaInit.controlBlock = dmaControlBlock;
  DMA_Init(&dmaInit);

  /* Configure ADC used for audio-in */
  guitarADCConfig();

  /* Trigger sampling according to configured sample rate */
  TIMER_TopSet(TIMER0, CMU_ClockFreqGet(cmuClock_HFPER) / GUITAR_AUDIO_SAMPLE_RATE);
  TIMER_Init(TIMER0, &timerInit);

  /* Wait until we have control over display */
  while(!redraw)
  {
    redraw = TFT_AddressMappedInit();
  }
  
  /* Init graphics context - abort on failure */
  glibStatus = GLIB_contextInit(&gc);
  if (glibStatus != GLIB_OK) while (1) ;
  
  /* Clear the screen */
  gc.backgroundColor = GLIB_rgbColor(0, 0, 0);
  GLIB_clear(&gc);
  
  while (1)
  {
    while (dataReadyForFFT)
    {
      float32_t freq;

      processingFFT = true;
      processFFT();
      dataReadyForFFT = false;
      processingFFT = false;
      
      /* Get frequency and make string with one decimal accuracy */
      freq = getFreq();
      sprintf(buf, "%6.1f", freq);
      
      /* Check if we should control TFT display instead of AEM/board controller */
      redraw = TFT_AddressMappedInit();
      if (redraw)
      {
        gc.foregroundColor = GLIB_rgbColor(220, 220, 220);
        gc.backgroundColor = GLIB_rgbColor(0, 0, 0);

        /* Print the frequency somewhere in the middle of the screen */
        GLIB_drawString(&gc,
                        buf,
                        strlen(buf),
                        100,
                        120,
                        1);
      }
    }
    EMU_EnterEM1();
  }
}
