/******************************************************************************
 * @file main_adc_scan.c
 * @brief ADC scan conversion example
 * @author Silicon Labs
 * @version 1.10
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


 #include <stdlib.h>
#include <stdio.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_emu.h"
#include "em_cmu.h"
#include "em_dma.h"
#include "em_adc.h"
#include "em_lcd.h"
#include "segmentlcd.h"
#include "rtcdrv.h"
#include "dmactrl.h"

/*******************************************************************************
 *******************************   DEFINES   ***********************************
 ******************************************************************************/

/** DMA channel used for scan sequence sampling adc channel 2, 3 and 4. */
#define DMA_CHANNEL    0
#define NUM_SAMPLES    3

/*******************************************************************************
 ***************************   LOCAL FUNCTIONS   *******************************
 ******************************************************************************/

/***************************************************************************//**
* @brief
*   Configure ADC for scan mode.
*******************************************************************************/
static void ADCConfig(void)
{
  ADC_Init_TypeDef     init     = ADC_INIT_DEFAULT;
  ADC_InitScan_TypeDef scanInit = ADC_INITSCAN_DEFAULT;

  /* Init common issues for both single conversion and scan mode */
  init.timebase = ADC_TimebaseCalc(0);
  init.prescale = ADC_PrescaleCalc(7000000, 0);
  ADC_Init(ADC0, &init);

  /* Init for scan sequence use ( for dvk: accelerometer X, Y and Z axis). */
  scanInit.reference = adcRefVDD;
  scanInit.input     = ADC_SCANCTRL_INPUTMASK_CH2 |
                       ADC_SCANCTRL_INPUTMASK_CH3 |
                       ADC_SCANCTRL_INPUTMASK_CH4;
  ADC_InitScan(ADC0, &scanInit);
}


/***************************************************************************//**
* @brief
*   Configure DMA usage for this application.
*******************************************************************************/
static void DMAConfig(void)
{
  DMA_Init_TypeDef       dmaInit;
  DMA_CfgDescr_TypeDef   descrCfg;
  DMA_CfgChannel_TypeDef chnlCfg;

  /* Configure general DMA issues */
  dmaInit.hprot        = 0;
  dmaInit.controlBlock = dmaControlBlock;
  DMA_Init(&dmaInit);

  /* Configure DMA channel used */
  chnlCfg.highPri   = false;
  chnlCfg.enableInt = false;
  chnlCfg.select    = DMAREQ_ADC0_SCAN;
  chnlCfg.cb        = NULL;
  DMA_CfgChannel(DMA_CHANNEL, &chnlCfg);

  descrCfg.dstInc  = dmaDataInc4;
  descrCfg.srcInc  = dmaDataIncNone;
  descrCfg.size    = dmaDataSize4;
  descrCfg.arbRate = dmaArbitrate1;
  descrCfg.hprot   = 0;
  DMA_CfgDescr(DMA_CHANNEL, true, &descrCfg);
}


/******************************************************************************
 * @brief  Main function
 * The main function sets up direct memory transfer and adc in scan mode,
 * the adc then scans 3 channels sends the data to memory through DMA in EM1.
 *****************************************************************************/
int main(void)
{
  /* Initialize chip */
  CHIP_Init();
  SegmentLCD_Init(false);

  SYSTEM_ChipRevision_TypeDef chipRev;
  int                         errataShift = 0;

  /* ADC errata for rev B when using VDD as reference, need to multiply */
  /* result by 2 */
  SYSTEM_ChipRevisionGet(&chipRev);
  if ((chipRev.major == 1) && (chipRev.minor == 1))
  {
    errataShift = 1;
  }

  /* Enable clocks required */
  CMU_ClockEnable(cmuClock_HFPER, true);
  CMU_ClockEnable(cmuClock_ADC0, true);
  CMU_ClockEnable(cmuClock_DMA, true);

  /* Configure ADC and DMA used for scanning channel 2, 3 and 4 */
  ADCConfig();
  DMAConfig();

  uint32_t samples[NUM_SAMPLES];
  int      i;

  /* Stay in this loop forever */
  while (1)
  {
    DMA_ActivateBasic(DMA_CHANNEL,
                      true,
                      false,
                      samples,
                      (void *)((uint32_t) &(ADC0->SCANDATA)),
                      NUM_SAMPLES - 1);

    /* Start Scan */
    ADC_Start(ADC0, adcStartScan);

    /* Poll for scan comversion complete */
    while (ADC0->STATUS & ADC_STATUS_SCANACT);

    if (errataShift)
    {
      for (i = 0; i < NUM_SAMPLES; i++)
      {
        samples[i] <<= errataShift;
      }
    }


    /* Format numbers and write to LCD */
    char buffer[10];
    sprintf(buffer, "%02d%02d%02d", (unsigned int) (samples[0] >> 6), (unsigned int) (samples[1] >> 6), (unsigned int) (samples[2] >> 6));
    SegmentLCD_Write(buffer);

    /* wait 100ms in EM2 before next conversion */
    RTCDRV_Trigger(100, NULL);
    EMU_EnterEM2(true);
  }
}



