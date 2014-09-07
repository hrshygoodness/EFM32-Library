/**************************************************************************//**
 * @file
 * @brief High Frequaency ADC Sampling using Timer, DMA and PRS
 * @author Energy Micro AS
 * @version 1.0.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silicon Labs Software License Agreement. See 
 * "http://developer.silabs.com/legal/version/v11/Silicon_Labs_Software_License_Agreement.txt"  
 * for details. Before using this software for any purpose, you must agree to the 
 * terms of that agreement.
 *
 ******************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include "efm32.h"
#include "em_chip.h"
#include "em_emu.h"
#include "em_cmu.h"
#include "em_adc.h"
#include "em_lcd.h"
#include "em_prs.h"
#include "em_dma.h"
#include "em_timer.h"

/* Drivers */
#include "segmentlcd.h"

/* Support functions */
#include "support.h"

/* Defines */
#define ADC_SAMPLE_RATE       100000
#define HFPER                 14000000
#define TIMER_TOP             (HFPER / ADC_SAMPLE_RATE)
#define BUFFER_SIZE_SHIFT     9
#define BUFFER_SIZE           (1 << BUFFER_SIZE_SHIFT)
#define BUFFERS_PER_SECOND    (ADC_SAMPLE_RATE / BUFFER_SIZE)
#define DMA_CHANNEL           0
#define PRS_CHANNEL           0

/* Uncomment to use Fahrenheit instead of Celsius */
/* #define FAHRENHEIT */

/* DMA control block, must be aligned to 256. */
#if defined (__ICCARM__)
#pragma data_alignment=256
DMA_DESCRIPTOR_TypeDef dmaControlBlock[DMA_CHAN_COUNT * 2];
#elif defined (__CC_ARM)
DMA_DESCRIPTOR_TypeDef dmaControlBlock[DMA_CHAN_COUNT * 2] __attribute__ ((aligned(256)));
#elif defined (__GNUC__)
DMA_DESCRIPTOR_TypeDef dmaControlBlock[DMA_CHAN_COUNT * 2] __attribute__ ((aligned(256)));
#else
#error Undefined toolkit, need to define alignment
#endif

/* Variables */
uint16_t              ramBufferA[BUFFER_SIZE];
uint16_t              ramBufferB[BUFFER_SIZE];
bool                  usingRamBufferA;
static DMA_CB_TypeDef cbInData;
char                  string[8];

/**************************************************************************//**
 * @brief ADC0 interrupt handler. Simply clears interrupt flag.
 *****************************************************************************/
static void preampDMAInCb(unsigned int channel, bool primary, void *user)
{
  (void) user; /* Unused parameter */

  /* Refresh DMA */
  DMA_RefreshPingPong(channel,     /* channel */
                      primary,         /* descriptor to update */
                      false,           /* do not use burst */
                      NULL,            /* do not change destination */
                      NULL,            /* do not change source */
                      BUFFER_SIZE - 1, /* number of transfers */
                      false);          /* do not stop after completing this cycle */

  /* Switch buffer */
  usingRamBufferA = primary;

  /* Set software interrupt */
  SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
}

/**************************************************************************//**
 * @brief Software interrupt handler
 * Computes and displays the average temperature
 *****************************************************************************/
void PendSV_Handler(void)
{
  static bool     enableAntenna = false;
  static uint32_t display_count = 0;
  int32_t         temp;
  uint32_t        i;

  uint32_t        avg = 0;
  /* Show Celsius on alphanumeric part of display */
  if (usingRamBufferA)
  {
    for (i = 0; i < BUFFER_SIZE; i++)
    {
      avg += ramBufferA[i];
    }
  }
  else
  {
    for (i = 0; i < BUFFER_SIZE; i++)
    {
      avg += ramBufferB[i];
    }
  }

  /* The buffer size is a power of two, so rightshift can be used instead of
   * division */
  avg = avg >> BUFFER_SIZE_SHIFT;
  
#if defined(_EFM32_GIANT_FAMILY)
  avg += 112;
#endif

  /* Show Celsius or Fahrenheit on alphanumeric part of display */
#ifndef FAHRENHEIT
  temp = (int32_t)(convertToCelsius(avg) * 10);
  snprintf(string, 8, "%2d,%1d%%C", (temp / 10), abs(temp) % 10);
#else
  temp = (int32_t)(convertToFahrenheit(avg) * 10);
  snprintf(string, 8, "%2d,%1d%%F", (temp / 10), abs(temp) % 10);
#endif

  /* Display result on LCD once a second */
  if (++display_count == BUFFERS_PER_SECOND)
  {
    display_count = 0;

    SegmentLCD_Write(string);

    /* Indicate update */
    SegmentLCD_Symbol(LCD_SYMBOL_ANT, enableAntenna);
    enableAntenna = !enableAntenna;
  }
}

/**************************************************************************//**
 * @brief Configure ADC with DMA
 *****************************************************************************/
static void adc_setup(void)
{
  /* Configure ADC single conversion */
  CMU_ClockEnable(cmuClock_ADC0, true);

  ADC_Init_TypeDef       init       = ADC_INIT_DEFAULT;
  ADC_InitSingle_TypeDef singleInit = ADC_INITSINGLE_DEFAULT;

  init.timebase   = ADC_TimebaseCalc(0);
  init.warmUpMode = adcWarmupKeepADCWarm;

  init.prescale = ADC_PrescaleCalc(7000000, 0);
  ADC_Init(ADC0, &init);

  singleInit.reference  = adcRef1V25;
  singleInit.input      = adcSingleInpTemp;
  singleInit.resolution = adcRes12Bit;
  singleInit.prsSel     = adcPRSSELCh0;
  singleInit.prsEnable  = true;

  singleInit.acqTime = adcAcqTime32;

  ADC_InitSingle(ADC0, &singleInit);

  /* Configure DMA usage by ADC */
  CMU_ClockEnable(cmuClock_DMA, true);

  DMA_CfgDescr_TypeDef   descrCfg;
  DMA_CfgChannel_TypeDef chnlCfg;

  cbInData.cbFunc  = preampDMAInCb;
  cbInData.userPtr = NULL;

  chnlCfg.highPri   = true;
  chnlCfg.enableInt = true;
  chnlCfg.select    = DMAREQ_ADC0_SINGLE;
  chnlCfg.cb        = &cbInData;
  DMA_CfgChannel(0, &chnlCfg);

  descrCfg.dstInc  = dmaDataInc2;
  descrCfg.srcInc  = dmaDataIncNone;
  descrCfg.size    = dmaDataSize2;
  descrCfg.arbRate = dmaArbitrate1;
  descrCfg.hprot   = 0;
  DMA_CfgDescr(DMA_CHANNEL, true, &descrCfg);
  DMA_CfgDescr(DMA_CHANNEL, false, &descrCfg);

  DMA_ActivatePingPong(DMA_CHANNEL,                              /* channel */
                       false,                                    /* do not use burst */
                       (void *) &ramBufferA,                     /* primary destination */
                       (void *)((uint32_t) &(ADC0->SINGLEDATA)), /* primary source */
                       BUFFER_SIZE - 1,                          /* primary number of transfers */
                       (void *) &ramBufferB,                     /*altenate deststination */
                       (void *)((uint32_t) &(ADC0->SINGLEDATA)), /* alternate source */
                       BUFFER_SIZE - 1);                         /* alternate number of transfers */

  /* Select ramBufferA as current */
  usingRamBufferA = true;
}

/**************************************************************************//**
 * @brief Configure PRS
 *****************************************************************************/
static void prs_setup(unsigned int prsChannel)
{
  PRS_LevelSet(0, 1 << (prsChannel + _PRS_SWLEVEL_CH0LEVEL_SHIFT));
  PRS_SourceSignalSet(prsChannel,
                      PRS_CH_CTRL_SOURCESEL_TIMER0,
                      PRS_CH_CTRL_SIGSEL_TIMER0OF,
                      prsEdgePos);
}


/**************************************************************************//**
 * @brief Configure Timer
 *****************************************************************************/
void timer_setup(void)
{
  /* Enable clock for TIMER0 module */
  CMU_ClockEnable(cmuClock_TIMER0, true);

  CMU_ClockEnable(cmuClock_PRS, true);

  /* Select TIMER0 parameters */
  TIMER_Init_TypeDef timerInit =
  {
    .enable     = true,
    .debugRun   = true,
    .prescale   = timerPrescale1,
    .clkSel     = timerClkSelHFPerClk,
    .fallAction = timerInputActionNone,
    .riseAction = timerInputActionNone,
    .mode       = timerModeUp,
    .dmaClrAct  = false,
    .quadModeX4 = false,
    .oneShot    = false,
    .sync       = false,
  };

  /* Set TIMER Top value */
  TIMER_TopSet(TIMER0, TIMER_TOP);

  /* Configure TIMER */
  TIMER_Init(TIMER0, &timerInit);
}

/**************************************************************************//**
 * @brief SWO Setup
 * Enables code view in energyAware Profiler
 *****************************************************************************/
void setupSWO(void)
{
  uint32_t *dwt_ctrl = (uint32_t *) 0xE0001000;
  uint32_t *tpiu_prescaler = (uint32_t *) 0xE0040010;
  uint32_t *tpiu_protocol = (uint32_t *) 0xE00400F0;

  CMU->HFPERCLKEN0 |= CMU_HFPERCLKEN0_GPIO;
  /* Enable Serial wire output pin */
  GPIO->ROUTE |= GPIO_ROUTE_SWOPEN;
#if defined(_EFM32_GIANT_FAMILY)
  /* Set location 0 */
  GPIO->ROUTE = (GPIO->ROUTE & ~(_GPIO_ROUTE_SWLOCATION_MASK)) | GPIO_ROUTE_SWLOCATION_LOC0;

  /* Enable output on pin - GPIO Port F, Pin 2 */
  GPIO->P[5].MODEL &= ~(_GPIO_P_MODEL_MODE2_MASK);
  GPIO->P[5].MODEL |= GPIO_P_MODEL_MODE2_PUSHPULL;
#else
  /* Set location 1 */
  GPIO->ROUTE = (GPIO->ROUTE & ~(_GPIO_ROUTE_SWLOCATION_MASK)) | GPIO_ROUTE_SWLOCATION_LOC1;
  /* Enable output on pin */
  GPIO->P[2].MODEH &= ~(_GPIO_P_MODEH_MODE15_MASK);
  GPIO->P[2].MODEH |= GPIO_P_MODEH_MODE15_PUSHPULL;
#endif
  /* Enable debug clock AUXHFRCO */
  CMU->OSCENCMD = CMU_OSCENCMD_AUXHFRCOEN;

  while(!(CMU->STATUS & CMU_STATUS_AUXHFRCORDY));

  /* Enable trace in core debug */
  CoreDebug->DHCSR |= 1;
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

  /* Enable PC and IRQ sampling output */
  *dwt_ctrl = 0x400113FF;
  /* Set TPIU prescaler to 16. */
  *tpiu_prescaler = 0xf;
  /* Set protocol to NRZ */
  *tpiu_protocol = 2;
  /* Unlock ITM and output data */
  ITM->LAR = 0xC5ACCE55;
  ITM->TCR = 0x10009;
}


/******************************************************************************
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
  /* Initialize chip */
  CHIP_Init();
  
  /* Enable code view */
  setupSWO();

  SegmentLCD_Init(false);

  /* Enable clocks required */
  CMU_ClockEnable(cmuClock_HFPER, true);


  DMA_Init_TypeDef dmaInit;
  dmaInit.hprot        = 0;
  dmaInit.controlBlock = dmaControlBlock;
  DMA_Init(&dmaInit);

  /* Configure Timer, PRS and ADC */
  timer_setup();
  prs_setup(PRS_CHANNEL);
  adc_setup();

  /* Enable software interrupt */
  NVIC_EnableIRQ(PendSV_IRQn);

  /* Set priorities */
  NVIC_SetPriority(DMA_IRQn, 0);                              /* Highest priority */
  NVIC_SetPriority(PendSV_IRQn, (1 << __NVIC_PRIO_BITS) - 1); /* Lowest priority */

  /* Stay in this loop forever */
  while (1)
  {
    EMU_EnterEM1();
  }
}
