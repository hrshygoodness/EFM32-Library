/**************************************************************************//**
 * @file
 * @brief High Frequency ADC sampling using Timer and Interrupts
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

/* Uncomment to use Fahrenheit instead of Celsius */
/* #define FAHRENHEIT */

char     string[8];
uint32_t temp;

uint16_t ramBufferA[BUFFER_SIZE];
uint16_t ramBufferB[BUFFER_SIZE];
bool     usingRamBufferA;
uint32_t buffer_index;

/**************************************************************************//**
 * @brief ADC0 interrupt handler.
 *****************************************************************************/
void ADC0_IRQHandler(void)
{
  ADC_IntClear(ADC0, ADC_IF_SINGLE);

  /* Add ADC result to the current buffer */
  if (usingRamBufferA)
    ramBufferA[buffer_index++] = ADC_DataSingleGet(ADC0);
  else
    ramBufferB[buffer_index++] = ADC_DataSingleGet(ADC0);

  /* If the current buffer is full */
  if (buffer_index == BUFFER_SIZE)
  {
    buffer_index    = 0;
    usingRamBufferA = !usingRamBufferA;

    /* Set software interrupt */
    SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
  }
}

/**************************************************************************//**
 * @brief Configure ADC
 *****************************************************************************/
static void adc_setup(void)
{
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

  singleInit.acqTime = adcAcqTime32;

  ADC_InitSingle(ADC0, &singleInit);

  /* Setup interrupt generation on completed conversion. */
  ADC_IntEnable(ADC0, ADC_IF_SINGLE);
  NVIC_EnableIRQ(ADC0_IRQn);


  usingRamBufferA = true;
  buffer_index    = 0;
}

/**************************************************************************//**
 * @brief TIMER0 interrupt handler.
 *****************************************************************************/
void TIMER0_IRQHandler(void)
{
  /* Clear flag for TIMER0 overflow interrupt */
  TIMER_IntClear(TIMER0, TIMER_IF_OF);

  /* Start ADC sampling */
  ADC_Start(ADC0, adcStartSingle);
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
  uint32_t        avg = 0;

  /* Show Celsius on alphanumeric part of display */
  if (usingRamBufferA)
  {
    for (uint32_t i = 0; i < BUFFER_SIZE; i++)
    {
      avg += ramBufferA[i];
    }
  }
  else
  {
    for (uint32_t i = 0; i < BUFFER_SIZE; i++)
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
 * @brief Configure Timer
 *****************************************************************************/
void timer_setup(void)
{
  /* Enable clock for TIMER0 module */
  CMU_ClockEnable(cmuClock_TIMER0, true);

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

  /* Enable overflow interrupt */
  TIMER_IntEnable(TIMER0, TIMER_IF_OF);

  /* Enable TIMER0 interrupt vector in NVIC */
  NVIC_EnableIRQ(TIMER0_IRQn);

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

  /* Configure ADC and Timer */
  adc_setup();
  timer_setup();

  /*Enable software interrupt */
  NVIC_EnableIRQ(PendSV_IRQn);

  /* Set interrupt priorities - the software should have lower priority than
   * the ADC and the TIMER. But the TIMER and ADC should have the same priority,
   * since they need to be executed in order. */
  NVIC_SetPriority(TIMER0_IRQn, 0);                           /* Highest priority */
  NVIC_SetPriority(ADC0_IRQn, 0);                             /* Highest priority */
  NVIC_SetPriority(PendSV_IRQn, (1 << __NVIC_PRIO_BITS) - 1); /* Lowest priority */

  /* Stay in this loop forever */
  while (1)
  {
    EMU_EnterEM1();
  }
}
