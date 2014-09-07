/***************************************************************************//**
 * @file main_adc_em2_optimized.c
 *
 * @brief This software examples demonstrates how to do low speed (< 10-20KHz) 
 * ADC sampling in the most energy efficient way. EM2 is entered between samples.
 * This code is adapted to be the most energy efficient approach to EM2 ADC sampling.
 * It does not use interrupts and rely instead on a loop that should not be 
 * interrupted by other tasks. Please see the adc_em2_interrupt example for adc
 * software that can be modified more easily.
 * The break even sampling frequency compared to using the autonomous
 * approach with PRS and DMA is about 15-25 KHz. (Please see application note AN0021).
 *
 * @author Silicon Labs
 * @version 1.10
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

 
#include "em_device.h"
#include "em_cmu.h"
#include "em_chip.h"
#include "em_rtc.h"
#include "em_gpio.h"
#include "em_adc.h"

#define LFRCOFREQ 32768

/* Change this to increase or decrease sampling frequency. */
#define WAKEUP_US 1000

/* Change this to increase or decrease number of samples. */
#define N_SAMPLES 512

uint16_t sampleBuffer[N_SAMPLES];

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

  /* No division prescaler to increase accuracy. */
  CMU_ClockDivSet(cmuClock_RTC, cmuClkDiv_1);

  /* Enable clock to RTC module */
  CMU_ClockEnable(cmuClock_RTC, true);

  init.enable   = false;
  init.debugRun = false;
  init.comp0Top = true; /* Count only to top before wrapping */
  RTC_Init(&init);
  
  RTC_CompareSet(0, ((LFRCOFREQ * WAKEUP_US) / 1000000));
    
  /* Enable interrupt generation from RTC0, needed for WFE (wait for event). */
  /* Notice that enabling the interrupt in the NVIC is not needed. */
  RTC_IntEnable(RTC_IF_COMP0);
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
  singleInit.input      = adcSingleInpCh0;
  
  /* Resolution can be set lower for even more energy efficient operation. */
  singleInit.resolution = adcRes12Bit;

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
void doAdcSampling(uint16_t* buffer, uint16_t sample_count)
{  
  /* Enable RTC, this can be enabled all the time as well if needed. */
  RTC_Enable(true);
  
  while(sample_count > 0)
  {    
    /* Enable deep sleep to enter EM2 between samples. */
    SCB->SCR = 0x14;
    
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
    SCB->SCR = 0x10; 
    __WFE();
    
    /* Clear the interrupt flag */
    ADC_IntClear(ADC0, ADC_IF_SINGLE);
    
    /* Clear pending IRQ */
    NVIC_ClearPendingIRQ(ADC0_IRQn);
    
    /* Get ADC result */
    buffer[sample_count--] = ADC_DataSingleGet(ADC0);
   
  }
  
  RTC_Enable(false);
}

/**************************************************************************//**
 * @brief Main function
 *****************************************************************************/
int main(void)
{
  
  /* Chip revision alignment and errata fixes */
  CHIP_Init();
    
  /* Set the clock frequency to 11MHz so the ADC can run on the undivided HFCLK */
  CMU_HFRCOBandSet(cmuHFRCOBand_11MHz);
  
  /* Configure RTC to use LFRCO as clock source */
  RTC_Setup(cmuSelect_LFRCO);
    
  /* Configure ADC */
  ADC_Config();
 
  while(1)
  {    
    /* Function that use the ADC to convert N samples, store them in sampleBuffer, enters EM2 between samples. */
    doAdcSampling(sampleBuffer, N_SAMPLES);
  }
}  
