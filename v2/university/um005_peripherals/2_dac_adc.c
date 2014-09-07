/**************************************************************************//**
 * @file
 * @brief DAC ADC Example
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
#include <stdio.h>
#include "efm32.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_dac.h"
#include "em_adc.h"
#include "em_gpio.h"
#include "em_lcd.h"
#include "segmentlcd.h"
#include "rtcdrv.h"

/* Uncomment the define below to use an external wire between the DAC and ADC
 * instead of the internal connection */
/* #define EXTERNAL */

/* Function prototypes */
void DAC_setup(void);
void DAC_WriteData(DAC_TypeDef *dac, unsigned int value, unsigned int ch);
static void ADCConfig(void);

/**************************************************************************//**
 * @brief  Main function
 * The DAC produces an output signal which the ADC measures. The measured
 * result is printed on the LCD screen.
 *
 * The code shows two different examples of ADC input:
 *
 * One version uses DAC output channel 0 as input.
 *
 * The other version uses ADC input channel 3 (PD3 on the STK). This pin must
 * then be connected with a wire to the DAC output channel 0 (PB11 on the STK).
 *
 *****************************************************************************/
int main(void)
{
  /* Variable declarations */
  uint32_t DAC_Value;
  uint32_t sample;
  double   voltage;

  /* Initialize chip */
  CHIP_Init();

  /* Initialize LCD */
  SegmentLCD_Init(false);

  /* Enable clocks required */
  CMU_ClockEnable(cmuClock_HFPER, true);
  CMU_ClockEnable(cmuClock_ADC0, true);

  /* Configure ADC */
  ADCConfig();

  /* Initialize the DAC */
  DAC_setup();

  /* Enable DAC channel 0, located on pin PB11 */
  DAC_Enable(DAC0, 0, true);


  /* This code is only necessary when using ADC input channel 3 which
   * is connected to pin PD3 on the STK. Enable it by uncommenting the
   * definition of EXTERNAL above */
#ifdef EXTERNAL
  CMU_ClockEnable(cmuClock_GPIO, true);
  GPIO_PinModeSet(gpioPortD, 3, gpioModeInputPull, 1);
  GPIO_PinOutClear(gpioPortD, 3);
#endif

  /* Stay in this loop forever */
  while (1)
  {
    /* Calculate DAC output to 0.5 V. */
    DAC_Value = (uint32_t)((0.5 * 4096) / 1.25);

    /* Write the new value to DAC register */
    DAC_WriteData(DAC0, DAC_Value, 0);

    /* Start single conversion for ADC */
    ADC_Start(ADC0, adcStartSingle);

    /* Wait while conversion is active */
    while (ADC0->STATUS & ADC_STATUS_SINGLEACT) ;

    /* Get ADC result */
    sample = ADC_DataSingleGet(ADC0);

    /* Calculate output voltage produced by the DAC  */
    voltage = (sample * 1.25) / 4096;

    /* Write the result to LCD */
    char buffer[10];
    snprintf(buffer, 8, "%1.2f", voltage);
    SegmentLCD_Write(buffer);

    /* wait 100ms in EM2 before next conversion */
    RTCDRV_Trigger(100, NULL);
    EMU_EnterEM2(true);
  }
}


/**************************************************************************//**
 * @brief  Setup DAC
 * Configures and starts the DAC
 *****************************************************************************/
void DAC_setup(void)
{
  /* Use default settings */
  DAC_Init_TypeDef        init        = DAC_INIT_DEFAULT;
  DAC_InitChannel_TypeDef initChannel = DAC_INITCHANNEL_DEFAULT;

  /* Enable the DAC clock */
  CMU_ClockEnable(cmuClock_DAC0, true);

  /* Calculate the DAC clock prescaler value that will result in a DAC clock
   * close to 500kHz. Second parameter is zero, if the HFPERCLK value is 0, the
   * function will check what the current value actually is. */
  init.prescale = DAC_PrescaleCalc(500000, 0);

  /* Set reference voltage to 1.25V */
  init.reference = dacRef1V25;

  /* Set output mode for DAC such that the DAC can produce output to both
   * pin and ADC. */
  init.outMode = dacOutputPinADC;

  /* Initialize the DAC and DAC channel. */
  DAC_Init(DAC0, &init);
  DAC_InitChannel(DAC0, &initChannel, 0);
}


/**************************************************************************//**
 * @brief  Write DAC conversion value
 *****************************************************************************/
void DAC_WriteData(DAC_TypeDef *dac, unsigned int value, unsigned int ch)
{
  /* Write data output value to the correct register. */
  if (!ch)
  {
    dac->CH0DATA = value;
  }
  else
  {
    dac->CH1DATA = value;
  }
}

/**************************************************************************//**
 * @brief Configure ADC for single conversions
 *****************************************************************************/
static void ADCConfig(void)
{
  ADC_Init_TypeDef       init       = ADC_INIT_DEFAULT;
  ADC_InitSingle_TypeDef singleInit = ADC_INITSINGLE_DEFAULT;

  /* Init common settings for both single conversion and scan mode */
  init.timebase = ADC_TimebaseCalc(0);

  /* Might as well finish conversion as quickly as possibly since polling */
  /* for completion. Set ADC clock to 7 MHz, use default HFPERCLK */
  init.prescale = ADC_PrescaleCalc(7000000, 0);

  /* WARMUPMODE must be set to Normal according to ref manual before
   * entering EM2. In this example, the warmup time is not a big problem
   * due to relatively infrequent polling. Leave at default NORMAL, */
  ADC_Init(ADC0, &init);

  /* Init for single conversion use, measure DAC output with 1.25 reference. */
  singleInit.reference = adcRef1V25;

#ifdef EXTERNAL
  /* ADC channel 3 as ADC input */
  singleInit.input = adcSingleInpCh3;
#else
  /* DAC otput channel 0 as ADC input */
  singleInit.input = adcSingleInpDACOut0;
#endif

  singleInit.resolution = adcRes12Bit;

  /* The datasheet specifies a minimum aquisition time when sampling vdd/3 */
  /* 32 cycles should be safe for all ADC clock frequencies */
  singleInit.acqTime = adcAcqTime32;

  ADC_InitSingle(ADC0, &singleInit);
}

