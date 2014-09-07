/******************************************************************************
 * @file main_idac_calibration.c
 * @brief IDAC Calibration Routine
 * @author Silicon Labs
 * @version 1.01
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
#include <stdlib.h>
#include "em_device.h"
#include "em_idac.h"
#include "em_gpio.h"
#include "em_emu.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_adc.h"

/* Max value of the IDAC Calibration tuning register */
#define IDAC_TUNING_REG_MAX (_IDAC_CAL_TUNING_MASK >> _IDAC_CAL_TUNING_SHIFT)

/* Max value of STEPSEL in the IDAC CURPROG register */
#define IDAC_STEP_MAX (_IDAC_CURPROG_STEPSEL_MASK >> _IDAC_CURPROG_STEPSEL_SHIFT)

/* Reference voltage for the ADC in mV */
#define ADC_REFERENCE 1250

/* IDAC current range start values in nA */
uint32_t IdacRangeStart[4] = {50, 1600, 500, 2000}; 

/* IDAC current step size in nA */
uint32_t IdacStepSize[4] = {50, 100, 500, 2000}; 



/***************************************************************************//**
 * @brief
 *   IDAC calibration function, calculates the IDAC current output using
 *   a resistor connected between ground and the selected ADC channel and tunes
 *   the IDAC Caibration register value
 * @param[in] range IDAC range to calibrate 
 * @param[in] step IDAC step to calibrate 
 * @param[in] resistor Value of the calibration resistor
 * @param[in] adcCh Selects the ADC input channel to use
 * @param[out] The devication from the target current, after calibration is done
 *******************************************************************************/
int32_t idacCalibrate(IDAC_Range_TypeDef range, uint32_t step, uint32_t resistor, ADC_SingleInput_TypeDef adcCh)
{
  uint32_t adcSample, target_current, current, i, n;
  int32_t deviation, oldDeviation, calRegValue;
  deviation = 0;
  
  if(step > IDAC_STEP_MAX)
  {
    return 9999999;
  }

  /* Set the ADC to the correct input channel */
  ADC0->SINGLECTRL = (ADC0->SINGLECTRL & ~_ADC_SINGLECTRL_INPUTSEL_MASK) | ((uint32_t)adcCh << _ADC_SINGLECTRL_INPUTSEL_SHIFT) ;
  
  /* Calculate target_current in nA*/ 
  switch(range)
  {
  case idacCurrentRange0:
    target_current = IdacRangeStart[0] + step*IdacStepSize[0];
    break;
  case idacCurrentRange1:
    target_current = IdacRangeStart[1] + step*IdacStepSize[1];
    break;
  case idacCurrentRange2:
    target_current = IdacRangeStart[2] + step*IdacStepSize[2];
    break;
  case idacCurrentRange3:
    target_current = IdacRangeStart[3] + step*IdacStepSize[3];
    break; 
  default:
    break;
  }  
  
  /* Set IdacRange to the range and step that is to be calibrated */
  IDAC_RangeSet(IDAC0, range);
  IDAC_StepSet(IDAC0, step);
  
  /* Enable IDAC out */
  IDAC_OutEnable(IDAC0, true); 
  
  /*  Reset the IDAC Calibration register */
  calRegValue = 0;
  IDAC0->CAL = calRegValue;

  /* Take a single sample to let the IDAC current output settle before the binary search */
  ADC_Start(ADC0, adcStartSingle);
  /* Wait while conversion is active */
  while (ADC0->STATUS & ADC_STATUS_SINGLEACT);
  
  /* Calculate number of steps */
  n = (uint32_t)(log(IDAC_TUNING_REG_MAX)/log(2));

  /* Do a binary search to find the best tuning value for the  */
  /* IDAC Calibration register, binary search takes a maximum of log2(N) steps  */
  for(i=0; i<n; i++)
  {
    /* Start single ADC sample  */
    ADC_Start(ADC0, adcStartSingle);
    /* Wait while conversion is active */
    while (ADC0->STATUS & ADC_STATUS_SINGLEACT);
    /* Get ADC result,  */
    adcSample = ADC_DataSingleGet(ADC0);
  
    /* Calculate current trought the calibration resistor
    * U = adcSample = (adcSample * ADC_REFERENCE)/2^12
    * R = CALIBRATION_RESISTOR
    * I = U/R */
    current = ((adcSample * ADC_REFERENCE)/ 4096); //mV
    current *= 1000000;  /* nV */         
    current /= resistor; /* nA */            
   
    /* Store the last deviation value */
    oldDeviation = deviation;
    /* Difference between the target and measured current */
    deviation = target_current-current;
    
    /* If current output is below the target, increment the tuning value */
    if(deviation > 0)
    {
      calRegValue += IDAC_TUNING_REG_MAX/(2<<i);
      IDAC0->CAL = calRegValue;
    }
    else if(deviation <= 0)
    {
      calRegValue -= IDAC_TUNING_REG_MAX/(2<<i);
      IDAC0->CAL = calRegValue;     
    }
  }
  
  /* If the binary search overstepped with 1 */
  if(abs(oldDeviation) < abs(deviation))
  {
    deviation = oldDeviation;
    if(oldDeviation > 0)
    {
      IDAC0->CAL -= 1;
    }
    else{
      IDAC0->CAL += 1;
    }
  }

  /* Disable IDAC out */
  IDAC_OutEnable(IDAC0, false);

  return deviation;
}

/***************************************************************************//**
 * @brief
 *   Configure IDAC usage for this application.
 *******************************************************************************/
void setupIdac()
{
  /* Enable IDAC clock */
  CMU_ClockEnable(cmuClock_IDAC0, true);
  
  /* Set outmode to IDAC_OUT */
  IDAC_Init_TypeDef idacInit = 
  {
    .enable = true,
    .outMode = idacOutputADC,
    .prsEnable = false,
    .prsSel = idacPRSSELCh0,
    .sinkEnable = false,
  };
  
  IDAC_Init(IDAC0, &idacInit); 
}
 

/***************************************************************************//**
 * @brief
 *   Configure ADC usage for this application.
 *******************************************************************************/
void setupAdc(void)
{
  ADC_Init_TypeDef       init       = ADC_INIT_DEFAULT;
  ADC_InitSingle_TypeDef singleInit = ADC_INITSINGLE_DEFAULT;
  
  /* Init common settings for both single conversion and scan mode */
  init.timebase = ADC_TimebaseCalc(0);
  /* Might as well finish conversion as quickly as possibly since polling */
  /* for completion. */
  /* Set ADC clock to 7 MHz, use default HFPERCLK */
  init.prescale = ADC_PrescaleCalc(7000000, 0);

  ADC_Init(ADC0, &init);

  /* Init for single conversion use 1.25 reference. */
  singleInit.reference  = adcRef1V25;
  /* On PD4, current from IDAC will be on this pin when sampled */ 
  singleInit.input      = adcSingleInpCh4; 
  singleInit.resolution = adcRes12Bit;
  
  /* Keeps the current channel selected by INPUTSEL connected when ADC is IDLE  */
  /* This setting will let IDAC put current out on the adc pin between ADC samples, */
  /* avoiding the small spikes in the current when turning it on/off fast */
  ADC0->CTRL |= ADC_CTRL_CHCONIDLE_KEEPCON;

  ADC_InitSingle(ADC0, &singleInit);
}

/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{


  /* Enable clocks required */
  CMU_ClockEnable(cmuClock_HFPER, true);
  CMU_ClockEnable(cmuClock_ADC0, true);
  
  /* Chip errata */
  CHIP_Init();

  /* Initialize ADC */
  setupAdc();
  
  /* Initialize IDAC */
  setupIdac();
  
  /* Calibrate IDAC range 3 and step 0x15 using adc channel 4 and resistor value 18K ohm */
  idacCalibrate(idacCurrentRange3, 0x15, 18000, adcSingleInpCh4);

  while (1)
  {
    /* Enter EM3 after calibrating IDAC */
    EMU_EnterEM3(false);
  }
}
