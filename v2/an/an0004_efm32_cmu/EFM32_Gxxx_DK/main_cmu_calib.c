/**************************************************************************//**
 * @file
 * @brief CMU calibration example for EFM32_G2xx_DK, EFM32_G8xx_DK, EFM32_G890_STK,
 * EFM32_TG840_STK and EFM32_GG990_STK.
 * @author Energy Micro AS
 * @version 1.06
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2010 Energy Micro AS, http://www.energymicro.com</b>
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
 * 4. The source and compiled code may only be used on Energy Micro "EFM32"
 *    microcontrollers and "EFR4" radios.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Energy Micro AS has no
 * obligation to support this Software. Energy Micro AS is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Energy Micro AS will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 *****************************************************************************/

#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_lcd.h"
#include "em_system.h"
#include "segmentlcd.h"

/* Max number of HFCycles for higher accuracy */
#define HFCYCLES            0xFFFFF
/* HF_cycles x ref_freq / HFCLK_freq = (2^20 - 1) x 32768 / 14000000 = 2454 */
#define UP_COUNTER_TUNED    2454
/* Initial HFRCO TUNING value */
#define TUNINGMAX           0xFF

/**************************************************************************//**
 * @brief Calibrate HFRCO for 14Mhz against LFXO
 * @param hfCycles down counter initial value
 * @param upCounterTuned value expected for the upCounter when HFRCO is tuned
 *****************************************************************************/
uint8_t calibrateHFRCO(uint32_t hfCycles, uint32_t upCounterTuned)
{
  /* run the calibration routine to know the initial value
   * of the up counter */
  uint32_t upCounter = CMU_Calibrate(hfCycles, cmuOsc_LFXO);
  /* Variable to store the previous upcounter value for comparison */
  uint32_t upCounterPrevious;
  /* Read the initial tuning value */
  uint8_t  tuningVal = CMU_OscillatorTuningGet(cmuOsc_HFRCO);

  /* If the up counter result is smaller than the
   * tuned value, the HFRCO is running
   * at a higher frequency so the HFRCO tuning
   * register has to be decremented */
  if (upCounter < upCounterTuned)
  {
    while (upCounter < upCounterTuned)
    {
      /* store the previous value for comparison */
      upCounterPrevious = upCounter;
      /* Decrease tuning value */
      tuningVal--;
      /* Write the new HFRCO tuning value */
      CMU_OscillatorTuningSet(cmuOsc_HFRCO, tuningVal);
      /* Run the calibration routine again */
      upCounter = CMU_Calibrate(hfCycles, cmuOsc_LFXO);
    }
    /* Check which value goes closer to the tuned one
     * and increase the tuningval if the previous value
     * is closer */
    if ((upCounter - upCounterTuned) > (upCounterTuned - upCounterPrevious))
    {
      tuningVal++;
    }
  }

  /* If the up counter result is higher than the
   * desired value, the HFRCO is running
   * at a higher frequency so the tuning
   * value has to be incremented */
  if (upCounter > upCounterTuned)
  {
    while (upCounter > upCounterTuned)
    {
      /* store the previous value for comparison */
      upCounterPrevious = upCounter;
      /* Increase tuning value */
      tuningVal++;
      /* Write the new HFRCO tuning value */
      CMU_OscillatorTuningSet(cmuOsc_HFRCO, tuningVal);
      /* Run the calibration routine again */
      upCounter = CMU_Calibrate(hfCycles, cmuOsc_LFXO);
    }
    /* Check which value goes closer to the tuned one
     * and decrease the tuningval if the previous value
     * is closer */
    if ((upCounterTuned - upCounter) > (upCounterPrevious - upCounterTuned))
    {
      tuningVal--;
    }
  }

  /* Return final HFRCO tuning value */
  return tuningVal;
}

/******************************************************************************
 * @brief  Main function
 * Main is called from _program_start, see assembly startup file
 *****************************************************************************/
int main(void)
{
  /* Initialize chip */
  CHIP_Init();

  /* The HFRCO is the default clock source for the HF
   * branch, but it is explicitly selected here
   * for comprehension purposes */
  CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFRCO);

  /* Enable LFXO to clock the LFA clock branch */
  CMU_OscillatorEnable(cmuOsc_LFXO, true, true);

  /* Select LFXO as clock source for the LFA branch */
  CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);

  /* Enable clock for LCD module */
  CMU_ClockEnable(cmuClock_LCD, true);

  /* Enable clock for GPIO module */
  CMU_ClockEnable(cmuClock_GPIO, true);

  /* Set PC12 as output so it can be
   * overriden by the peripheral, in this case the CMU */
  GPIO_PinModeSet(gpioPortC, 12, gpioModePushPull, 0);

  /* Initialize LCD */
  SegmentLCD_Init(false);

  /* Set the tuning to the max value so the HFRCO goes to a frequency above 14Mhz
   * - the frequency will depend on the chip */
  CMU_OscillatorTuningSet(cmuOsc_HFRCO, TUNINGMAX);

  /* Select Clock Output 0 as High Frequency RC without prescalling (14 Mhz) */
  CMU->CTRL = CMU->CTRL | CMU_CTRL_CLKOUTSEL1_LFRCO | CMU_CTRL_CLKOUTSEL0_HFRCO;

  /* Route the Clock output pin to Location 1 and enable them */
  CMU->ROUTE = CMU_ROUTE_LOCATION_LOC1 | CMU_ROUTE_CLKOUT0PEN;

  /* Write Tuning on the LCD while the function runs */
  SegmentLCD_Write("TUNING");

  /* Variable for reading the tuning value after the calibration */
  uint8_t tuning;

  /* Run HFRCO calibration for the 14Mhz band
   * This function takes about 10 seconds to execute because
   * the HFRCO was purposely untuned to the 14Mhz band upper limit
   * (TUNING = 0xFF)
   * The calibration loop inside this function will have to execute
   * as many times as needed for the HFRCO become calibrated
   * which usually happens for a TUNING value of 116-122, depending
   * on the chip */
  tuning = calibrateHFRCO(HFCYCLES, UP_COUNTER_TUNED);

  /* Write the tuning result on the LCD and TUNING FINISHED */
  SegmentLCD_Write("DONE");
  SegmentLCD_Number(tuning);

  while (1) ;
}
