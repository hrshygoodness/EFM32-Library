/***************************************************************************//**
 * @file main_acmp_conf.c
 * @brief Analog Comparator Polled Example
 * @author Silicon Labs
 * @version 1.09
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


#include <stdio.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_emu.h"
#include "em_cmu.h"
#include "em_acmp.h"
#include "em_lcd.h"
#include "segmentlcd.h"
#include "rtcdrv.h"

/******************************************************************************
 * @brief  Main function, enables the analog comparator and prints the
 * comparator output to the lcd every 100 ms. Stays in EM2 between wakeups.
 * Main is called from _program_start, see assembly startup file
 *****************************************************************************/
int main(void)
{
  /* Initialize chip */
  CHIP_Init();
  SegmentLCD_Init(false);

  /* Enable clocks required */
  CMU_ClockEnable(cmuClock_ACMP0, true);

  const ACMP_Init_TypeDef acmp_init =
  {
    false,                              /* Full bias current*/
    false,                              /* Half bias current */
    7,                                  /* Biasprog current configuration */
    false,                              /* Enable interrupt for falling edge */
    false,                              /* Enable interrupt for rising edge */
    acmpWarmTime256,                    /* Warm-up time in clock cycles, >140 cycles for 10us with 14MHz */
    acmpHysteresisLevel0,               /* Hysteresis configuration */
    0,                                  /* Inactive comparator output value */
    false,                              /* Enable low power mode */
    0,                                  /* Vdd reference scaling */
    true,                               /* Enable ACMP */
  };

  /* Init and set ACMP channel */
  ACMP_Init(ACMP0, &acmp_init);
  ACMP_ChannelSet(ACMP0, acmpChannel1V25, acmpChannel4);

  /* Set up GPIO as output on location 1, which is PE2, without inverting */
  ACMP_GPIOSetup(ACMP0, 1, 1, 0);

  /* Stay in this loop forever at end of program */
  while (1)
  {
    /* Write ACMP status to LCD */
    SegmentLCD_Number((int) ((ACMP0->STATUS & ACMP_STATUS_ACMPOUT) >> _ACMP_STATUS_ACMPOUT_SHIFT));

    /* Wait 100ms in EM2 */
    RTCDRV_Trigger(100, NULL);
    EMU_EnterEM2(true);
  }
}

