/******************************************************************************
 * @file main_acmp_int.c
 * @brief Analog Comparator Window Interrupt Example
 * @author Silicon Labs
 * @version 1.09
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


#include <stdio.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_emu.h"
#include "em_cmu.h"
#include "em_acmp.h"
#include "em_lcd.h"
#include "segmentlcd.h"


/***************************************************************************//**
* @brief
*   Init analog comparator.
*******************************************************************************/
static void ACMPInit(void)
{
  const ACMP_Init_TypeDef acmp_init =
  {
    false,                              /* Full bias current*/
    false,                              /* Half bias current */
    7,                                  /* Biasprog current configuration */
    true,                               /* Enable interrupt for falling edge */
    true,                               /* Enable interrupt for rising edge */
    acmpWarmTime256,                    /* Warm-up time in clock cycles, >140 cycles for 10us with 14MHz */
    acmpHysteresisLevel0,               /* Hysteresis configuration */
    0,                                  /* Inactive comparator output value */
    false,                              /* Enable low power mode */
    0,                                  /* Vdd reference scaling */
    true,                               /* Enable ACMP */
  };

  /* Init and set ACMP channel */
  ACMP_Init(ACMP0, &acmp_init);
  ACMP_ChannelSet(ACMP0, acmpChannel2V5, acmpChannel4);

  ACMP_IntEnable(ACMP0, ACMP_IEN_EDGE);   /* Enable edge interrupt */


  /* Wait for warmup */
  while (!(ACMP0->STATUS & ACMP_STATUS_ACMPACT)) ;

  /* Enable interrupts */
  NVIC_ClearPendingIRQ(ACMP0_IRQn);
  NVIC_EnableIRQ(ACMP0_IRQn);
}


/**************************************************************************//**
 * @brief ACMP0 Interrupt handler
 *****************************************************************************/
void ACMP0_IRQHandler(void)
{
  /* Clear interrupt flag */
  ACMP0->IFC = ACMP_IFC_EDGE;
  
  /* Write ACMP status to LCD */
  SegmentLCD_Number((int) ((ACMP0->STATUS & ACMP_STATUS_ACMPOUT) >> _ACMP_STATUS_ACMPOUT_SHIFT));
}

/******************************************************************************
 * @brief  Main function, enables analog comparator 0 and waits in em2 for
 * comparator interrupt, both rising and falling edges.
 * Main is called from _program_start, see assembly startup file
 *****************************************************************************/
int main(void)
{
  /* Initialize chip */
  CHIP_Init();
  SegmentLCD_Init(false);

  /* Enable clocks required */
  CMU_ClockEnable(cmuClock_ACMP0, true);

  ACMPInit();
  
  /* Write ACMP status to LCD */
  SegmentLCD_Number((int) ((ACMP0->STATUS & ACMP_STATUS_ACMPOUT) >> _ACMP_STATUS_ACMPOUT_SHIFT));

  /* Stay in this loop forever at end of program */
  while (1)
  {
    /* Enter em2 and wait for acmp interrupt */
    EMU_EnterEM2(true);
  }
}

