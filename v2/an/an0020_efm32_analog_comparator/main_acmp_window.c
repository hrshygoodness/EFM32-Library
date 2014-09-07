/******************************************************************************
 * @file main_acmp_window.c
 * @brief Analog Comparator Window Mode Example
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

#define VDD 3.3
#define UPPER_LIMIT 2.0
#define LOWER_LIMIT 1.0

uint32_t AComp0     = 0;
uint32_t AComp1     = 0;

/***************************************************************************//**
* @brief
*   Init analog comparators for window mode.
*******************************************************************************/
static void ACMPInit(void)
{
  const ACMP_Init_TypeDef acmp0_init =
  {
    false,                              /* Full bias current*/
    false,                              /* Half bias current */
    7,                                  /* Biasprog current configuration */
    true,                               /* Enable interrupt for falling edge */
    true,                               /* Enable interrupt for rising edge */
    acmpWarmTime256,                    /* Warm-up time in clock cycles, >140 cycles for 10us with 14MHz */
    acmpHysteresisLevel1,               /* Hysteresis configuration */
    0,                                  /* Inactive comparator output value */
    false,                              /* Enable low power mode */
    (int) (LOWER_LIMIT*63/VDD),         /* Vdd reference scaling */
    true,                               /* Enable ACMP */
  };
  
  const ACMP_Init_TypeDef acmp1_init =
  {
    false,                              /* Full bias current*/
    false,                              /* Half bias current */
    7,                                  /* Biasprog current configuration */
    true,                               /* Enable interrupt for falling edge */
    true,                               /* Enable interrupt for rising edge */
    acmpWarmTime256,                    /* Warm-up time in clock cycles, >140 cycles for 10us with 14MHz */
    acmpHysteresisLevel1,               /* Hysteresis configuration */
    0,                                  /* Inactive comparator output value */
    false,                              /* Enable low power mode */
    (int) (UPPER_LIMIT*63/VDD),         /* Vdd reference scaling */
    true,                               /* Enable ACMP */
  };

  /* Init and set ACMP channels */
  ACMP_Init(ACMP0, &acmp0_init);
  ACMP_Init(ACMP1, &acmp1_init);

  /* Configure 1.0V as lower boundary and channel0 on PC4 as input */
  ACMP_ChannelSet(ACMP0, acmpChannelVDD, acmpChannel4);

  /* Configure 2.0V as upper boundary and channel0 on PC12 as input */
  ACMP_ChannelSet(ACMP1, acmpChannelVDD, acmpChannel4);

  ACMP_IntEnable(ACMP0, ACMP_IEN_EDGE);   /* Enable edge interrupt */
  ACMP_IntEnable(ACMP1, ACMP_IEN_EDGE);

  /* Wait for warm up */
  while (!(ACMP0->STATUS & ACMP_STATUS_ACMPACT)) ;
  while (!(ACMP1->STATUS & ACMP_STATUS_ACMPACT)) ;

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
  ACMP1->IFC = ACMP_IFC_EDGE;
  
  /* Combines the two outputs to produce a number between 0 and 3 which indicates inputstate */
  AComp0 = ((ACMP0->STATUS & ACMP_STATUS_ACMPOUT) >> _ACMP_STATUS_ACMPOUT_SHIFT);
  AComp1 = ((ACMP1->STATUS & ACMP_STATUS_ACMPOUT) >> _ACMP_STATUS_ACMPOUT_SHIFT);

  /* Write state to lcd */
  if (!AComp0 & !AComp1) 
    SegmentLCD_Write("under");
  else if (AComp0 & !AComp1) 
    SegmentLCD_Write("inside");
  else if (AComp0 & AComp1) 
    SegmentLCD_Write("over");
  else 
    SegmentLCD_Write("error");
}

/******************************************************************************
 * @brief  Main function, stays in em2 waiting for comparator interrupts
 * from one or both of the comparators, prints out if the input signal is
 * under, inside or over the window.
 * Main is called from _program_start, see assembly startup file
 *****************************************************************************/
int main(void)
{
  /* Initialize chip */
  CHIP_Init();
  SegmentLCD_Init(false);

  /* Enable clocks required */
  CMU_ClockEnable(cmuClock_ACMP0, true);
  CMU_ClockEnable(cmuClock_ACMP1, true);

  ACMPInit();
  
  /* Combines the two outputs to produce a number between 0 and 3 which indicates inputstate */
  AComp0 = ((ACMP0->STATUS & ACMP_STATUS_ACMPOUT) >> _ACMP_STATUS_ACMPOUT_SHIFT);
  AComp1 = ((ACMP1->STATUS & ACMP_STATUS_ACMPOUT) >> _ACMP_STATUS_ACMPOUT_SHIFT);

  /* Write state to lcd */
  if (!AComp0 & !AComp1) 
    SegmentLCD_Write("under");
  else if (AComp0 & !AComp1) 
    SegmentLCD_Write("inside");
  else if (AComp0 & AComp1) 
    SegmentLCD_Write("over");
  else 
    SegmentLCD_Write("error");

  /* Stay in this loop forever at end of program */
  while (1)
  {
    EMU_EnterEM2(true);
  }
}

