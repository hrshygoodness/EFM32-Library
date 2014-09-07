/**************************************************************************//**
 * @file vcmp_demo.c
 * @brief VCMP Demo Application
 * @author Silicon Labs
 * @version 1.08
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

#include "em_chip.h"
#include "em_device.h"
#include "em_cmu.h"
#include "em_system.h"
#include "em_vcmp.h"
#include "segmentlcd.h"

#define MIN_VOLTAGE_LEVEL    2.97
#define MAX_VOLTAGE_LEVEL    3.03

typedef struct
{
  uint32_t min;
  uint32_t max;
} VCMP_VoltageRange_TypeDef;

/***************************************************************************//**
 * @brief
 *   Wait for comparator propagation delay
 ******************************************************************************/
void WaitForComparatorUpdate()
{
  /* Reenable VCMP to trigger a new warm-up cycle */
  VCMP_Disable();
  VCMP_Enable();
  
  /* Wait for VCMP warm-up */
  while (!VCMP_Ready()) ;
}

/***************************************************************************//**
 * @brief
 *   Main function
 *
 * @details
 *   Continously checks if supply voltage is within predefined range
 ******************************************************************************/
int main()
{
  /* Declare VCMP Init struct */
  VCMP_Init_TypeDef vcmp =
  {
    false,                              /* Half bias current */
    7,                                  /* Bias current configuration */
    false,                              /* Enable interrupt for falling edge */
    false,                              /* Enable interrupt for rising edge */
    vcmpWarmTime256Cycles,              /* Warm-up time in clock cycles */
    vcmpHystNone,                       /* Hysteresis configuration */
    0,                                  /* Inactive comparator output value */
    false,                              /* Enable low power mode */
    VCMP_VoltageToLevel(3.3),           /* Trigger level */
    true                                /* Enable VCMP after configuration */
  };

  /* Declare VCMP voltage range struct */
  VCMP_VoltageRange_TypeDef voltageRange =
  {
    VCMP_VoltageToLevel(MIN_VOLTAGE_LEVEL),
    VCMP_VoltageToLevel(MAX_VOLTAGE_LEVEL)
  };

  /* Declare variables */
  bool voltageAboveMin;
  bool voltageBelowMax;

  /* Initialize chip */
  CHIP_Init();

  /* Initialize LCD */
  SegmentLCD_Init(false);

  /* Initialize VCMP */
  CMU_ClockEnable(cmuClock_VCMP, true);
  VCMP_Init(&vcmp);

  while (1)
  {   
    /* Set and check lower limit */
    VCMP_TriggerSet(voltageRange.min);
    WaitForComparatorUpdate();
    voltageAboveMin = VCMP_VDDHigher();

    /* Set and check upper limit */
    VCMP_TriggerSet(voltageRange.max);
    WaitForComparatorUpdate();
    voltageBelowMax = VCMP_VDDLower();

    /* Display result */
    if (voltageAboveMin && voltageBelowMax)
    {
      SegmentLCD_Write("V OK");
    }
    else
    {
      if (!voltageAboveMin)
        SegmentLCD_Write("V LOW");

      if (!voltageBelowMax)
        SegmentLCD_Write("V HIGH");
    }
  }
}
