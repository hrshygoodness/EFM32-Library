/**************************************************************************//**
 * @file resetcause_low_voltage.c
 * @brief RMU and VCMP Demo Application
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
#include "em_emu.h"
#include "em_lcd.h"
#include "em_rmu.h"
#include "em_system.h"
#include "em_vcmp.h"
#include "segmentlcd.h"

#define VOLTAGE_LEVEL    2.2

uint32_t resetCause;

/***************************************************************************//**
 * @brief
 *   Displays reset cause on segment LCD
 ******************************************************************************/
void DisplayResetCause()
{
  switch (resetCause)
  {
  case RMU_RSTCAUSE_PORST:
    SegmentLCD_Write("PORST");
    break;

  case RMU_RSTCAUSE_BODUNREGRST:
    SegmentLCD_Write("BOD");
    break;

  case RMU_RSTCAUSE_BODREGRST:
    SegmentLCD_Write("BODREG");
    break;
    
  case RMU_RSTCAUSE_EXTRST | RMU_RSTCAUSE_WDOGRST:
    SegmentLCD_Write("EXTWDOG");

  case RMU_RSTCAUSE_EXTRST:
    SegmentLCD_Write("EXTRST");
    break;

  case RMU_RSTCAUSE_WDOGRST:
    SegmentLCD_Write("WDOGRST");
    break;
    
  case RMU_RSTCAUSE_LOCKUPRST | RMU_RSTCAUSE_SYSREQRST:
    SegmentLCD_Write("LOCKSYS");
    
  case RMU_RSTCAUSE_LOCKUPRST:
    SegmentLCD_Write("LOCKRST");
    break;

  case RMU_RSTCAUSE_SYSREQRST:
    SegmentLCD_Write("SYSRST");
    break;
  }
}

/***************************************************************************//**
 * @brief
 *   Main function
 *
 * @details
 *   Reads the RESETCAUSE register and displays cause of reset on segment LCD
 *   Initializes VCMP for falling edge interrupt and enters EM2
 ******************************************************************************/
int main()
{
  /* Declare VCMP Init struct */
  VCMP_Init_TypeDef vcmp =
  {
    true,                               /* Half bias current */
    0,                                  /* Bias current configuration */
    true,                               /* Enable interrupt for falling edge */
    false,                              /* Enable interrupt for rising edge */
    vcmpWarmTime256Cycles,              /* Warm-up time in clock cycles */
    vcmpHyst20mV,                       /* Hysteresis configuration */
    1,                                  /* Inactive comparator output value */
    false,                              /* Enable low power mode */
    VCMP_VoltageToLevel(VOLTAGE_LEVEL), /* Trigger level */
    false                               /* Enable VCMP after configuration */
  };

  /* Initialize chip */
  CHIP_Init();

  /* Read and clear reset cause register */
  resetCause = RMU_ResetCauseGet();
  RMU_ResetCauseClear();

  /* Initialize LCD and display reset cause */
  SegmentLCD_Init(false);
  DisplayResetCause();

  /* Initialize VCMP */
  CMU_ClockEnable(cmuClock_VCMP, true);
  VCMP_Init(&vcmp);
  
  /* Enable VCMP interrupt lines */
  NVIC_EnableIRQ(VCMP_IRQn);
  VCMP_IntEnable(VCMP_IEN_EDGE | VCMP_IEN_WARMUP);
  
  /* Enable VCMP and wait for warm-up complete */
  VCMP_Enable();
  __WFI();

  while (1)
  {
    EMU_EnterEM2(false);
  }
}

/***************************************************************************//**
 * @brief
 *   VCMP interrupt handler, triggers on EDGE and WARMUP events
 ******************************************************************************/
void VCMP_IRQHandler()
{
  /* Execute on WARMUP interrupt */
  if (VCMP->IF & VCMP_IF_WARMUP)
  {
    /* Enable Low Power Reference */
    VCMP_LowPowerRefSet(true);

    /* Clear interrupt flag */
    VCMP_IntClear(VCMP_IFC_WARMUP);
  }

  /* Execute on EDGE interrupt */
  if (VCMP->IF & VCMP_IF_EDGE)
  {
    /* Low voltage warning */
    SegmentLCD_Write("Vdd LOW");

    /* Clear interrupt flag */
    VCMP_IntClear(VCMP_IFC_EDGE);
  }
}
