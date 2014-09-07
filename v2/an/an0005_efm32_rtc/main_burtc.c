/*****************************************************************************
 * @file main_burtc.c

 * @brief Backup Real Time Counter Demo Application
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
 
#include "em_device.h"
#include "em_chip.h"
#include "em_rtc.h"
#include "em_burtc.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_rmu.h"

/* Defines*/
#define LFRCO_FREQUENCY                  32768
#define WAKEUP_INTERVAL_MS              500
#define BURTC_COUNT_BETWEEN_WAKEUP      (((LFRCO_FREQUENCY * WAKEUP_INTERVAL_MS) / 1000)-1)


/**************************************************************************//**
 * @brief BURTC Interrupt Handler clears the flag 
 *****************************************************************************/
void BURTC_IRQHandler(void)
{
  /* Clear interrupt source */
  BURTC_IntClear(BURTC_IFC_COMP0); 
}

/**************************************************************************//**
 * @brief  Setup BURTC
 * Using LFRCO clock source and enabling interrupt on COMP0 match
 *****************************************************************************/
void setupBurtc(void)
{
  BURTC_Init_TypeDef burtcInit = BURTC_INIT_DEFAULT; 
  
  burtcInit.enable       = true;                /* Enable BURTC after initialization  */
  burtcInit.mode         = burtcModeEM3;        /* BURTC is enabled in EM0-EM3 */
  burtcInit.debugRun     = false;               /* Counter shall keep running during debug halt. */              
  burtcInit.clkSel       = burtcClkSelLFRCO;    /* Select LFRCO as clock source */
  burtcInit.clkDiv       = burtcClkDiv_1;       /* Clock prescaler             */
  burtcInit.lowPowerComp = 0;                   /* Number of least significantt clock bits to ignore in low power mode */
  burtcInit.timeStamp    = true;                /* Enable time stamp on entering backup power domain */
  burtcInit.compare0Top  = true;                /* Clear counter on compare match */
  burtcInit.lowPowerMode = burtcLPDisable;      /* Low power operation mode, requires LFXO or LFRCO */

  BURTC_CompareSet(0, BURTC_COUNT_BETWEEN_WAKEUP);  /* Set top value for comparator */
  
  /* Enabling Interrupt from BURTC */
  NVIC_EnableIRQ(BURTC_IRQn);
  BURTC_IntEnable( BURTC_IF_COMP0 );    /* Enable compare interrupt flag */
    
  /* Initialize BURTC */
  BURTC_Init(&burtcInit);
}


/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
  /* Initialize chip */
  CHIP_Init();
  
  /* Enable Low energy clocking module clock. */
  CMU_ClockEnable(cmuClock_CORELE, true);

  /* Disable LFA and LFB clock domains to save power */
  CMU->LFCLKSEL = 0;
  
  /* Starting LFRCO and waiting until it is stable */
  CMU_OscillatorEnable(cmuOsc_LFRCO, true, true);

  /* Enable access to BURTC registers */
  RMU_ResetControl(rmuResetBU, false);

  /* Setting up burtc */
  setupBurtc();
  
  while (1)
  {
    /* Enter EM2. */
    EMU_EnterEM2(false);
  }
}

