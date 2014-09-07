/******************************************************************************
 * @file main_gg_dk.c
 * @brief Backup power domain and backup real time counter application note
 * @author Silicon Labs
 * @version 1.26
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

/* Include emlib */ 
#include "em_burtc.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_device.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_rmu.h"

#include "clock.h"
#include "clock_config.h"
#include "clockApp_dk.h"

#include "bsp.h"


/* Declare variables */
static uint32_t resetcause = 0;

/* Declare BURTC variables */
static uint32_t burtcCountAtWakeup = 0;

/* Calendar struct for initial date setting */
static struct tm initialCalendar;


/* Function prototypes */
void budSetup( void );
void burtcSetup( void );


/******************************************************************************
 * @brief  Main function
 * 
 *****************************************************************************/
int main( void ) 
{
  /* Initialize chip - handle erratas */
  CHIP_Init();

  /* Read and clear RMU->RSTCAUSE as early as possible */
  resetcause = RMU->RSTCAUSE;
  RMU_ResetCauseClear();

  /* Enable clock to low energy modules */
  CMU_ClockEnable(cmuClock_CORELE, true);

  /* Read Backup Real Time Counter value */
  burtcCountAtWakeup = BURTC_CounterGet();

  /* Configure Backup Domain */
  budSetup();

  /* Setting up a structure to initialize the calendar
     for 1 January 2012 12:00:00
     The struct tm is declared in time.h 
     More information for time.h library in http://en.wikipedia.org/wiki/Time.h */  
  initialCalendar.tm_sec    =  0;    /* 0 seconds (0-60, 60 = leap second)*/
  initialCalendar.tm_min    =  0;    /* 0 minutes (0-59) */
  initialCalendar.tm_hour   =  12;   /* 12 hours (0-23) */
  initialCalendar.tm_mday   =  1;    /* 1st day of the month (1 - 31) */
  initialCalendar.tm_mon    =  0;    /* January (0 - 11, 0 = January) */
  initialCalendar.tm_year   =  112;  /* Year 2012 (year since 1900) */
  initialCalendar.tm_wday   =  0;    /* Sunday (0 - 6, 0 = Sunday) */
  initialCalendar.tm_yday   =  0;    /* 1st day of the year (0-365) */
  initialCalendar.tm_isdst  =  -1;   /* Daylight saving time; enabled (>0), disabled (=0) or unknown (<0) */ 

  /* Set the calendar */
  clockInit(&initialCalendar);  

  /* If waking from backup mode, restore time from retention registers */
  uint32_t resetmaskp = RMU_RSTCAUSE_BUMODERST | RMU_RSTCAUSE_BUBODVDDDREG;
  uint32_t resetmaskn = RMU_RSTCAUSE_BUBODREG | RMU_RSTCAUSE_BUBODUNREG | RMU_RSTCAUSE_BUBODBUVIN | RMU_RSTCAUSE_EXTRST | RMU_RSTCAUSE_PORST;
  if ( (resetcause & resetmaskp) && !(resetcause & resetmaskn) )
  {
  }

  /* If waking from backup mode, restore time from retention registers */
  if ( (resetcause & RMU_RSTCAUSE_BUMODERST | RMU_RSTCAUSE_BUBODVDDDREG) && !(resetcause & RMU_RSTCAUSE_BUBODREG | RMU_RSTCAUSE_BUBODUNREG | RMU_RSTCAUSE_BUBODBUVIN | RMU_RSTCAUSE_EXTRST | RMU_RSTCAUSE_PORST) )
 {
 }
 
  /* If waking from backup mode, restore time from retention registers */
  if (    (resetcause & RMU_RSTCAUSE_BUMODERST) 
      && !(resetcause & RMU_RSTCAUSE_BUBODREG)
      && !(resetcause & RMU_RSTCAUSE_BUBODUNREG)
      && !(resetcause & RMU_RSTCAUSE_BUBODBUVIN) 
      &&  (resetcause & RMU_RSTCAUSE_BUBODVDDDREG)
      && !(resetcause & RMU_RSTCAUSE_EXTRST)
      && !(resetcause & RMU_RSTCAUSE_PORST) )
  {
    /* Initialize display application */
    clockAppInit();

    
    /* Check if retention registers were being written to when backup mode was entered */
    if ( (BURTC_Status() & BURTC_STATUS_RAMWERR) >> _BURTC_STATUS_RAMWERR_SHIFT )
    {
      clockAppPrintRamWErr();
    }
   
    /* Check if timestamp is written */
    if (! ((BURTC_Status() & BURTC_STATUS_BUMODETS) >> _BURTC_STATUS_BUMODETS_SHIFT) )
    { 
      clockAppPrintNoTimestamp();
    }

    /* Restore time from backup RTC + retention memory and print backup info*/
    clockAppRestore( burtcCountAtWakeup );
    
    /* Reset timestamp */
    BURTC_StatusClear();
  }

  /* If normal startup, initialize BURTC */
  else
  {
		/* Start LFXO and wait until it is stable */
		CMU_OscillatorEnable(cmuOsc_LFXO, true, true);

    /* Setup BURTC */
    burtcSetup();

    /* Initialize display application */
    clockAppInit();

    /* Display reset cause */
    clockAppPrintResetCause(resetcause);

    /* Start BURTC */
    BURTC_Enable( true );

    /* Backup initial calendar (initialize retention registers) */
    clockAppBackup();
  }

  /* Enable BURTC interrupts */
  NVIC_ClearPendingIRQ( BURTC_IRQn );
  NVIC_EnableIRQ( BURTC_IRQn );

  /* Display reset cause */
  clockAppPrintResetCause(resetcause);


  /* ---------- Eternal while loop ---------- */
  while (1)
  {
    /* Update display if necessary */
    clockAppDisplay();    

    /* Sleep while waiting for interrupt */
    EMU_EnterEM2(false);
  }
}



/***************************************************************************//**
 * @brief Set up backup domain.
 ******************************************************************************/
void budSetup(void)
{
  /* Assign default TypeDefs */
  EMU_EM4Init_TypeDef em4Init = EMU_EM4INIT_DEFAULT;
  EMU_BUPDInit_TypeDef bupdInit = EMU_BUPDINIT_DEFAULT; 

  /*Setup EM4 configuration structure */
  em4Init.lockConfig = true;
  em4Init.osc = emuEM4Osc_LFXO;
  em4Init.buRtcWakeup = false;
  em4Init.vreg = true;

  /* Setup Backup Power Domain configuration structure */                                          
  bupdInit.probe = emuProbe_Disable;
  bupdInit.bodCal = false;
  bupdInit.statusPinEnable = true;
  bupdInit.resistor = emuRes_Res0;
  bupdInit.voutStrong = false;
  bupdInit.voutMed = false;
  bupdInit.voutWeak = false;
  bupdInit.inactivePower = emuPower_None;
  bupdInit.activePower = emuPower_MainBU;
  bupdInit.enable = true;

  /* Unlock configuration */
  EMU_EM4Lock( false );
   
  /* Initialize EM4 and Backup Power Domain with init structs */
  EMU_BUPDInit( &bupdInit );
  EMU_EM4Init( &em4Init );
  
  /* Release reset for backup domain */
  RMU_ResetControl( rmuResetBU, false );

  /* Lock configuration */
  EMU_EM4Lock( true );
}


/******************************************************************************
 * @brief   Configure backup RTC
 *****************************************************************************/
void burtcSetup(void)
{
  /* Create burtcInit struct and fill with default values */ 
  BURTC_Init_TypeDef burtcInit = BURTC_INIT_DEFAULT;

  /* Set burtcInit to proper values for this application */
  /* To make this example easier to read, all fields are listed, 
     even those which are equal to their default value */
  burtcInit.enable = false;
  burtcInit.mode = burtcModeEM4;
  burtcInit.debugRun = false;
  burtcInit.clkSel = burtcClkSelLFXO;
  burtcInit.clkDiv = burtcClkDiv_128;
  burtcInit.timeStamp = true;
  burtcInit.compare0Top = false;
  burtcInit.lowPowerMode = burtcLPDisable;
  
  /* Initialize BURTC with burtcInit struct */
  BURTC_Init( &burtcInit );
  
  /* Enable BURTC interrupt on compare match and counter overflow */
  BURTC_IntEnable( BURTC_IF_COMP0 | BURTC_IF_OF );
}
