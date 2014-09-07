/******************************************************************************
 * @file main_calendar.c
 * @brief Tickless calendar
 * @author Silicon Labs
 * @version 2.06
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
#include "em_device.h"
#include "em_chip.h"
#include "em_emu.h"
#include "em_cmu.h"
#include "em_rmu.h"
#include "em_rtc.h"

/* Include system clock */
#include "clock.h"

/* Include user interface */
#include "calendar_ui.h"


/* Function prototypes */
void rtcSetup( void );
void cmuSetup( void );

/* Set RTC frequency */
#define RTC_COUNTS_PER_SEC          32768 

/* Set display update interval */
#define DISPLAY_INTERVAL            1          /* in seconds */
#define RTC_COUNTS_BETWEEN_DISPLAY  (DISPLAY_INTERVAL*RTC_COUNTS_PER_SEC)

/* Calendar struct for initial date setting */
struct tm initialCalendar;

/* Flags that signal when to update display */
volatile bool doDisplayUpdate = true;



/******************************************************************************
 * @brief  Main function
 * 
 *****************************************************************************/
int main( void ) 
{
  /* Initialize chip - handle erratas */
  CHIP_Init();


  /* Set up clocks */
  cmuSetup();

  /* Initialize the tickless calendar */
  Clock_Init_TypeDef initialCalendar  = CLOCK_INIT_DEFAULT;
  initialCalendar.rtcCountsPerSec     = RTC_COUNTS_PER_SEC;
  clockInit(&initialCalendar);  

  /* Initialize display ++ */
  uiInit();

  /* Undo CMU changes done by SegmentLCD_Init */
  CMU_ClockSelectSet(cmuClock_LFA,cmuSelect_LFXO);
  CMU_OscillatorEnable(cmuOsc_LFRCO, false, false);
   
  /* Setup RTC */
  rtcSetup();

  /* Enable RTC interrupts */
  NVIC_ClearPendingIRQ(RTC_IRQn);
  NVIC_EnableIRQ(RTC_IRQn);

  
  /* ---------- Eternal while loop ---------- */
  while (1)
  {
    /* Update display */
    if ( doDisplayUpdate )
    {
      /* Clear displayUpdate flag */
      doDisplayUpdate = false;

      /* Get current time and write to display */
      time_t currentTime;
      time( &currentTime );
      uiDisplay( &currentTime );  
    }

    /* Sleep while waiting for interrupt */
    EMU_EnterEM2(true);
  }
}



/******************************************************************************
 * @brief   Configure RTC
 *
 *****************************************************************************/
void rtcSetup(void)
{

  RTC_Init_TypeDef rtcInit;

  /* Configure RTC */
  rtcInit.debugRun = false;
  rtcInit.comp0Top = false;
  rtcInit.enable = false;

  /* Initialize RTC */
  RTC_Init(&rtcInit);

  /* Enable COMP0 interrupt to update the display */
  /* Enable overflow interrupt to keep track of overflows */
  RTC_IntEnable(RTC_IEN_COMP0 | RTC_IEN_OF);
  
  /* Enable RTC */
  RTC_Enable(true);
}

/******************************************************************************
 * @brief   Configure clocks
 *
 *****************************************************************************/
void cmuSetup(void)
{
  /* Start LFXO and wait until it is stable */
  CMU_OscillatorEnable(cmuOsc_LFXO, true, true);

  /* Route the LFXO clock to the RTC and set the prescaler */
  CMU_ClockSelectSet(cmuClock_LFA,cmuSelect_LFXO);
  CMU_ClockEnable(cmuClock_RTC, true);
  
  /* Prescaler of 1 = 30 us of resolution and overflow each 8 min */
  CMU_ClockDivSet(cmuClock_RTC,cmuClkDiv_1);
   
  /* Enable clock to low energy modules */
  CMU_ClockEnable(cmuClock_CORELE, true);

  /* Enable clock to GPIO */
  CMU_ClockEnable(cmuClock_GPIO, true);
}



/***************************************************************************//**
 * @brief RTC Interrupt Handler
 *
 ******************************************************************************/
void RTC_IRQHandler(void)
{
  /* Interrupt source: compare match 0 */
  /*   Increment compare value and update TFT display */
  if ( RTC_IntGet() & RTC_IF_COMP0 )
  {
    RTC_CompareSet(0,RTC->COMP0 + RTC_COUNTS_BETWEEN_DISPLAY);
    RTC_IntClear(RTC_IFC_COMP0);

    /* Set flag for display update */
    doDisplayUpdate = true;
  }


  /* Interrupt source: counter overflow */
  /*   Increase overflow counter and update display */
  if ( RTC_IntGet() & RTC_IF_OF )
  {
    clockOverflow( );
    RTC_IntClear(RTC_IFC_OF);

    /* Set flag for display update */
    doDisplayUpdate = true;
  }

}



