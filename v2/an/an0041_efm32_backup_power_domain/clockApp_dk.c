/***************************************************************************//**
 * @file clockApp_dk.c
 * @brief Application handling calendar display and user input in 
 *        EFM32 Backup Power Domain Application Note
 * @author Silicon Labs
 * @version 1.26
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

/* Include standard libraries */ 
#include <stdint.h>
#include <stdio.h>

/* Include emlib */
#include "em_burtc.h"
#include "em_device.h"
#include "em_gpio.h"

/* Include BSP drivers */
#include "bsp.h"
#include "glib.h"
#include "tftamapped.h"

/* Include others */
#include "clock.h"
#include "clock_config.h"
#include "clockApp_dk.h"


/* Calendar struct */
static struct tm calendar;

/* Declare variables for time keeping */
static uint32_t  burtcCount = 0;
static uint32_t  burtcOverflowCounter = 0;
static uint32_t  burtcOverflowInterval;   
static uint32_t  burtcOverflowIntervalRem;
static uint32_t  burtcTimestamp;
static time_t    startTime;
static time_t    currentTime;

/* TFT defines */
#define LINE_HEIGHT      8 /* Line height (in pixels) */
#define CHAR_WIDTH       8 /* Character width (in pixels) */

/* Declare variables for TFT output*/
static GLIB_Context_t  gc;
static bool redraw = false;
static char tftStringBuf[40];
static char* tftString = tftStringBuf;
static int tftStrLen;
static bool displayUpdate = true;

/* Interrupt pin used to detect DVK button/joystick activity */
#define GPIO_INT_PORT    gpioPortE
#define GPIO_INT_PIN     0

/* DVK variables for clock adjustment */
static uint16_t  dvkIrqsource = 0;
static uint16_t  dvkButtons = 0;
static uint16_t  dvkJoystick = 0;


/* Clock defines */
//#define LFXO_FREQUENCY 32768
//#define BURTC_PRESCALING 128
//#define UPDATE_INTERVAL 1
//#define COUNTS_PER_SEC (LFXO_FREQUENCY/BURTC_PRESCALING)
//#define COUNTS_BETWEEN_UPDATE (UPDATE_INTERVAL*COUNTS_PER_SEC)

/******************************************************************************
 * @brief GPIO Even Interrupt Handler
 *
 *  Reads DVK joystick and button state upon GPIO interrupt, 
 *  and adjusts clock accordingly.
 *
 ******************************************************************************/
void GPIO_EVEN_IRQHandler(void)
{
  /* Clear interrupt */
  dvkIrqsource = BSP_InterruptFlagsGet();
  BSP_InterruptFlagsClear(dvkIrqsource);

  /* Clear GPIO interrupt */
  GPIO_IntClear(1 << GPIO_INT_PIN);
  
  if ( dvkIrqsource & BC_INTFLAG_JOYSTICK ) 
  {
    /* Read joystick state */
    dvkJoystick = BSP_JoystickGet();

    /* ---------- Adjust YEAR ---------- */
    if ( dvkButtons & BC_UIF_PB1 )
    {
      startTime = clockGetStartTime( );  
      calendar = * localtime( &startTime );
      if ( dvkJoystick & BC_UIF_JOYSTICK_DOWN )
      {
        calendar.tm_year--;
      }
      else if ( dvkJoystick & BC_UIF_JOYSTICK_UP )
      {
        calendar.tm_year++;
      }
      /* Set new epoch offset */
      startTime = mktime(&calendar);
      clockSetStartTime( startTime );  
      clockAppBackup( );
      displayUpdate = true;
    }

    /* ---------- Adjust MONTH+DAY ---------- */
    if ( dvkButtons & BC_UIF_PB2 )
    {
      startTime = clockGetStartTime( );  
      calendar = * localtime( &startTime );
      if ( dvkJoystick & BC_UIF_JOYSTICK_DOWN )
      {
        calendar.tm_mon--;
      }
      else if ( dvkJoystick & BC_UIF_JOYSTICK_UP )
      {
        calendar.tm_mon++;
      }
      else if ( dvkJoystick & BC_UIF_JOYSTICK_LEFT )
      {
        calendar.tm_mday--;
      }
      else if ( dvkJoystick & BC_UIF_JOYSTICK_RIGHT )
      {
        calendar.tm_mday++;
      }
      /* Set new epoch offset */
      startTime = mktime(&calendar);
      clockSetStartTime( startTime );  
      clockAppBackup( );
      displayUpdate = true;
    }

    /* ---------- Adjust HOUR+MIN ---------- */
    if ( dvkButtons & BC_UIF_PB3 )
    {
      startTime = clockGetStartTime( );  
      calendar = * localtime( &startTime );
      if ( dvkJoystick & BC_UIF_JOYSTICK_DOWN )
      {
        calendar.tm_hour--;
      }
      else if ( dvkJoystick & BC_UIF_JOYSTICK_UP )
      {
        calendar.tm_hour++;
      }
      else if ( dvkJoystick & BC_UIF_JOYSTICK_LEFT )
      {
        calendar.tm_min--;
      }
      else if ( dvkJoystick & BC_UIF_JOYSTICK_RIGHT )
      {
        calendar.tm_min++;
      }
      /* Set new epoch offset */
      startTime = mktime(&calendar);
      clockSetStartTime( startTime );  
      clockAppBackup( );
      displayUpdate = true;
    }
  }

  /* Read push button state */
  if ( dvkIrqsource & BC_INTFLAG_PB )
  {
    dvkButtons = BSP_PushButtonsGet();

    /* ---------- Adjust (reset) SEC ---------- */
    if ( dvkButtons & BC_UIF_PB4 )
    {
      /* Get current time */
      time_t t = time(NULL);
      struct tm c = * localtime(&t);

      /* Get epoch offset */
      startTime = clockGetStartTime( );  
      calendar = * localtime( &startTime );

      /* Adjust epoch offset by subtracting current seconds */
      calendar.tm_sec -= c.tm_sec;
      
      /* Set new epoch offset */
      startTime = mktime(&calendar);
      clockSetStartTime( startTime );  
      clockAppBackup( );
      displayUpdate = true;
    }
  }
}



/***************************************************************************//**
 * @brief RTC Interrupt Handler, invoke callback if defined.
 *        The interrupt table is in assembly startup file startup_efm32gg.s
 *        Do critical tasks in interrupt handler. Other tasks are handled in main
 *        while loop.
 ******************************************************************************/
void BURTC_IRQHandler(void)
{
  /* Interrupt source: compare match */
  /*   Increment compare value and
   *   update TFT display            */
  if ( BURTC_IntGet() & BURTC_IF_COMP0 )
  {
    BURTC_CompareSet( 0, BURTC_CompareGet(0) + COUNTS_BETWEEN_UPDATE );
    BURTC_IntClear( BURTC_IF_COMP0 );
  }

  /* Interrupt source: counter overflow */
  /*   Increase overflow counter 
   *   and backup calendar              */
  if ( BURTC_IntGet() & BURTC_IF_OF )
  {
    clockOverflow( );
    clockAppBackup();
    BURTC_IntClear( BURTC_IF_OF );
  }

  displayUpdate = true;
}



/***************************************************************************//**
 * @brief Initialize application
 *
 ******************************************************************************/
void clockAppInit(void)
{
  /* Compute overflow interval (integer) and remainder */
  burtcOverflowInterval  =  ((uint64_t)UINT32_MAX+1)/COUNTS_BETWEEN_UPDATE; /* in seconds */
  burtcOverflowIntervalRem = ((uint64_t)UINT32_MAX+1)%COUNTS_BETWEEN_UPDATE;

  /* Set BURTC compare value for first wakeup */
  BURTC_CompareSet( 0, COUNTS_BETWEEN_UPDATE );

  /* Initialize Development Kit */
  BSP_Init(BSP_INIT_DEFAULT);
  
  /* Initialize interrupts for DVK */
  dvkIrqInit();
  gpioIrqInit();

  /* Initialize TFT for output */
  dvkTftSetup();
}



/***************************************************************************//**
 * @brief  Backup CALENDAR to retention registers
 *
 *   RET[0].REG : number of BURTC overflows
 *   RET[1].REG : epoch offset
 *
 ******************************************************************************/
void clockAppBackup(void)
{
  /* Write overflow counter to retention memory */
  BURTC_RetRegSet( 0, clockGetOverflowCounter() );

  /* Write local epoch offset to retention memory */
  BURTC_RetRegSet( 1, clockGetStartTime() );
}



/***************************************************************************//**
 * @brief  Restore CALENDAR from retention registers
 *
 *  @param[in] burtcCountAtWakeup BURTC value at power up. Only used for printout
 *
 ******************************************************************************/
void clockAppRestore(uint32_t burtcCountAtWakeup)
{
  uint32_t burtcStart;
  uint32_t nextUpdate ;

  /* Store current BURTC value for consistency in display output within this function */
  burtcCount = BURTC_CounterGet();

  /* Timestamp is BURTC value at time of main power loss */
  burtcTimestamp = BURTC_TimestampGet();

  /* Read overflow counter from retention memory */  
  burtcOverflowCounter = BURTC_RetRegGet( 0 );

  /* Check for overflow while in backup mode 
     Assume that overflow interval >> backup source capacity 
     i.e. that overflow has only occured once during main power loss */
  if ( burtcCount < burtcTimestamp )
  {
    burtcOverflowCounter++;
  }
  
  /* Restore epoch offset from retention memory */
  clockSetStartTime( BURTC_RetRegGet( 1 ) );

  /* Restore clock overflow counter */
  clockSetOverflowCounter(burtcOverflowCounter);

  /* Calculate start point for current BURTC count cycle 
     If (COUNTS_BETWEEN_UPDATE/burtcOverflowInterval) is not an integer,
     BURTC value at first update is different between each count cycle */
  burtcStart = (burtcOverflowCounter * (COUNTS_BETWEEN_UPDATE - burtcOverflowIntervalRem)) % COUNTS_BETWEEN_UPDATE;

  /*  Calculate next update compare value 
      Add 1 extra UPDATE_INTERVAL to be sure that counter doesn't 
      pass COMP value before interrupts are enabled */
  nextUpdate = burtcStart + ((burtcCount / COUNTS_BETWEEN_UPDATE) +1 ) * COUNTS_BETWEEN_UPDATE ;
  BURTC_CompareSet( 0, nextUpdate );

  clockAppPrintWakeupStatus(burtcCountAtWakeup);
}



/***************************************************************************//**
 * @brief  Show current time on TFT display
 *
 ******************************************************************************/
void clockAppDisplay(void)
{
  /* Check if MCU have control over TFT display instead of AEM/board controller */
  redraw = TFT_AddressMappedInit();
  if ( redraw && displayUpdate )
  {
    currentTime = time( NULL );
    calendar = * localtime( &currentTime );
    strftime( tftString, 40, "%Y-%m-%d %H:%M:%S", &calendar );

    /* Print string to TFT display using GLIB_drawString routine */
    GLIB_drawString( &gc, tftString, 40, 0, 0, 1 );
    
    /* Clear displayUpdate flag */
    displayUpdate = false;
  }
}
    


/***************************************************************************//**
* @brief  Setup TFT display
 *
 ******************************************************************************/
void dvkTftSetup(void)
{
  EMSTATUS       status;
  GLIB_Rectangle_t rect = {
    .xMin =   0,
    .yMin =   0,
    .xMax = 319,
    .yMax = 239,
  };

  /* Wait until we have control over display */
  while (!redraw)
  {
    redraw = TFT_AddressMappedInit();
  }

  /* Init graphics context - abort on failure */
  status = GLIB_contextInit(&gc);
  if (status != GLIB_OK) while (1) ;

  /* Clear framebuffer */
  gc.foregroundColor = Black;
  GLIB_drawRectFilled(&gc, &rect);
  gc.foregroundColor = White;
}



/***************************************************************************//**
 * @brief Initialize GPIO interrupt for DVK buttons
 *
 ******************************************************************************/
void gpioIrqInit(void)
{
  /* Configure interrupt pin as input with pull-up */
  GPIO_PinModeSet(GPIO_INT_PORT, GPIO_INT_PIN, gpioModeInputPull, 1);

  /* Set falling edge interrupt and clear/enable it */
  GPIO_IntConfig(GPIO_INT_PORT, GPIO_INT_PIN, false, true, true);

  NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
  NVIC_EnableIRQ(GPIO_EVEN_IRQn);
}



/***************************************************************************//**
 * @brief Enable DVK FPGA to generate GPIO PC14 trigger on control updates
 *
 ******************************************************************************/
void dvkIrqInit(void)
{
  /* Enable interrupts on push button events only */
  BSP_InterruptDisable(0xffff);
  BSP_InterruptFlagsClear(0xffff);
  BSP_InterruptEnable(BC_INTEN_PB);
  BSP_InterruptEnable(BC_INTEN_JOYSTICK);
}



/***************************************************************************//**
 * @brief  Prints out RMU->RSTCAUSE 
 *
 ******************************************************************************/
void clockAppPrintResetCause(uint32_t resetCause)
{
    tftStrLen = snprintf(tftString, 40, "Last reset cause: 0x%04X", (unsigned int) resetCause);
    GLIB_drawString(&gc, tftString, tftStrLen, 0 * CHAR_WIDTH, 2 * LINE_HEIGHT, 1);
}



/***************************************************************************//**
 * @brief  Prints out "recover-after-backup" status info on TFT display
 *
 *   Input argument is BURTC counter value at wakeup from backup mode
 *
 ******************************************************************************/
void clockAppPrintWakeupStatus(uint32_t burtcCountAtWakeup)
{
  tftStrLen = snprintf(tftString, 40, "BURTC->STATUS: 0x%08X", (unsigned int) BURTC_Status() );
  GLIB_drawString(&gc, tftString, tftStrLen , 0 * CHAR_WIDTH, 6 * LINE_HEIGHT, 1);

  tftStrLen = snprintf(tftString, 40, "BURTC->TS:  %u", (unsigned int) burtcTimestamp);
  GLIB_drawString(&gc, tftString, tftStrLen, 0 * CHAR_WIDTH, 8 * LINE_HEIGHT, 1);

  tftStrLen = snprintf(tftString, 40, "BURTC->CNT: %u", (unsigned int) burtcCountAtWakeup);
  GLIB_drawString(&gc, tftString, tftStrLen, 0 * CHAR_WIDTH, 9 * LINE_HEIGHT, 1);

  double secondsInBackup = (burtcCountAtWakeup - burtcTimestamp) / (double)COUNTS_PER_SEC;
  tftStrLen = snprintf(tftString, 40, "=> Time in backup: %0.2f s", secondsInBackup);
  GLIB_drawString(&gc, tftString, tftStrLen, 0 * CHAR_WIDTH, 10 * LINE_HEIGHT, 1);

  time_t backupTime = clockGetStartTime() + burtcOverflowCounter*burtcOverflowInterval + burtcOverflowCounter*burtcOverflowIntervalRem/COUNTS_PER_SEC + (burtcTimestamp/COUNTS_PER_SEC);
  calendar = * localtime ( &backupTime );
  tftStrLen = strftime(tftString,40,"Power out: %Y-%m-%d %H:%M:%S", &calendar );
  GLIB_drawString(&gc, tftString, tftStrLen, 0 * CHAR_WIDTH, 12 * LINE_HEIGHT, 1);

  time_t wakeupTime = clockGetStartTime() + burtcOverflowCounter*burtcOverflowInterval + burtcOverflowCounter*burtcOverflowIntervalRem/COUNTS_PER_SEC + (burtcCountAtWakeup/COUNTS_PER_SEC);
  calendar = * localtime ( &wakeupTime );
  tftStrLen = strftime(tftString,40,"Power up:  %Y-%m-%d %H:%M:%S", &calendar );
  GLIB_drawString(&gc, tftString, tftStrLen, 0 * CHAR_WIDTH, 13 * LINE_HEIGHT, 1);
}



/***************************************************************************//**
 * @brief  Prints out "No Timestamp" message on TFT display
 *
 ******************************************************************************/
void clockAppPrintNoTimestamp(void)
{  
  GLIB_drawString(&gc, "NO TIMESTAMP", 12, 0*CHAR_WIDTH, 15*LINE_HEIGHT, 1);
}



/***************************************************************************//**
 * @brief  Prints out "RAM Write Error" message on TFT display
 *
 ******************************************************************************/
void clockAppPrintRamWErr(void)
{
  GLIB_drawString(&gc, "RAMWERR", 8, 0*CHAR_WIDTH, 16*LINE_HEIGHT, 1);
}
