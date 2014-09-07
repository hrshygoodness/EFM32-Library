/***************************************************************************//**
 * @file clockApp_stk.c
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
#include <stddef.h>

/* Include emlib */
#include "em_burtc.h"
#include "em_cmu.h"
#include "em_device.h"
#include "em_gpio.h"

/* Include BSP drivers */
#include "segmentlcd.h"

/* Include other */
#include "clock.h"
#include "clock_config.h"
#include "clockApp_stk.h"


/* Calendar struct */
static struct tm calendar;

/* Declare variables for time keeping */
//static uint32_t  burtcCount = 0;
static uint32_t  burtcOverflowCounter = 0;
static uint32_t  burtcOverflowIntervalRem;
static uint32_t  burtcTimestamp;
static time_t    startTime;
static time_t    currentTime;


/* Declare variables for LCD output*/
static char displayStringBuf[6];
static char* displayString = displayStringBuf;
static bool displayUpdate = true;



/******************************************************************************
 * @brief GPIO Odd Interrupt Handler
 *
 *  Increment minute on GPIO interrupt from even numbered pins. Assume no 
 *  other odd GPIO interrupts are enabled.
 *
 ******************************************************************************/
void GPIO_ODD_IRQHandler(void)
{
  /* Clear interrupt for GPIO port number 9 */
  GPIO_IntClear( 1 << 9 );

  startTime = clockGetStartTime( );
  calendar = * localtime( &startTime );
  
  /* Add 1 hour */
  calendar.tm_hour++ ;

  /* Write updated time */
  startTime = mktime( &calendar );
  clockSetStartTime( startTime );
  clockAppBackup( );
  displayUpdate = true;
}

/******************************************************************************
 * @brief GPIO Even Interrupt Handler
 *
 *  Increment hour on GPIO interrupt from even numbered pins. Assume no 
 *  other even GPIO interrupts are enabled.
 *
 ******************************************************************************/
void GPIO_EVEN_IRQHandler(void)
{
  /* Clear interrupt for GPIO port number 10 */
  GPIO_IntClear( 1 << 10 );

  startTime = clockGetStartTime( );
  calendar = * localtime( &startTime );
  
  /* Add 1 minute */
  calendar.tm_min++ ;

  startTime = mktime( &calendar );
  clockSetStartTime( startTime );
  clockAppBackup( );
  displayUpdate = true;
}



/***************************************************************************//**
 * @brief RTC Interrupt Handler, invoke callback if defined.
 *        The interrupt table is in assembly startup file startup_efm32gg.s
 *        Do critical tasks in interrupt handler. Other tasks are handled in main
 *        while loop.
 ******************************************************************************/
void BURTC_IRQHandler(void)
{
  /*   Interrupt source: compare match */
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
  burtcOverflowIntervalRem = ((uint64_t)UINT32_MAX+1)%COUNTS_BETWEEN_UPDATE;

  /* Set BURTC compare value for first wakeup */
  BURTC_CompareSet( 0, COUNTS_BETWEEN_UPDATE );
  
  /* Initialize GPIO with interrupts for STK */
  gpioInit();

  /* Initialize LCD without voltage boost */
  SegmentLCD_Init(false);
  /* Select LFXO as clock for LFA and shut off LFRCO (changed by SegmentLCD_Init) */
  CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);
  CMU_OscillatorEnable(cmuOsc_LFRCO, false , false);
  
  /* Turn on Gecko and EFM32 logo */  
  SegmentLCD_Symbol(LCD_SYMBOL_EFM32, 1);
  SegmentLCD_Symbol(LCD_SYMBOL_GECKO, 1);
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
 *
 ******************************************************************************/
void clockAppRestore( void )
{
  uint32_t burtcCount;
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
  clockSetOverflowCounter( burtcOverflowCounter );

  /* Calculate start point for current BURTC count cycle 
     If (COUNTS_BETWEEN_UPDATE/burtcOverflowInterval) is not an integer,
     BURTC value at first update is different between each count cycle */
  burtcStart = (burtcOverflowCounter * (COUNTS_BETWEEN_UPDATE - burtcOverflowIntervalRem)) % COUNTS_BETWEEN_UPDATE;

  /*  Calculate compare value for next display update. 
      Add 1 extra UPDATE_INTERVAL to be sure that counter doesn't 
      pass COMP value before interrupts are enabled */
  nextUpdate = burtcStart + ((burtcCount / COUNTS_BETWEEN_UPDATE) +1 ) * COUNTS_BETWEEN_UPDATE ;
  BURTC_CompareSet( 0, nextUpdate );
}



/***************************************************************************//**
 * @brief  Show current time on TFT display
 *
 ******************************************************************************/
void clockAppDisplay(void)
{
  /* Check if a display update is due */
  if ( displayUpdate )
  {
    currentTime = time( NULL );
    calendar = * localtime( &currentTime );
    
    /* Make string from calendar */
    displayStringBuf[0] = 0x30 + (calendar.tm_hour / 10);
    displayStringBuf[1] = 0x30 + (calendar.tm_hour % 10);;
    displayStringBuf[2] = 0x30 + (calendar.tm_min / 10);
    displayStringBuf[3] = 0x30 + (calendar.tm_min % 10);
    displayStringBuf[4] = 0x30 + (calendar.tm_sec / 10);
    displayStringBuf[5] = 0x30 + (calendar.tm_sec % 10);
    
    /* Print time on LCD */
    SegmentLCD_Write(displayString);
    SegmentLCD_Symbol(LCD_SYMBOL_COL3, 1);
    SegmentLCD_Symbol(LCD_SYMBOL_COL5, 1);

    /* Clear displayUpdate flag */
    displayUpdate = false;
  }
}



/***************************************************************************//**
 * @brief  Prints reset cause on display
 *
 ******************************************************************************/
void clockAppPrintResetCause(uint32_t resetCause)
{
    SegmentLCD_UnsignedHex(resetCause);
}



/***************************************************************************//**
 * @brief Initialize GPIO interrupt for STK buttons
 *
 ******************************************************************************/
void gpioInit(void)
{
  CMU_ClockEnable(cmuClock_GPIO, true);
  
  /* Configure Push Button 0 as an input, and enable interrupt on falling edge */
  GPIO_PinModeSet(gpioPortB, 9, gpioModeInput, 0);
  GPIO_IntConfig(gpioPortB, 9, false, true, true);

  /* Configure Push Button 1 as an input, and enable interrupt on falling edge */
  GPIO_PinModeSet(gpioPortB, 10, gpioModeInput, 0);
  GPIO_IntConfig(gpioPortB, 10, false, true, true);

  /* Clear and enable interrupts */
  NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
  NVIC_EnableIRQ(GPIO_EVEN_IRQn);
  NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
  NVIC_EnableIRQ(GPIO_ODD_IRQn);
}
