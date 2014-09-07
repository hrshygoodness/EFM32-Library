/***************************************************************************//**
 * @file clock.c
 * @brief System clock for tickless calendar application note
 * @author Silicon Labs
 * @version 2.06
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
#include <time.h>
#include <stddef.h>

/* Include emlib */
#include "em_device.h"

/* Include system clock*/
#include "clock.h"

/* RTC variables. Used for converting RTC counter to system time */
static uint16_t   rtcCountsPerSec       = 0;
static time_t     rtcStartTime          = 0;
static uint32_t   rtcOverflowCounter    = 0;
static uint32_t   rtcOverflowInterval   = 0;
static uint32_t   rtcOverflowIntervalR  = 0;



/******************************************************************************
 * @brief Returns the current system time
 *
 * @param timer
 *   If not a null pointer, time is copied to this
 *
 * @return
 *   Current system time. Should, but does not, return -1 if system time is not available
 *
 *****************************************************************************/
#if defined (__ICCARM__)
time_t __time32( time_t * timer )
#elif defined (__CC_ARM)
time_t time( time_t * timer )
#elif defined (__GNUC__)
time_t time( time_t * timer )
#else
#error Undefined toolkit, need to define alignment 
#endif
{
  time_t t;

  /* Add the time offset */
  t = rtcStartTime;

  /* Add time based on number of counter overflows*/
  t += rtcOverflowCounter * rtcOverflowInterval;

  /* Add remainder if the overflow interval is not an integer */   
  if ( rtcOverflowIntervalR != 0 )
  {
    t += (rtcOverflowCounter * rtcOverflowIntervalR) / rtcCountsPerSec;
  }
  
  /* Add the number of seconds for RTC */
  t += ( RTC->CNT / rtcCountsPerSec );

  /* Copy system time to timer if not NULL*/  
  if ( timer != NULL )
  {
    *timer = t;
  }

  return t;
}



/***************************************************************************//**
 * @brief Initialize system CLOCK
 *
 * @param[in] timeptr
 *   Calendar struct that is used to set the start time of the counter.
 *
 ******************************************************************************/
void clockInit(Clock_Init_TypeDef *init)
{
  /* Store configuration variables */
  rtcCountsPerSec = init->rtcCountsPerSec;

  /* Reset overflow counter */
  rtcOverflowCounter = 0;

  /* Calculate overflow interval based on RTC counter width and frequency */
  rtcOverflowInterval   = ((0x00FFFFFF+1) / rtcCountsPerSec);
  rtcOverflowIntervalR  = ((0x00FFFFFF+1) % rtcCountsPerSec); /* remainder */

  /* Set epoch offset */
  clockSetStartCalendar( &init->startDate);
}



/***************************************************************************//**
 * @brief Set the epoch offset
 *
 * @param[in] timeptr
 *   Calendar struct which is converted to unix time and used as new epoch 
 *   offset
 *
 ******************************************************************************/
void clockSetStartCalendar(struct tm * timeptr)
{
  rtcStartTime = mktime(timeptr); 
}



/***************************************************************************//**
 * @brief Set the epoch offset
 *
 * @param[in] offset
 *   unix time when the counter was started
 *
 ******************************************************************************/
void clockSetStartTime(time_t offset)
{
  rtcStartTime = offset;
}



/***************************************************************************//**
 * @brief Get the epoch offset
 *
 * @return 
 *   unix time when the counter was started
 *
 ******************************************************************************/
time_t clockGetStartTime(void)
{
  return rtcStartTime;
}






/***************************************************************************//**
 * @brief Call this function on counter overflow to let CLOCK know how many
 *        overflows has occured since start time
 *
 ******************************************************************************/
uint32_t clockOverflow(void)
{
  rtcOverflowCounter++;
  return rtcOverflowCounter;
}



/***************************************************************************//**
 * @brief Call this function on counter overflow to let CLOCK know how many
 *        overflows has occured since start time
 *
 ******************************************************************************/
void clockSetOverflowCounter(uint32_t of)
{
  rtcOverflowCounter = of;
}



/***************************************************************************//**
 * @brief Call this function on counter overflow to let CLOCK know how many
 *        overflows has occured since start time
 *
 ******************************************************************************/
uint32_t clockGetOverflowCounter(void)
{
  return rtcOverflowCounter;
}
