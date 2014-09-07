/***************************************************************************//**
 * @file clock_tc.h
 * @brief CLOCK_TC header file
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

#ifndef __CLOCK_TC_H
#define __CLOCK_TC_H

#include <time.h>
#include <math.h>

/* Setting up a structure to initialize the calendar
   for January 1 2012 12:00:00
   The struct tm is declared in time.h 
   More information about time.h library is found on http://en.wikipedia.org/wiki/Time.h */  
#if defined (__CS_GNUC__)
#define CLOCK_DEFAULT_START_DATE                                                            \
{                                                                                           \
  0,    /* tm_sec:   0 seconds (0-60, 60 = leap second)*/                                   \
  0,    /* tm_min:   0 minutes (0-59) */                                                    \
  12,   /* tm_hour:  0 hours (0-23) */                                                      \
  1,    /* tm_mday:  1st day of the month (1 - 31) */                                       \
  0,    /* tm_mon:   January (0 - 11, 0 = January) */                                       \
  112,  /* tm_year:  Year 2012 (year since 1900) */                                         \
  0,    /* tm_wday:  Sunday (0 - 6, 0 = Sunday) */                                          \
  0,    /* tm_yday:  1st day of the year (0-365) */                                         \
  -1,   /* tm_isdst: Daylight saving time; enabled (>0), disabled (=0) or unknown (<0) */   \
  0,    /* __extra_1 */                                                                     \
  0     /* __extra_2 */                                                                     \
}
#else
#define CLOCK_DEFAULT_START_DATE                                                            \
{                                                                                           \
  0,    /* tm_sec:   0 seconds (0-60, 60 = leap second)*/                                   \
  0,    /* tm_min:   0 minutes (0-59) */                                                    \
  12,   /* tm_hour:  0 hours (0-23) */                                                      \
  1,    /* tm_mday:  1st day of the month (1 - 31) */                                       \
  0,    /* tm_mon:   January (0 - 11, 0 = January) */                                       \
  112,  /* tm_year:  Year 2012 (year since 1900) */                                         \
  0,    /* tm_wday:  Sunday (0 - 6, 0 = Sunday) */                                          \
  0,    /* tm_yday:  1st day of the year (0-365) */                                         \
  -1    /* tm_isdst: Daylight saving time; enabled (>0), disabled (=0) or unknown (<0) */   \
}
#endif


/** CLOCK initialization structure. */
typedef struct
{
  struct tm startDate;          /**< Date when RTC was started */
  uint32_t rtcCountsPerSec;     /**< RTC count frequency [Hz]*/
  uint32_t tempCompInterval;    /**< Temperature compensation interval [s] */
} Clock_Init_TypeDef;


/** Suggested default config for Clock init structure. */
#define CLOCK_INIT_DEFAULT                                                 \
{                                                                          \
  CLOCK_DEFAULT_START_DATE,   /* Use default start date */                 \
  32768,                      /* RTC frequency is 32.768 kHz */            \
  60,                         /* Compensate for temp drift every 60 sec */ \
}


/* Public function prototypes*/
void clockInit(Clock_Init_TypeDef *init);
void clockSetStartCalendar(struct tm * timeptr);
void clockSetStartTime(time_t offset);
time_t clockGetStartTime(void);
uint32_t clockOverflow(void);
void clockSetOverflowCounter(uint32_t of);
uint32_t clockGetOverflowCounter(void);
int32_t clockDoTemperatureCompensation(void);

#endif
