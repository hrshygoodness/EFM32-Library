/***************************************************************************//**
 * @file clock_tc.c
 * @brief Temperature compensated system clock for tickless calendar application note
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
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <time.h>


/* Include emlib */
#include "em_adc.h"
#include "em_cmu.h"
#include "em_device.h"
#include "em_emu.h"
#include "em_int.h"

/* Include temperature compensated system clock */
#include "clock_tc.h"
#include "crystal_parameters.h"

/* Declare static functions */
static void clockTempSensorInit(void);
static int8_t clockReadThermometer(void);


/* EFM32 temperature sensor gradient */
#define THERMOMETER_GRADIENT  (-6.3)

/* RTC variables. Used for converting RTC counter to system time */
static uint16_t   rtcCountsPerSec       = 0;
static time_t     rtcStartTime          = 0;
static uint32_t   rtcOverflowCounter    = 0;
static uint32_t   rtcOverflowInterval   = 0;
static uint32_t   rtcOverflowIntervalR  = 0;

/* Thermometer calibration variables */
static uint8_t    thermCalTemp;
static uint16_t   thermCalValue;

/* Temperature compensation variables */
static int32_t    tempCompAccumulator   = 0;
static int32_t    tempCompInterval      = 0;


#if defined ( __CROSSWORKS_ARM )
double round( double n ) 
{
  double ret;
  if ( n >= (double)0.0 )
  {
    if ( (n - floor(n)) >= (double)0.5 )
      ret = ceil(n);
    else
      ret = floor(n);
  }
  else
  {
    if ( (n - ceil(n)) <= (double)(-0.5) )
      ret = floor(n);
    else
      ret = ceil(n);
  }
  
  return ret;
}
#endif

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

  /* Add compensation for crystal temperature drift */
  t += tempCompAccumulator;

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
 * @param[in] init
 *   Clock_Init_TypeDef struct that is used to set the start time of the counter
 *   and the RTC count frequency.
 *
 ******************************************************************************/
void clockInit(Clock_Init_TypeDef *init)
{
  /* Store configuration variables */
  rtcCountsPerSec = init->rtcCountsPerSec;
  tempCompInterval = init->tempCompInterval;

  /* Reset overflow counter */
  rtcOverflowCounter = 0;

  /* Calculate overflow interval based on RTC counter width and frequency */
  rtcOverflowInterval   = ((0x00FFFFFF+1) / rtcCountsPerSec);
  rtcOverflowIntervalR  = ((0x00FFFFFF+1) % rtcCountsPerSec); /* remainder */

  /* Set epoch offset */
  clockSetStartCalendar( &init->startDate);

  /* Set up ADC for internal temperature sensor */
  clockTempSensorInit();
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



/******************************************************************************
 * @brief Initializes ADC for temperature measurement
 *
 *****************************************************************************/
static void clockTempSensorInit(void)
{
  /* Read thermometer calibration temperature from DI page */
  thermCalTemp = ((DEVINFO->CAL & _DEVINFO_CAL_TEMP_MASK) >> _DEVINFO_CAL_TEMP_SHIFT);  

  /* Read thermometer calibration value from DI page */
  thermCalValue = ((DEVINFO->ADC0CAL2 & _DEVINFO_ADC0CAL2_TEMP1V25_MASK) >> _DEVINFO_ADC0CAL2_TEMP1V25_SHIFT);      

#if defined(_EFM32_GIANT_FAMILY)
  /* This is a work around for Chip Rev.D Errata, Revision 0.6. */
  /* Check for product revision 16 and 17 and set the offset */
  /* for ADC0_TEMP_0_READ_1V25. */
  uint8_t prodRev = (DEVINFO->PART & _DEVINFO_PART_PROD_REV_MASK) >> _DEVINFO_PART_PROD_REV_SHIFT;

  if( (prodRev == 16) || (prodRev == 17) )
  {
      thermCalValue -= 112;
  }
#endif


  /* Enable ADC peripheral clock */
  CMU_ClockEnable(cmuClock_ADC0, true);

  /* Base the ADC configuration on the default setup. */
  ADC_Init_TypeDef init = ADC_INIT_DEFAULT;
  ADC_InitSingle_TypeDef sInit = ADC_INITSINGLE_DEFAULT;

  /* Initialize timebases */
  init.timebase = ADC_TimebaseCalc(0);
  init.prescale = ADC_PrescaleCalc(400000,0);
  ADC_Init(ADC0, &init);

  /* Set input to temperature sensor. Reference must be 1.25V */
  sInit.reference = adcRef1V25;
  sInit.input = adcSingleInpTemp;
  ADC_InitSingle(ADC0, &sInit);

  /* Setup interrupt generation on completed conversion. */
  ADC_IntEnable(ADC0, ADC_IEN_SINGLE);
  NVIC_EnableIRQ(ADC0_IRQn);

}



/******************************************************************************
 * @brief Starts a temp measurement
 *
 *****************************************************************************/
static int8_t clockReadThermometer(void)
{ 
  uint32_t sample;
  double  temp;
  
  /* Start one ADC sample */
  ADC_Start(ADC0, adcStartSingle);

  while ( !(ADC0->STATUS & ADC_STATUS_SINGLEDV ) )
  {
    /* Wait in EM1 for ADC to complete */
    INT_Disable();
    EMU_EnterEM1();
    INT_Enable();
  }

  /* Read sensor value */
  sample = ADC_DataSingleGet(ADC0);

  /* Convert temperature reading to degrees celsius */
  temp = thermCalTemp - (((double)thermCalValue -(double)sample)/THERMOMETER_GRADIENT);

  /* Return temperature as an integer */
  return (int8_t)round(temp);
}



/***************************************************************************//**
 * @brief Call this function when CLOCK should perform a temperature
 *        reading and compensate for oscillator drift
 * @return Returns total temperature compensation offset
 ******************************************************************************/
int32_t clockDoTemperatureCompensation(void)
{
  int8_t temperature;
  uint8_t temperatureOffset;
  static int32_t accumulator = 0;
  
  /* Read temperature from internal thermometer */
  temperature = clockReadThermometer();
  temperatureOffset = abs( temperature - CRYSTAL_NOMINAL_TEMP );
  
  /* Checking value*/
  if (temperatureOffset > CRYSTAL_COMPENSATION_TABLE_LENGTH - 1)
  { 
    temperatureOffset = CRYSTAL_COMPENSATION_TABLE_LENGTH - 1;
  }
    
  /* Get additional accumulator value from lookup table using temperature value
     It is assumed that the temperature was the same during the sleep period
     so the value from the lookup table is multiplied by the number of sleep seconds
     Update accumulator with new value */
  accumulator += (crystalCompensationTable[temperatureOffset] * tempCompInterval);
 
  /* Check if accumulated delay is more than 1 second */
  while( accumulator > ( 1000*(int32_t)rtcCountsPerSec ) )
  {
    /* if above threshold, decrement accumulator and subtract exceeding count */
    accumulator -= 1000*rtcCountsPerSec;
    tempCompAccumulator--;
  }

  /* Check if accumulated rush is more than 1 second */
  while( accumulator < ( -1000*(int32_t)rtcCountsPerSec ) )
  {
    /* if above threshold, increment accumulator and add exceeding count */
    accumulator += 1000*rtcCountsPerSec;
    tempCompAccumulator++;
  }

  return tempCompAccumulator;
}



/**************************************************************************//**
 * @brief ADC0 interrupt handler. Simply clears interrupt flag.
 *****************************************************************************/
void ADC0_IRQHandler(void)
{
  ADC_IntClear(ADC0, ADC_IFC_SINGLE);
}

