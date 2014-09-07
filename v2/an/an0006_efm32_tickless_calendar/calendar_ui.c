/***************************************************************************//**
 * @file calendar_ui.c
 * @brief User interface for tickless calendar application note
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
#include <stdint.h>
#include <stddef.h>

/* Include emlib */
#include "em_device.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_rtc.h"

/* Include temperature compensated clock */
#include "clock_tc.h"


/* Include Segment LCD driver for STK */
#include "segmentlcd.h"

/* Include calendar user interface */
#include "calendar_ui.h"

/* Calendar struct used as temporary storage */
struct tm calendar;

/* Structs for modules used */
LCD_TypeDef *lcd = LCD;

/* Declare variables for LCD output*/
static char  displayStringBuf[6];
static char* displayString = displayStringBuf;


/* Map GPIOs to push buttons for different EFM32 kits */
#if defined ( STK3700 )
  #define PB0_PORT gpioPortB
  #define PB0_PIN 9 
  #define PB1_PORT gpioPortB
  #define PB1_PIN 10 
#elif defined ( STK3300 )
  #define PB0_PORT gpioPortD
  #define PB0_PIN 8
  #define PB1_PORT gpioPortB
  #define PB1_PIN 11 
#elif defined ( STKG8XX )
  #define PB0_PORT gpioPortB
  #define PB0_PIN 9
  #define PB1_PORT gpioPortB
  #define PB1_PIN 10 
#else
  #error "undefined KIT"
#endif


/***************************************************************************//**
 * @brief Initialize application
 *
 ******************************************************************************/
void uiInit(void)
{
  /* Initialize GPIO with interrupts for STK */
  uiGpioInit();

  /* Initialize LCD without voltage boost */
  SegmentLCD_Init(false);

  /* Turn on EFM32 logo and Gecko*/  
  SegmentLCD_Symbol(LCD_SYMBOL_EFM32, 1);
  SegmentLCD_Symbol(LCD_SYMBOL_GECKO, 1);
}



/***************************************************************************//**
 * @brief  Show current time on LCD display
 *
 ******************************************************************************/
void uiDisplay(time_t* t)
{
  /* Read out system time if input parameter = NULL */
  if ( t == NULL )
  {
    time_t currentTime = time( NULL );
    t = &currentTime;
  }

  /* Make calendar from unix time */
  calendar = * localtime( t );

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
  
}
   




/***************************************************************************//**
 * @brief Initialize GPIO interrupt for STK buttons
 *
 ******************************************************************************/
void uiGpioInit(void)
{
  /* Enable clock to GPIO */
  CMU_ClockEnable(cmuClock_GPIO, true);

  /* Configure GPIO for Push Button 0 as an input. Enable interrupt on falling edge */
  GPIO_PinModeSet(PB0_PORT, PB0_PIN, gpioModeInput, 0);
  GPIO_IntConfig(PB0_PORT, PB0_PIN, false, true, true);

  /* Configure GPIO for Push Button 1 as an input. Enable interrupt on falling edge */
  GPIO_PinModeSet(PB1_PORT, PB1_PIN, gpioModeInput, 0);
  GPIO_IntConfig(PB1_PORT, PB1_PIN, false, true, true );

  /* Clear and enable interrupts */
  NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
  NVIC_EnableIRQ(GPIO_EVEN_IRQn);
  NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
  NVIC_EnableIRQ(GPIO_ODD_IRQn);
}

/******************************************************************************
 * @brief GPIO Even Interrupt Handler
 *
 *  Increment hour/minute (kit dependent) on GPIO interrupt from even numbered pins.
 *  Assume no other even GPIO interrupts are enabled.
 *
 ******************************************************************************/
void GPIO_EVEN_IRQHandler(void)
{
  time_t startTime;

  /* Read out time that corresponds to RTC count = 0 */
  startTime = clockGetStartTime( );
  calendar = * localtime( &startTime );


  /* Kit specific implementation of even GPIO interrupt */
#if defined ( STK3700 )
  /* Clear interrupt for GPIO port PB_EVEN_PIN */
  GPIO_IntClear( 1 << PB1_PIN );

  /* Add 1 minute */
  calendar.tm_min++;
  
#elif defined (STK3300)
  /* Clear interrupt for GPIO port PB_EVEN_PIN */
  GPIO_IntClear( 1 << PB0_PIN );

  /* Add 1 hour */
  calendar.tm_hour++;
  
#elif defined (STKG8XX)
  /* Clear interrupt for GPIO port PB_EVEN_PIN */
  GPIO_IntClear( 1 << PB1_PIN );

  /* Add 1 minute */
  calendar.tm_min++ ;
  
#else
  #error "undefined KIT"
#endif


  /* Write new start time to clock */
  startTime = mktime( &calendar );
  clockSetStartTime( startTime );

  /* Update display with new time */
  uiDisplay(NULL);
}



/******************************************************************************
 * @brief GPIO Odd Interrupt Handler
 *
 *  Increment hour/minute (kit dependent) on GPIO interrupt from odd numbered pins.
 *  Assume no other odd GPIO interrupts are enabled.
 *
 ******************************************************************************/
void GPIO_ODD_IRQHandler(void)
{
  time_t startTime;

  /* Read out time that corresponds to RTC count = 0 */
  startTime = clockGetStartTime( );
  calendar = * localtime( &startTime );

  
  /* Kit specific implementation of even GPIO interrupt */
#if defined ( STK3700 )
  /* Clear interrupt for GPIO port PB_EVEN_PIN */
  GPIO_IntClear( 1 << PB0_PIN );

  /* Add 1 hour */
  calendar.tm_hour++;
  
#elif defined (STK3300)
  /* Clear interrupt for GPIO port PB_EVEN_PIN */
  GPIO_IntClear( 1 << PB1_PIN );

  /* Add 1 minute */
  calendar.tm_min++;
  
#elif defined (STKG8XX)
  /* Clear interrupt for GPIO port PB_EVEN_PIN */
  GPIO_IntClear( 1 << PB0_PIN );

  /* Add 1 hour */
  calendar.tm_hour++ ;
  
#else
  #error "undefined KIT"
#endif

  /* Write new start time to clock */
  startTime = mktime( &calendar );
  clockSetStartTime( startTime );

  /* Update display with new time */
  uiDisplay(NULL);
}

