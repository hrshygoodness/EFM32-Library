/**************************************************************************//**
 * @file main.c
 * @brief Clock example for EFM32ZG-STK3200
 * @version 3.20.5
 *
 * This example shows how to optimize your code in order to drive
 * a graphical display in an energy friendly way.
 *
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 *
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <stdio.h>
#include <time.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_pcnt.h"
#include "em_prs.h"
#include "display.h"
#include "glib.h"

/* Frequency of RTC (COMP0) pulses on PRS channel 2 
   = frequency of LCD polarity inversion. */
#define RTC_PULSE_FREQUENCY    (64)

/* Clock mode */
typedef enum
{
  CLOCK_MODE_ANALOG,
  CLOCK_MODE_DIGITAL
} ClockMode_t;
volatile ClockMode_t clockMode = CLOCK_MODE_ANALOG;

/* The current time reference. Number of seconds since midnight
 * January 1, 1970.  */
static volatile time_t curTime = 0;

/* PCNT interrupt counter */
static volatile int pcntIrqCount = 0;

/* Flag to check when we should redraw a frame */
static volatile bool updateDisplay = true;
static volatile bool timeIsFastForwarding = false;

/* Global glib context */
GLIB_Context_t gc;

/* Analog clock prototypes */
void analogClockInitGraphics(void);
void analogClockUpdate(struct tm *t, bool forceRedraw);


/**************************************************************************//**
 * @brief Setup GPIO interrupt for pushbuttons.
 *****************************************************************************/
static void gpioSetup(void)
{
  /* Enable GPIO clock */
  CMU_ClockEnable(cmuClock_GPIO, true);

  /* Configure PC8 as input and enable interrupt  */
  GPIO_PinModeSet(gpioPortC, 8, gpioModeInputPull, 1);
  GPIO_IntConfig(gpioPortC, 8, false, true, true);

  NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
  NVIC_EnableIRQ(GPIO_EVEN_IRQn);

  /* Configure PC9 as input and enable interrupt */
  GPIO_PinModeSet(gpioPortC, 9, gpioModeInputPull, 1);
  GPIO_IntConfig(gpioPortC, 9, false, true, true);

  NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
  NVIC_EnableIRQ(GPIO_ODD_IRQn);
}

/**************************************************************************//**
 * @brief GPIO Interrupt handler (PB0)
 *        Switches between analog and digital clock modes.
 *****************************************************************************/
void GPIO_EVEN_IRQHandler(void)
{
  /* Acknowledge interrupt */
  GPIO_IntClear(1 << 8);

  /* Toggle clock mode (analog/digital) */
  clockMode = (clockMode == CLOCK_MODE_ANALOG) ? CLOCK_MODE_DIGITAL : CLOCK_MODE_ANALOG;
  updateDisplay = true;
}

/**************************************************************************//**
 * @brief GPIO Interrupt handler (PB1)
 *        Increments the time by one minute.
 *****************************************************************************/
void GPIO_ODD_IRQHandler(void)
{
  /* Acknowledge interrupt */
  GPIO_IntClear(1 << 9);

  /* Increase time by 1 second. */
  curTime++;

  timeIsFastForwarding = true;
  updateDisplay = true;
}


/**************************************************************************//**
 * @brief   Set up PCNT to generate an interrupt every second.
 *
 *****************************************************************************/
void pcntInit(void)
{
  PCNT_Init_TypeDef pcntInit = PCNT_INIT_DEFAULT;

  /* Enable PCNT clock */
  CMU_ClockEnable(cmuClock_PCNT0, true);
  /* Set up the PCNT to count RTC_PULSE_FREQUENCY pulses -> one second */
  pcntInit.mode = pcntModeOvsSingle;
  pcntInit.top = RTC_PULSE_FREQUENCY;
  pcntInit.s1CntDir = false;
  pcntInit.s0PRS = pcntPRSCh2;

  PCNT_Init(PCNT0, &pcntInit);
  
  /* Select PRS as the input for the PCNT */
  PCNT_PRSInputEnable(PCNT0, pcntPRSInputS0, true);
  
  /* Enable PCNT interrupt every second */
  NVIC_EnableIRQ(PCNT0_IRQn);
  PCNT_IntEnable(PCNT0, PCNT_IF_OF);
}


/**************************************************************************//**
 * @brief   This interrupt is triggered at every second by the PCNT
 *
 *****************************************************************************/
void PCNT0_IRQHandler(void)
{
  PCNT_IntClear(PCNT0, PCNT_IF_OF);
  
  pcntIrqCount++;
  
  /* Increase time with 1s */
  if (!(timeIsFastForwarding))
  {
    curTime++;
  }

  /* Notify main loop to redraw clock on display. */
  updateDisplay = true;
}

/**************************************************************************//**
 * @brief  Increments the clock quickly while PB1 is pressed.
 *         A callback is used to update either the analog or the digital clock.
 *
 *****************************************************************************/
void fastForwardTime(void (*drawClock)(struct tm*, bool redraw))
{
  unsigned int i = 0;
  struct tm    *time; 
  
  /* Wait 2 seconds before starting to adjust quickly */
  int waitForPcntIrqCount = pcntIrqCount + 2;

  while (pcntIrqCount != waitForPcntIrqCount)
  {
    /* Return if the button is released */
    if (GPIO_PinInGet(gpioPortC, 9) == 1) {
      timeIsFastForwarding = false;
      return;
    }

    /* Keep updating the second counter while waiting */
    if (updateDisplay)
    {
      time = localtime((time_t const *) &curTime);
      drawClock(time, true);
    }

    EMU_EnterEM2(false);
  }

  /* Keep incrementing the time while the button is held */
  while (GPIO_PinInGet(gpioPortC, 9) == 0)
  {
    if (i % 1000 == 0)
    {
      /* Increase time by 1 minute (60 seconds). */
      curTime += 60;

      time = localtime((time_t const *) &curTime);
      drawClock(time, true);
    }
    i++;
  }
  timeIsFastForwarding = false;
}

/**************************************************************************//**
 * @brief  Shows an analog clock on the display.
 *
 *****************************************************************************/
void analogClockShow(bool redraw)
{
  /* Convert time format */
  struct tm *time = localtime((time_t const *) &curTime);

  if (updateDisplay)
  {
    /* Draw clock face to frame buffer */
    analogClockUpdate(time, redraw);
    updateDisplay = false;
    if (timeIsFastForwarding)
    {
      fastForwardTime(analogClockUpdate);
    }
  }
}

/**************************************************************************//**
 * @brief  Updates the digital clock.
 *
 *****************************************************************************/
void digitalClockUpdate(struct tm *time, bool redraw)
{ 
  char clockString[16];
  
  if (redraw)
  {
    GLIB_setFont(&gc, (GLIB_Font_t *)&GLIB_FontNumber16x20);   
    gc.backgroundColor = White;
    gc.foregroundColor = Black;
    GLIB_clear(&gc);  
  }     
  
  sprintf(clockString, "%02d:%02d:%02d", time->tm_hour, time->tm_min, time->tm_sec);
  GLIB_drawString(&gc, clockString, strlen(clockString), 1, 52, true);

  /* Update display */
  DMD_updateDisplay();
}

/**************************************************************************//**
 * @brief  Shows an digital clock on the display.
 *
 *****************************************************************************/
void digitalClockShow(bool redraw)
{
  /* Convert time format */
  struct tm *time = localtime((time_t const *) &curTime);

  if (updateDisplay)
  {
    digitalClockUpdate(time, redraw);
    updateDisplay = false;
    if (timeIsFastForwarding)
    {
      fastForwardTime(digitalClockUpdate);
    }
  }
}


/**************************************************************************//**
 * @brief  Main function of clock example.
 *
 *****************************************************************************/
int main(void)
{  
  EMSTATUS status;
  bool redraw;
  ClockMode_t prevClockMode = CLOCK_MODE_DIGITAL;
  
  /* Chip errata */
  CHIP_Init();

  /* Use the 21 MHz band in order to decrease time spent awake.
     Note that 21 MHz is the highest HFRCO band on ZG. */
  CMU_ClockSelectSet( cmuClock_HF, cmuSelect_HFRCO  );
  CMU_HFRCOBandSet( cmuHFRCOBand_21MHz );

  /* Setup GPIO for pushbuttons. */
  gpioSetup();

  /* Initialize display module */    
  status = DISPLAY_Init();
  if (DISPLAY_EMSTATUS_OK != status)
    while (true)
      ;

  /* Initialize the DMD module for the DISPLAY device driver. */
  status = DMD_init(0);
  if (DMD_OK != status)
    while (true)
      ;

  status = GLIB_contextInit(&gc);
  if (GLIB_OK != status)
    while (true)
      ;
  
  /* Set PCNT to generate interrupt every second */
  pcntInit();
  
  /* Pre-compte positions for the analog graphics */
  analogClockInitGraphics();

  /* Enter infinite loop that switches between analog and digitcal clock
   * modes, toggled by pressing the button PB0. */
  while (true)
  {    
    redraw = (prevClockMode != clockMode);
    prevClockMode = clockMode;
    if (CLOCK_MODE_ANALOG == clockMode)
    {
      analogClockShow(redraw);
    }
    else
    {
      digitalClockShow(redraw);
    }
    /* Sleep between each frame update */
    EMU_EnterEM2(false);
  }
}
