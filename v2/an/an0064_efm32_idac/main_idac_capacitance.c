 /******************************************************************************
 * @file main_idac_capacitance.c
 * @brief IDAC capacitance measurement example
 * @author Silicon Labs
 * @version 1.01
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

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include "em_device.h"
#include "em_idac.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_prs.h"
#include "em_timer.h"
#include "em_gpio.h"
#include "em_acmp.h"

#include "display.h"
#include "retargettextdisplay.h"
#include "textdisplay.h"

/* */
static volatile uint32_t capacitance = 0;

/* Range and step value for the IDAC Current, these values decide */
/* at what current the capacitator will be charged at.  */
const int8_t IdacRange = 3;
const int8_t IdacStep = 0x0f;  

/* IDAC current range start values in nA */
uint32_t IdacRangeStart[4] = {50, 1600, 500, 2000}; 

/* IDAC current step size in nA */
uint32_t IdacStepSize[4] = {50, 100, 500, 2000}; 

/* Flags that keep track of display updates */
static volatile bool updateDisplay = false;
static volatile bool timeout = false;
static volatile bool measurementComplete = false;


/* Use iprintf on GCC for smaller codesize */ 
#if !defined(__ICCARM__) && !defined (__CC_ARM) && !defined (__CROSSWORKS_ARM)
  #define printf iprintf
#endif

/**************************************************************************//**
 * @brief TIMER1_IRQHandler
 * Interrupt Service Routine for TIMER1 
 *****************************************************************************/
void TIMER1_IRQHandler(void)
{ 
  /* Clear flag for TIMER1 overflow interrupt */
  TIMER_IntClear(TIMER1, TIMER_IF_OF);
  
  /* Stop IDAC and TIMER*/
  IDAC_OutEnable(IDAC0, false);
  TIMER_Enable(TIMER1, false);  

   /* Drain excess voltage over the capacitator using the IDAC pin*/
  GPIO_PinModeSet(gpioPortB, 11, gpioModeWiredAnd, 0);  

  /* Set flag for display update */
  updateDisplay = false;
  timeout = true;
  
}

/**************************************************************************//**
 * @brief ACMP0 Interrupt handler
 * Interrupt Service Routine for ACMP0 when the capacitator is charged up
 * to the 
 *****************************************************************************/
void ACMP0_IRQHandler(void)
{
  /* Clear interrupt flag */
  ACMP0->IFC = ACMP_IFC_EDGE;  
  
  uint32_t current, timerCnt, timerFreq, timerPresc, timeMs;  

   /* Drain excess voltage over the capacitator using the IDAC pin*/
  GPIO_PinModeSet(gpioPortB, 11, gpioModeWiredAnd, 0);
  
  /* Stop IDAC and TIMER*/
  IDAC_OutEnable(IDAC0, false);
  TIMER_Enable(TIMER1, false);

  /* Fetch counter value and timer frequency and prescaler */
  timerCnt = TIMER_CounterGet(TIMER1);
  timerFreq = CMU_ClockFreqGet(cmuClock_TIMER1);
  timerPresc = 1 << ((TIMER1->CTRL & _TIMER_CTRL_PRESC_MASK) >> _TIMER_CTRL_PRESC_SHIFT);

  /* Reset cnt reg */
  TIMER_CounterSet(TIMER1, 0);  

  /* Calculate target_current in nA*/ 
  switch((IDAC_Range_TypeDef)IdacRange)
  {
  case idacCurrentRange0:
    current = IdacRangeStart[0] + IdacStep*IdacStepSize[0];
    break;
  case idacCurrentRange1:
    current = IdacRangeStart[1] + IdacStep*IdacStepSize[1];
    break;
  case idacCurrentRange2:
    current = IdacRangeStart[2] + IdacStep*IdacStepSize[2];
    break;
  case idacCurrentRange3:
    current = IdacRangeStart[3] + IdacStep*IdacStepSize[3];
    break; 
  default:
    current = 99999;
    break;
  }  
  
  /* Time = (timerCnt * timerPresc)/(timerFreq) */
  timeMs = (timerCnt * timerPresc)/(timerFreq/1000);
  
  /* C = i*t/V */
  capacitance = (current*timeMs)/(1250); /* nF */

  measurementComplete = true;
  updateDisplay = true;
}

/**************************************************************************//**
 * @brief GPIO_EVEN_IRQHandler
 * Interrupt Service Routine ven GPIO Interrupt Line
 *****************************************************************************/
void GPIO_EVEN_IRQHandler(void)
{ 
  /* clear flag for PC8 interrupt */
  GPIO_IntClear(1<<8);
  
  /* Disable PB11 input so the IDAC can control it*/
  GPIO_PinModeSet(gpioPortB, 11, gpioModeDisabled, 0);
  
  /* Start timer and IDAC on the same time */
  TIMER_Enable(TIMER1, true);
  
  /* Turns on the current */
  IDAC_OutEnable(IDAC0, true);
}

/**************************************************************************//**
 * @brief updateDisplay
 * Prints text to LCD
 *****************************************************************************/
void updateScreen(void)
{
  /* Print to display */

  if( timeout )
  {
    printf("\r   -TIMEOUT-" );
    timeout = false;
  }
  
  if( measurementComplete )
  {
    printf("\r   %6ld nF ", capacitance);
    measurementComplete = false;
  }

  updateDisplay = false;
}


/**************************************************************************//**
 * @brief setupIdac
 * Configures the IDAC
 *****************************************************************************/
void setupIdac()
{
  /* Enable IDAC clock */
  CMU_ClockEnable(cmuClock_IDAC0, true);
  
  IDAC_Init_TypeDef idacInit = 
  {
    .enable = true,
    .outMode = idacOutputPin,
    .prsEnable = false,
    .prsSel = idacPRSSELCh0,
    .sinkEnable = false,
  };
  
  IDAC_Init(IDAC0, &idacInit);
  
  IDAC_RangeSet(IDAC0, (IDAC_Range_TypeDef)IdacRange);
  IDAC_StepSet(IDAC0, IdacStep);
}

/***************************************************************************//**
* @brief setupTimer
*   Configure TIMER to upcount mode with interrupt on overflow
*******************************************************************************/
void setupTimer()
{
  /* Enable clock for TIMER1 module */
  CMU_ClockEnable(cmuClock_TIMER1, true);
  
  /* Select TIMER1 parameters */  
  TIMER_Init_TypeDef timerInit =
  {
    .enable     = false, 
    .debugRun   = true, 
    .prescale   = timerPrescale256, 
    .clkSel     = timerClkSelHFPerClk, 
    .fallAction = timerInputActionNone, 
    .riseAction = timerInputActionNone, 
    .mode       = timerModeUp, 
    .dmaClrAct  = false,
    .quadModeX4 = false, 
    .oneShot    = false, 
    .sync       = false, 
  };
  
  /* Enable overflow interrupt */
  TIMER_IntEnable(TIMER1, TIMER_IF_OF);
  
  /* Enable TIMER1 interrupt vector in NVIC */
  NVIC_EnableIRQ(TIMER1_IRQn);
  
  /* Configure TIMER */
  TIMER_Init(TIMER1, &timerInit);
  
  /* Select ACMP0 as source and ACOMPOUT as signal */
  PRS_SourceSignalSet(0, PRS_CH_CTRL_SOURCESEL_ACMP0, PRS_CH_CTRL_SIGSEL_ACMP0OUT, prsEdgeOff);
  
  /* TIMER CC use PRS as input on PRSCH0 */
  TIMER1->CC[0].CTRL |= TIMER_CC_CTRL_INSEL_PRS | TIMER_CC_CTRL_PRSSEL_PRSCH0;
  
  /* Configure TIMER CTRL to stop timer on rising edge */
  TIMER1->CTRL |= TIMER_CTRL_RISEA_STOP;
}

/**************************************************************************//**
 * @brief ACMPsetup
 * Configures the ACMP to interrupt on 1.25V
 *****************************************************************************/
void setupAcmp()
{
  /* Enable clocks required */
  CMU_ClockEnable(cmuClock_ACMP0, true);

  const ACMP_Init_TypeDef acmp_init =
  {
    false,                              /* Full bias */
    false,                              /* Half bias  */
    7,                                  /* Biasprog  configuration */
    false,                              /* Enable interrupt for falling edge */
    true,                               /* Enable interrupt for rising edge */
    acmpWarmTime256,                    /* Warm-up time in clock cycles, >140 cycles for 10us with 14MHz */
    acmpHysteresisLevel0,               /* Hysteresis configuration */
    0,                                  /* Inactive comparator output value */
    false,                              /* Enable low power mode */
    0,                                  /* Vdd reference scaling */
    false,                               /* Enable ACMP */
  };

  /* Init and set up ACMP0_CH1 (PC1)(exp header 5) */
  ACMP_Init(ACMP0, &acmp_init);
  ACMP_ChannelSet(ACMP0, acmpChannel1V25, acmpChannel1);
  
  /* Enable edge interrupt */
  ACMP_IntEnable(ACMP0, ACMP_IEN_EDGE);   
  
  ACMP_Enable(ACMP0);
  
  /* Wait for warmup */
  while (!(ACMP0->STATUS & ACMP_STATUS_ACMPACT)) ;

  /* Enable interrupts */
  NVIC_ClearPendingIRQ(ACMP0_IRQn);
  NVIC_EnableIRQ(ACMP0_IRQn);
}

/**************************************************************************//**
 * @brief setupGpio
 * Sets up interrupt on the push buttons
 *****************************************************************************/
void setupGpio()
{
  /* Enable clock for GPIO module */
  CMU_ClockEnable(cmuClock_GPIO, true);
  
  /* Configure PC8 as an input for PB0 button with filter enable (out = 1)*/
  GPIO_PinModeSet(gpioPortC, 8, gpioModeInput, 1);
  
  /* Enable GPIO_EVEN interrupt vector in NVIC */
  NVIC_EnableIRQ(GPIO_EVEN_IRQn);
  
  /* Configure PC8 interrupt on falling edge */
  GPIO_IntConfig(gpioPortC, 8, false, true, true);  
}

/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{  
  /* Chip errata */
  CHIP_Init();
  
  /* Enable clocks required */
  CMU_ClockEnable(cmuClock_HFPER, true);
  CMU_ClockEnable(cmuClock_PRS, true);
  
  /* Initialize IDAC */
  setupIdac();
  
  /* Initialize timer  */
  setupTimer();
  
  /* Drain excess voltage over the capacitator using the IDAC pin*/
  GPIO_PinModeSet(gpioPortB, 11, gpioModeWiredAnd, 0);
  
 /* Initialize the display module. */
  DISPLAY_Init();

  /* Retarget stdio to the display. */
  RETARGET_TextDisplayInit();

  /* Print initial text on display */
  printf( TEXTDISPLAY_ESC_SEQ_CURSOR_HOME_VT100 );
  printf( "\n");
  printf( "     EFM32      \n");
  printf( "   ZERO GECKO   \n" );
  printf( "\n" );
  printf( "\n" );
  printf( "     IDAC       \n" );
  printf( "  CAPACITANCE   \n" ); 
  printf( "  MEASUREMENT   \n" );
  printf( "     DEMO       \n" );
  printf( "\n" );
  printf( "\n" );
  printf( "   Press PB0\n");
  printf( "   to start\n" );
  printf( "\n\n" );

  
  /* Initialize ACOMP0_CH1 (PC1)  */
  setupAcmp();
  
  /* Initialize button interrupts */
  setupGpio();

  /* Stay in this loop forever at end of program */
  while (1)
  { 
    if ( updateDisplay )
    {
      updateScreen();
    }
    EMU_EnterEM1();    
  }
}
