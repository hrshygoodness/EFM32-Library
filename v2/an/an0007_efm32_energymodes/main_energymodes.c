/******************************************************************************
 * @file main_energymodes.c
 * @brief Energy Modes Demo Application
 * @author Silicon Labs
 * @version 1.09
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

#include "em_device.h"
#include "em_chip.h"
#include "em_rtc.h"
#include "em_cmu.h"
#include "em_emu.h"


/* Defines */
/* When this define is present, the program will enter EN4 at the end. */
/* If it is not present, EM3 is entered. */
#define ENTER_EM4
/* Clock defines*/
#define LFXO_FREQUENCY         32768
#define RTC_TIMEOUT_S          5
#define RTC_COUNT_TO_WAKEUP    (LFXO_FREQUENCY * RTC_TIMEOUT_S)


/* Structs for modules used */
CMU_TypeDef     *cmu    = CMU;
RTC_TypeDef     *rtc    = RTC;
TIMER_TypeDef   *timer  = TIMER0;


/* Global flag to indicate rtc interrupt */
volatile uint8_t rtcInterrupt = 0;



/******************************************************************************
 * @brief  Start LFXO for RTC
 * Starts the LFXO and routes it to the RTC.
 *****************************************************************************/
void startLFXOForRTC(void)
{    
  /* Starting LFXO and waiting until it is stable */
  CMU_OscillatorEnable(cmuOsc_LFXO, true, true);

    /* Routing the LFXO clock to the RTC */
  CMU_ClockSelectSet(cmuClock_LFA,cmuSelect_LFXO);
  CMU_ClockEnable(cmuClock_RTC, true);

  /* Enabling clock to the interface of the low energy modules */
  CMU_ClockEnable(cmuClock_CORELE, true);
}



/******************************************************************************
 * @brief  Sets up the RTC
 *
 *****************************************************************************/
void setupRTCTimeout(void)
{
  /* Setting up RTC */
  RTC_CompareSet(0, RTC_COUNT_TO_WAKEUP);
  RTC_IntEnable(RTC_IFC_COMP0);
  NVIC_EnableIRQ(RTC_IRQn);
  RTC_Enable(true);
}


/******************************************************************************
 * @brief waits for rtc trigger
 *****************************************************************************/
void waitForRTC(void)
{
  /* Waiting for rtc interrupt */
  rtcInterrupt = 0;
  setupRTCTimeout();
  while (rtcInterrupt == 0) ;
}



/******************************************************************************
 * @brief enables all clocks and waits
 *****************************************************************************/
void enableAllClocks(void)
{
  /* Turning on all oscillators */
  /* Then waiting for all oscillators to stabilize */
  CMU_OscillatorEnable(cmuOsc_AUXHFRCO, true, true);
  CMU_OscillatorEnable(cmuOsc_HFXO, true, true);
  CMU_OscillatorEnable(cmuOsc_LFRCO, true, true);
  CMU_OscillatorEnable(cmuOsc_LFXO, true, true);

  /* Switching the CPU clock source to HFXO */
  /* This will increase current consumtion, since it runs on 32MHz */
  CMU_ClockSelectSet(cmuClock_HF,cmuSelect_HFXO);

  /* Enabling clocks to all core and peripheral modules */
  cmu->HFCORECLKEN0 = 0xFFFFFFFF;
  cmu->HFPERCLKEN0  = 0xFFFFFFFF;

  /* Wait */
  waitForRTC();
}



/******************************************************************************
 * @brief disables all clocks and waits
 *****************************************************************************/
void disableAllClocks(void)
{
  /* Switching the CPU clock source to HFRCO */
  CMU_ClockSelectSet(cmuClock_HF,cmuSelect_HFRCO);

  /* Disabling every oscillator except hfrco and lfxo */
  /* NOTE: MAKE SURE NOT TO DISABLE THE CURRENT CPU CLOCK!!*/
  CMU_OscillatorEnable(cmuOsc_AUXHFRCO, false, true);
  CMU_OscillatorEnable(cmuOsc_HFXO, false, true);
  CMU_OscillatorEnable(cmuOsc_LFRCO, false, true);

  /* Disabling all unused clocks. The LE clock must be on in order to use the
   * RTC */
  cmu->HFCORECLKEN0 = CMU_HFCORECLKEN0_LE;
  cmu->HFPERCLKEN0  = 0;

  /* Wait */
  waitForRTC();
}



/******************************************************************************
 * @brief changes band and prescales core clock
 *****************************************************************************/
void downScaleCoreClock(void)
{
  /* Changing the band of the HFRCO */
  CMU_HFRCOBandSet(cmuHFRCOBand_7MHz);

  /* Setting prescaling of the CPU clock*/
  CMU_ClockDivSet(cmuClock_CORE,cmuClkDiv_4);

  /* Wait */
  waitForRTC();
}



/******************************************************************************
 * @brief enters em1 and waits for timer interrupt
 *****************************************************************************/
void prepareEM1(void)
{
  /* Enabling clock to timer0 */
  CMU_ClockEnable(cmuClock_TIMER0, true);

  /* Scaling down the clock of the peripherals */
  /* Remember to enable the peripheral clock */
  CMU_ClockDivSet(cmuClock_HFPER,cmuClkDiv_512);

  /* Setup timer to give interrupt upon overflow */
  timer->IEN = TIMER_IEN_OF;
  timer->CMD = TIMER_CMD_START;
  NVIC_EnableIRQ(TIMER0_IRQn);
}



/******************************************************************************
 * @brief enters em2 and waits for rtc interrupt
 *****************************************************************************/
void prepareEM2(void)
{
  /* Enabling clock to timer0. Also DMA clock (see errata)*/
  CMU_ClockEnable(cmuClock_CORELE, true);
  CMU_ClockEnable(cmuClock_DMA, true);

  /* Setting up RTC to issue interrupt */
  setupRTCTimeout();
}



/******************************************************************************
 * @brief  Main function
 * Main is called from _program_start, see assembly startup file
 *****************************************************************************/
int main(void)
{
  /* Initialize chip */
  CHIP_Init();

  /* Initalizing */
  startLFXOForRTC();

  /* Starting to enter different energy-modes and  various clock settings */
  /************************************************************************/

  /* Turn on all clocks and oscillators. Then wait in EM0 */
  enableAllClocks();

  /* Turn off all clocks and wait in EM0 */
  /* Current consumption drops drastically */
  disableAllClocks();

  /* Adjusting core clock down to further reduce current consumption */
  downScaleCoreClock();

  /* Enter EM1. Using Timer0 to wake up */
  prepareEM1();
  EMU_EnterEM1();

  /* Only enable RTC and wait in EM2 */
  prepareEM2();
  EMU_EnterEM2(false);

#ifdef ENTER_EM4
  /* Enter EM4 */
  EMU_EnterEM4();
#else
  /* Enter EM3 */
  EMU_EnterEM3(false);
#endif
return 0;
}



/******************************************************************************
 * @brief RTC Interrupt Handler. Sets interrupt flag and disables the RTC.
 *        The interrupt table is in assembly startup file startup_efm32.s
 *****************************************************************************/
void RTC_IRQHandler(void)
{
  /* Clear interrupt source */
  RTC_IntClear(RTC_IFC_COMP0);
  /* Flushing instructions to make sure that the interrupt is not re-triggered*/
  /* This may be required when the peripheral clock is slower than the core */
  __DSB();

  /* Disabling interrupts from RTC */
  NVIC_DisableIRQ(RTC_IRQn);

  /* Disabling RTC */
  RTC_Enable(false);

  /* Asserting that an RTC interrupt has occured. */
  rtcInterrupt = 1;
}



/******************************************************************************
 * @brief TIMER0 Interrupt Handler. Clears interrupt flag.
 *        The interrupt table is in assembly startup file startup_efm32.s
 *****************************************************************************/
void TIMER0_IRQHandler(void)
{
  /* Clear interrupt source */
  timer->IFC = TIMER_IFC_OF;
  /* Flushing instructions to make sure that the interrupt is not re-triggered*/
  /* This may be required when the peripheral clock is slower than the core */
  __DSB();

  /* Stopping timer */
  timer->CMD = TIMER_CMD_STOP;

  /* Disabling interrupts from TIMER */
  NVIC_DisableIRQ(TIMER0_IRQn);
}
