/**************************************************************************//**
 * @file
 * @brief CMU configuration example for EFM32_STKs
 * @author Energy Micro AS
 * @version 1.06
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2010 Energy Micro AS, http://www.energymicro.com</b>
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
 * 4. The source and compiled code may only be used on Energy Micro "EFM32"
 *    microcontrollers and "EFR4" radios.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Energy Micro AS has no
 * obligation to support this Software. Energy Micro AS is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Energy Micro AS will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 *****************************************************************************/

#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_system.h"
#include "em_timer.h"

/* overflow of 1 second @ 14Mhz */
#define TOP    13671

#if defined ( STK3700 )
  #define LED_PORT gpioPortE
  #define LED_PIN  2
#elif defined ( STK3300 )
  #define LED_PORT gpioPortD
  #define LED_PIN  7
#elif defined ( STKG8XX )
  #define LED_PORT gpioPortC
  #define LED_PIN  0
#else
  #error "undefined KIT"
#endif


bool    prescChange = false;
bool    bandChange  = false;
bool    xtalChange  = false;
uint8_t count       = 0;

/**************************************************************************//**
 * @brief CMU_IRQHandler
 * Interrupt Service Routine CMU Interrupt Line
 *****************************************************************************/
void CMU_IRQHandler(void)
{
  /* Read Interrupt flags */
  uint32_t intFlags = CMU_IntGet();

  /* Clear interrupt flags register */
  CMU_IntClear(CMU_IF_HFXORDY | CMU_IF_HFRCORDY);

  /* If HFXORDY interrupt occured select it as clock source
   * for the HF clock branch and disable HFRCO (not in use anymore) */
  if (intFlags & CMU_IF_HFXORDY)
  {
    CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);
    CMU_OscillatorEnable(cmuOsc_HFRCO, false, false);
  }
  /* If HFRCORDY interrupt occured select it as clock source
   * for the HF clock branch and disable HFXO (not in use anymore) */
  else
  {
    CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFRCO);
    CMU_OscillatorEnable(cmuOsc_HFXO, false, false);
  }
}

/**************************************************************************//**
 * @brief TIMER0_IRQHandler
 * Interrupt Service Routine TIMER0 Interrupt Line
 *****************************************************************************/
void TIMER0_IRQHandler(void)
{
  /* Clear flag for TIMER0 overflow interrupt */
  TIMER_IntClear(TIMER0, TIMER_IF_OF);

  /* Toggle LED ON/OFF */
  GPIO_PinOutToggle(LED_PORT, LED_PIN);

  /* increment counter */
  count++;
}

/******************************************************************************
 * @brief  Main function
 * Main is called from _program_start, see assembly startup file
 *****************************************************************************/
int main(void)
{
  /* Initialize chip */
  CHIP_Init();

  /* Prescale the HFPERCLK -> HF/2 = 14/2 = 7Mhz */
  CMU_ClockDivSet(cmuClock_HFPER, cmuClkDiv_2);

  /* Enable clock for GPIO module */
  CMU_ClockEnable(cmuClock_GPIO, true);

  /* Enable clock for TIMER0 module */
  CMU_ClockEnable(cmuClock_TIMER0, true);

  /* Configure LED_PIN from LED_PORT as pushpull with output low */
  GPIO_PinModeSet(LED_PORT, LED_PIN, gpioModePushPull, 0);



  /* Select timer parameters */
  TIMER_Init_TypeDef timerInit =
  {
    .enable     = true,
    .debugRun   = true,
    .prescale   = timerPrescale1024,
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
  TIMER_IntEnable(TIMER0, TIMER_IF_OF);

  /* Set TIMER Top value */
  TIMER_TopSet(TIMER0, TOP);

  /* Configure TIMER */
  TIMER_Init(TIMER0, &timerInit);

  /* Enable TIMER0 interrupt vector in NVIC */
  NVIC_EnableIRQ(TIMER0_IRQn);

  /* Enable interrupts for HFXORDY and HFRCORDY */
  CMU_IntEnable(CMU_IF_HFXORDY | CMU_IF_HFRCORDY);

  /* Enable CMU interrupt vector in NVIC */
  NVIC_EnableIRQ(CMU_IRQn);

  while (1)
  {
    /* Go to EM1 */
    EMU_EnterEM1();

    /* If 5 togglings occured and prescaling
     * hans't changed */
    if (count > 5 && !prescChange)
    {
      /* Remove the prescaling for the HFPERCLK,
       * TIMER now running at 14Mhz */
      CMU_ClockDivSet(cmuClock_HFPER, cmuClkDiv_1);
      /* signal that prescaler has been changed
       * so it doesn't go inside the if clause again */
      prescChange = true;
    }
    /* If 10 togglings occured and banf
     * hans't changed */
    else if (count > 10 && !bandChange)
    {
      /* Change HFRCO band to 21Mhz */
      CMU_HFRCOBandSet(cmuHFRCOBand_21MHz);
      /* signal that band has been changed to 21Mhz
       * so it doesn't go inside the if clause again */
      bandChange = true;
    }
    /* If 15 togglings occured and crystal
     * hans't been enabled */
    else if (count > 15 && !xtalChange)
    {
      /* Enable HFXO without waiting for HFXORDY */
      CMU_OscillatorEnable(cmuOsc_HFXO, true, false);
      /* signal that the crystal (HFXO) has been enables
       * so it doesn't go inside the if clause again */
      xtalChange = true;
    }
    /* If 20 togglings go back to original HFRCO@14Mhz
     * with a 2 prescaler */
    else if (count > 20)
    {
      /* Enable HFRCO without waiting fo HFRCORDY */
      CMU_OscillatorEnable(cmuOsc_HFRCO, true, false);
      /* Set HFRCO band back to 14 Mhz*/
      CMU_HFRCOBandSet(cmuHFRCOBand_14MHz);
      /* Prescale the HFPERCLK -> HF/2 = 14/2 = 7Mhz */
      CMU_ClockDivSet(cmuClock_HFPER, cmuClkDiv_2);
      /* clear counter and flags */
      count       = 0;
      prescChange = false;
      bandChange  = false;
      xtalChange  = false;
    }
  }
}
