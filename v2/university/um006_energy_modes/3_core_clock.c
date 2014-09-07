/**************************************************************************//**
 * @file
 * @brief Core Clock Example
 * @author Energy Micro AS
 * @version 1.0.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silicon Labs Software License Agreement. See 
 * "http://developer.silabs.com/legal/version/v11/Silicon_Labs_Software_License_Agreement.txt"  
 * for details. Before using this software for any purpose, you must agree to the 
 * terms of that agreement.
 *
 ******************************************************************************/
#include "efm32.h"
#include "em_chip.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "bsp.h"

/**************************************************************************//**
 * @brief Blink Led
 * This function uses a software delay to illustrate the difference between
 * running the CPU on 32 MHz and 64 Hz. Software delays are generally  not
 * a good idea!
 *****************************************************************************/
void blink_led(uint32_t delay, uint32_t rounds)
{
  uint32_t count   = 0;
  bool     enabled = false;

  while (1)
  {
    /* If count is a multiple of delay */
    if (!(count % delay))
    {
      BSP_LedToggle(0);
      if (enabled)
      {
        if (!(--rounds)) break;
      }

      count   = 0;
      enabled = !enabled;
    }
    count++;
  }
}

void setupSWO(void)
{
  uint32_t *dwt_ctrl = (uint32_t *) 0xE0001000;
  uint32_t *tpiu_prescaler = (uint32_t *) 0xE0040010;
  uint32_t *tpiu_protocol = (uint32_t *) 0xE00400F0;

  CMU->HFPERCLKEN0 |= CMU_HFPERCLKEN0_GPIO;
  /* Enable Serial wire output pin */
  GPIO->ROUTE |= GPIO_ROUTE_SWOPEN;
#if defined(_EFM32_GIANT_FAMILY)
  /* Set location 0 */
  GPIO->ROUTE = (GPIO->ROUTE & ~(_GPIO_ROUTE_SWLOCATION_MASK)) | GPIO_ROUTE_SWLOCATION_LOC0;

  /* Enable output on pin - GPIO Port F, Pin 2 */
  GPIO->P[5].MODEL &= ~(_GPIO_P_MODEL_MODE2_MASK);
  GPIO->P[5].MODEL |= GPIO_P_MODEL_MODE2_PUSHPULL;
#else
  /* Set location 1 */
  GPIO->ROUTE = (GPIO->ROUTE & ~(_GPIO_ROUTE_SWLOCATION_MASK)) | GPIO_ROUTE_SWLOCATION_LOC1;
  /* Enable output on pin */
  GPIO->P[2].MODEH &= ~(_GPIO_P_MODEH_MODE15_MASK);
  GPIO->P[2].MODEH |= GPIO_P_MODEH_MODE15_PUSHPULL;
#endif
  /* Enable debug clock AUXHFRCO */
  CMU->OSCENCMD = CMU_OSCENCMD_AUXHFRCOEN;

  while(!(CMU->STATUS & CMU_STATUS_AUXHFRCORDY));

  /* Enable trace in core debug */
  CoreDebug->DHCSR |= 1;
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

  /* Enable PC and IRQ sampling output */
  *dwt_ctrl = 0x400113FF;
  /* Set TPIU prescaler to 16. */
  *tpiu_prescaler = 0xf;
  /* Set protocol to NRZ */
  *tpiu_protocol = 2;
  /* Unlock ITM and output data */
  ITM->LAR = 0xC5ACCE55;
  ITM->TCR = 0x10009;
}


/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
  /* Initialize chip */
  CHIP_Init();
  
  setupSWO();

  /* Enable HFXO */
  CMU_OscillatorEnable(cmuOsc_HFXO, true, true);

  /* Switch HFCLK to HFXO */
  CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);

  /* Turn off HFRCO */
  CMU_OscillatorEnable(cmuOsc_HFRCO, false, false);

  BSP_LedsInit();

  while (1)
  {
    /* Blink led three times, with a factor 1 million delay */
    blink_led(1000000, 3);

    /* Turn on LFRCO */
    CMU_OscillatorEnable(cmuOsc_LFRCO, true, true);

    /* Select LFRCO */
    CMU_ClockSelectSet(cmuClock_HF, cmuSelect_LFRCO);

    /* Maximum prescaling for smalles frequency (64 Hz) */
    CMU_ClockDivSet(cmuClock_CORE, cmuClkDiv_512);

    /* Turn off HFXO */
    CMU_OscillatorEnable(cmuOsc_HFXO, false, false);

    /* Blink led three times, with a factor 1 delay */
    blink_led(1, 3);

    /* Turn on HFXO */
    CMU_OscillatorEnable(cmuOsc_HFXO, true, true);

    /* Select HFXO */
    CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);

    /* No prescaling for maximum clock frequency (32 MHz) */
    CMU_ClockDivSet(cmuClock_CORE, cmuClkDiv_1);

    /* Turn off LFRCO */
    CMU_OscillatorEnable(cmuOsc_LFRCO, false, false);
  }
}
