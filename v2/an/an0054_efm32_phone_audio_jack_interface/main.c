/**************************************************************************//**
 * @file main.c
 * @brief Hijack demo for EFM32TG-STK3300
 * @version 1.05
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
#include <stdbool.h>
#include <stdio.h>

#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_emu.h"

/* Driver header file(s). */
#include "segmentlcd.h"

/* HiJack header file. */
#include "hijack.h"

/* Local prototypes */
void gpioSetup(void);
void setupSWO(void);
void Delay(uint32_t delayVal);


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
 * @brief GPIO Interrupt handler (PB11)
 *****************************************************************************/
void GPIO_ODD_IRQHandler(void)
{
  /* Acknowledge interrupt */
  GPIO_IntClear(1 << 11);
  HIJACK_ByteTx(0x55);
}


/**************************************************************************//**
 * @brief GPIO Interrupt handler (PD8)
 *****************************************************************************/
void GPIO_EVEN_IRQHandler(void)
{
  /* Acknowledge interrupt */
  GPIO_IntClear(1 << 8);
  HIJACK_ByteTx(0xAA);

}


/**************************************************************************//**
 * @brief Setup GPIO interrupt to change demo mode
 *****************************************************************************/
void gpioSetup(void)
{
  /* Enable GPIO in CMU */
  CMU_ClockEnable(cmuClock_GPIO, true);
  
  /* Configure PD8 and PB11 as input */
  GPIO_PinModeSet(gpioPortD, 8, gpioModeInput, 0);
  GPIO_PinModeSet(gpioPortB, 11, gpioModeInput, 0);

  /* Set falling edge interrupt for both ports */
  GPIO_IntConfig(gpioPortD, 8, false, true, true);
  GPIO_IntConfig(gpioPortB, 11, false, true, true);

  /* Enable interrupt in core for even and odd gpio interrupts */
  NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
  NVIC_EnableIRQ(GPIO_EVEN_IRQn);

  NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
  NVIC_EnableIRQ(GPIO_ODD_IRQn);
}
/**************************************************************************//**
 * @brief Delay loop function with optimization turned off.
 *****************************************************************************/
void Delay(uint32_t delayVal)
{
  /* Volatile keeps it from getting optimized away. */
  volatile uint32_t i;
  for(i = 0;i<delayVal;i++);
}
/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
  uint32_t j;
  
  /* Chip errata */
  CHIP_Init();

  /* Select clock source for HF clock. */
  CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);
  
  /* Prescale the core clock -> HF/4 = 32/4 = 8Mhz */
  CMU_ClockDivSet(cmuClock_CORE, cmuClkDiv_4);

  /* Configure push button interrupts. */
  gpioSetup();
  
  /* configure SWO output for debugging. */
  setupSWO();
  
  /* Init Segment LCD without boost. */
  SegmentLCD_Init(false);

  /* Turn on relevant symbols on the LCD. */
  SegmentLCD_Symbol(LCD_SYMBOL_GECKO, true);

  /* Print welcome text on the LCD. */
  SegmentLCD_Write("HiJack");

  /* Init the HiJack interface. */
  HIJACK_Init(NULL);
  
  /* While loop sending a range of values upon wakeup.  */
  while(1){    
    for(j = 0;j<254;j++){
      HIJACK_ByteTx(j);
      Delay(10000);
    }
    
  }   
}
