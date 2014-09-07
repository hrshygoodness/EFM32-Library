/**************************************************************************//**
 * @file
 * @brief Energy Modes Example
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
#include <stdint.h>
#include <stdbool.h>
#include "efm32.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_rtc.h"
#include "bsp.h"

#if defined(_EFM32_GIANT_FAMILY)

/* Defines for Push Button 0  */
#define PB0_PORT                    gpioPortB
#define PB0_PIN                     9


#else
/* Defines for Push Button 0  */
#define PB0_PORT                    gpioPortD
#define PB0_PIN                     8

#endif

/* Which Energy Mode should be entered? */
typedef enum { EM0, EM1, EM2, EM3, EM4 }   states_t;
states_t STATE = EM0;

/**************************************************************************//**
 * @brief GPIO Handler
 * Triggered by pressing Push Button 0
 *****************************************************************************/
void GPIO_IRQHandler(void)
{
  /* Clear interrupt source */
  GPIO_IntClear(1 << PB0_PIN);

  /* Update state */
  switch (STATE)
  {
  case EM0:
    STATE = EM1;
    break;
  case EM1:
    STATE = EM2;
    break;
  case EM2:
    STATE = EM3;
    break;
  case EM3:
    STATE = EM4;
    break;
  }
}

/**************************************************************************//**
 * @brief odd GPIO Handler
 * Triggered by pressing Push Button 0
 *****************************************************************************/
void GPIO_ODD_IRQHandler(void){
  
  GPIO_IRQHandler();
  
}
/**************************************************************************//**
 * @brief even GPIO Handler
 * Triggered by pressing Push Button 0
 *****************************************************************************/
void GPIO_EVEN_IRQHandler(void){
  
  GPIO_IRQHandler();
  
}

/**************************************************************************//**
 * @brief SWO Setup
 * Enables code view in energyAware Profiler
 *****************************************************************************/
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

  BSP_LedsInit();

  /* Enable clock for GPIO module */
  CMU_ClockEnable(cmuClock_GPIO, true);

  /* Configure interrupt for Push Button 0 */
  GPIO_PinModeSet(PB0_PORT, PB0_PIN, gpioModeInput, 1);
#if defined(_EFM32_GIANT_FAMILY)
  NVIC_EnableIRQ(GPIO_ODD_IRQn);
#else
  NVIC_EnableIRQ(GPIO_EVEN_IRQn);
#endif
  GPIO_IntConfig(PB0_PORT, PB0_PIN, false, true, true);

  /* By turning on retention in EM4, the state of the GPIO pins
   * can be retained in all energy modes */
  GPIO->CTRL = GPIO_CTRL_EM4RET;

  /* Uncomment to drive LED in all energy modes */
  /* BSP_LedSet(0);*/

  while (1)
  {
    switch (STATE)
    {
    case EM0:
      break;
    case EM1:
      EMU_EnterEM1();
      break;
    case EM2:
      EMU_EnterEM2(true);
      break;
    case EM3:
      EMU_EnterEM3(true);
    case EM4:
      EMU_EnterEM4();
      break;
    }
  }
}
