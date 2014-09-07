/*******************************************************************************
 * @file kits.c
 * @brief Sets up and handles kit-specific interface (buttons, leds)
 * @author Silicon Labs
 * @version 1.03
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
#include "em_device.h"
#include "em_gpio.h"
#include "bsp.h"
#include "state_machine.h"
#include "dap.h"
#include "kits.h"

#if defined(STK)
#include "segmentlcd.h"
#endif


extern bool execute;
extern uint32_t state;


#if !defined(STK)
/**********************************************************
 * Checks the state of buttons on the kit. 
 * will update the state or execute flag
 * if the corresponding button is pressed. 
 * Used when programmer is run on DK. 
 **********************************************************/
void checkButtons(void)
{
  /* Keep track of the button state */
  static uint32_t lastButtonState = 0;
  
  /* Retrieve the current state of push buttons */
  uint32_t buttons = BSP_PushButtonsGet();
  
  /* If unchanged, do nothing */
  if ( buttons == lastButtonState ) {
    return;
  }
  
  lastButtonState = buttons;
  
  if ( buttons == 1 ) 
  {
    /* Button 0 is pressed. Switch to next state */
    state = getNextState(state);
  } 
  else if ( buttons == 2 ) 
  {
    /* Button 1 is pressed. Execute current state */
    execute = true;
  }
}
#endif

/**********************************************************
 * Toggles a LED on on the kit to indicate whether a 
 * target is connected or not. 
 **********************************************************/
void setConnectedLed(bool on)
{
#if defined(STK)
  if ( on ) {
    GPIO_PinOutSet(gpioPortE, 3);
  } else {
    GPIO_PinOutClear(gpioPortE, 3);
  }
#else
  uint32_t leds = BSP_LedsGet();
  if ( on )
  {
    leds |= (1 << 0);
  } else {
    leds &= ~(1 << 0);
  }
  BSP_LedsSet(leds);
#endif
}

/**********************************************************
 * Toggles a LED on on the kit to indicate whether an 
 * operation is currently executing.
 **********************************************************/
void setBusyLed(bool on)
{
#if defined(STK)
  if ( on ) {
    GPIO_PinOutSet(gpioPortE, 2);
  } else {
    GPIO_PinOutClear(gpioPortE, 2);
  }
#else
  uint32_t leds = BSP_LedsGet();
  if ( on )
  {
    leds |= (1 << 1);
  } else {
    leds &= ~(1 << 1);
  }
  
  BSP_LedsSet(leds);
#endif
}

void initKit(void)
{
  /* Enable SWCLK pin */
  GPIO_PinModeSet((GPIO_Port_TypeDef)SWCLK_PORT, SWCLK_PIN, gpioModePushPull, 0);
  
  /* Enable SWDIO pin */
  GPIO_PinModeSet((GPIO_Port_TypeDef)SWDIO_PORT, SWDIO_PIN, gpioModePushPull, 1);
  
  /* Enable RESET pin */
  GPIO_PinModeSet((GPIO_Port_TypeDef)RESET_PORT, RESET_PIN, gpioModePushPull, 1);
  
  /* Enable SWO output */
  SWO_Setup();
  
#if defined(STK)  /* Using STK */
  GPIO_PinModeSet(gpioPortB, 9, gpioModeInput, 0);
  GPIO_IntConfig(gpioPortB, 9, false, true, true);
  NVIC_EnableIRQ(GPIO_ODD_IRQn);
  
  GPIO_PinModeSet(gpioPortB, 10, gpioModeInput, 0);
  GPIO_IntConfig(gpioPortB, 10, false, true, true);
  NVIC_EnableIRQ(GPIO_EVEN_IRQn);
  
  GPIO_PinModeSet(gpioPortE, 2, gpioModePushPull, 0);
  GPIO_PinModeSet(gpioPortE, 3, gpioModePushPull, 0);
    
  SegmentLCD_Init(false);    
  
#else  /* Using DK */
  
  BSP_Init(BSP_INIT_DK_SPI);
  
  BSP_LedsSet(0);
  
  
#endif
  
  
}

/**********************************************************
 * Initializes SWO output. 
 **********************************************************/
void SWO_Setup(void)
{
  uint32_t *dwt_ctrl = (uint32_t *) 0xE0001000;
  uint32_t *tpiu_prescaler = (uint32_t *) 0xE0040010;
  uint32_t *tpiu_protocol = (uint32_t *) 0xE00400F0;
  uint32_t *tpiu_ffcr = (uint32_t *) 0xE0040304;

  CMU->HFPERCLKEN0 |= CMU_HFPERCLKEN0_GPIO;

  /* Enable Serial wire output pin */
  GPIO->ROUTE |= GPIO_ROUTE_SWOPEN;

#if defined(_EFM32_GECKO_FAMILY) || defined(_EFM32_TINY_FAMILY)
  /* Set location 1 */
  GPIO->ROUTE = (GPIO->ROUTE & ~(_GPIO_ROUTE_SWLOCATION_MASK)) | GPIO_ROUTE_SWLOCATION_LOC1;
  /* Enable output on pin */
  GPIO->P[2].MODEH &= ~(_GPIO_P_MODEH_MODE15_MASK);
  GPIO->P[2].MODEH |= GPIO_P_MODEH_MODE15_PUSHPULL;
#elif defined(_EFM32_GIANT_FAMILY)
  /* Set location 0 */
  GPIO->ROUTE = (GPIO->ROUTE & ~(_GPIO_ROUTE_SWLOCATION_MASK)) | GPIO_ROUTE_SWLOCATION_LOC0;
  /* Enable output on pin */
  GPIO->P[5].MODEL &= ~(_GPIO_P_MODEL_MODE2_MASK);
  GPIO->P[5].MODEL |= GPIO_P_MODEL_MODE2_PUSHPULL;
#else
  #error Unknown device family!
#endif

  /* Enable debug clock AUXHFRCO */
  CMU->OSCENCMD = CMU_OSCENCMD_AUXHFRCOEN;

  while(!(CMU->STATUS & CMU_STATUS_AUXHFRCORDY));

  /* Enable trace in core debug */
  CoreDebug->DHCSR |= 1;
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

  /* Enable PC and IRQ sampling output */
  *dwt_ctrl = 0x400113FF;

  /* Set TPIU prescaler to 16 (14 MHz / 16 = 875 kHz SWO speed) */
  *tpiu_prescaler = 0xf;

  /* Set protocol to NRZ */
  *tpiu_protocol = 2;
  *tpiu_ffcr = 0x100;

  /* Unlock ITM and output data */
  ITM->LAR = 0xC5ACCE55;
  ITM->TCR = 0x10009;

  /* ITM Channel 0 is used for UART output */
  ITM->TER |= (1UL << 0);
  
  /* ITM Channel 1 is used for a custom debug output in this example. */
  ITM->TER |= (1UL << 1);  
}


/**********************************************************
 * Interrupt handler for push button PB0 (pin PB9). 
 * Changes state when the button is pushed.
 * Used when programmer is run on STK.
 **********************************************************/
void GPIO_ODD_IRQHandler(void)
{
  GPIO->IFC = GPIO->IF;
  
  /* Do not allow to change state while busy */
  if ( state != STATE_BUSY ) 
  {
    /* Go to the next state */
    state = getNextState(state);
  }
}


/**********************************************************
 * Interrupt handler for push button PB1 (pin PB10). 
 * Executes the current state when pushed.
 * Used when programmer is run on STK.
 **********************************************************/
void GPIO_EVEN_IRQHandler(void)
{
  GPIO->IFC = GPIO->IF;
  
  /* Do not allow to execute while in the busy state */
  if ( state != STATE_BUSY ) 
  {
    /* Set the execute flag. The state loop will acknowledge
     * this flag and start the selected task */
    execute = true;
    
    setBusyLed(true);
  }
}