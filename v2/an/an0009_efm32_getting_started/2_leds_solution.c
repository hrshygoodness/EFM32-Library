/******************************************************************************
 * @file 2_leds_solution.c
 * @brief Blinking LEDs with STK solution
 * @author Silicon Labs
 * @version 1.18
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
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_chip.h"

#if defined ( STK3200 )
  #define LED_PORT    gpioPortC
  #define LED_PIN     10
  #define PB0_PORT    gpioPortC
  #define PB0_PIN     8
#elif defined ( STK3700 )
  #define LED_PORT    gpioPortE
  #define LED_PIN     2
  #define PB0_PORT    gpioPortB
  #define PB0_PIN     9
#elif defined ( STK3300 )
  #define LED_PORT    gpioPortD
  #define LED_PIN     7
  #define PB0_PORT    gpioPortD
  #define PB0_PIN     8
#elif defined ( STKG8XX )
  #define LED_PORT    gpioPortC
  #define LED_PIN     0
  #define PB0_PORT    gpioPortB
  #define PB0_PIN     9
#else
  #error "undefined KIT"
#endif


/******************************************************************************
 * @brief Delay function
 *****************************************************************************/
void Delay(uint16_t milliseconds)
{
  /* Enable clock for TIMER0 */
  CMU->HFPERCLKEN0 |= CMU_HFPERCLKEN0_TIMER0;
  
  /* Set prescaler to maximum */
  TIMER0->CTRL = (TIMER0->CTRL & ~_TIMER_CTRL_PRESC_MASK) |  TIMER_CTRL_PRESC_DIV1024;
  
  /* Clear TIMER0 counter value */
  TIMER0->CNT = 0;
  
  /* Start TIMER0 */
  TIMER0->CMD = TIMER_CMD_START;
  
  /* Wait until counter value is over the threshold */
  while(TIMER0->CNT < 13*milliseconds){
   /* Do nothing, just wait */ 
  }
  
  /* Stop TIMER0 */
  TIMER0->CMD = TIMER_CMD_STOP;
}

/******************************************************************************
 * @brief  Main function
 * Main is called from _program_start, see assembly startup file
 *****************************************************************************/
int main(void)
{ 
  /* Initialize chip */
  CHIP_Init();  
  
  /* Initialize led_status */
  uint16_t led_status = 0;
 
  /* Enable clock for GPIO */
  CMU_ClockEnable(cmuClock_GPIO, true);
  
  /* Configure LED_PORT pin LED_PIN (User LED) as push/pull outputs */
  GPIO_PinModeSet(LED_PORT,         /* Port */
                  LED_PIN,          /* Pin */
                  gpioModePushPull, /* Mode */
                  0 );              /* Output value */
  
  /* Set PB0_PORT PB0_PIN (Push button 0) as input */
  GPIO_PinModeSet(PB0_PORT,         /* Port */
                  PB0_PIN,          /* Pin */
                  gpioModeInput,    /* Mode */
                  0 );              /* Output value */
  
  /* Wait in this loop until led_status reaches 15 */
  while(led_status < 2){
    
    /* Wait until button is toggled. NOTE: Value is 0 when pushed and vice versa */
    while(GPIO_PinInGet(PB0_PORT, PB0_PIN)); 
    while(!GPIO_PinInGet(PB0_PORT, PB0_PIN)); 
    
    /* Increase count */
    led_status++;
  
    /* Set LSB of count value on LED */
    GPIO_PortOutSetVal(LED_PORT, led_status<<LED_PIN, 1<<LED_PIN);
  }

  /* Run LED animation */
  while(1){
    led_status = 0; /* Reset LED status */
    while(led_status < 25){
      /* Turn LED off */  
      GPIO_PortOutSetVal(LED_PORT, 0x0, 1<<LED_PIN);
      Delay(led_status++); /* Wait */
      /* Turn LED on */ 
      GPIO_PortOutSetVal(LED_PORT, 1<<LED_PIN, 1<<LED_PIN); 
      Delay(25-led_status); /* Wait */
    }
    led_status = 0; /* Reset LED status */
    while(led_status < 25){
      /* Turn LED off */  
      GPIO_PortOutSetVal(LED_PORT, 0x0, 1<<LED_PIN);
      Delay(25-led_status); /* Wait */
      /* Turn LED on */ 
      GPIO_PortOutSetVal(LED_PORT, 1<<LED_PIN, 1<<LED_PIN); 
      Delay(led_status++); /* Wait */
    }
  }
}



