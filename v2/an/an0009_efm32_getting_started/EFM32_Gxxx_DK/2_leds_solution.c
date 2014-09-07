/******************************************************************************
 * @file 2_leds_solution.c
 * @brief LED blink example
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
#include "bsp.h"
#include "em_chip.h"

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

  /* Initialize development kit */
  BSP_Init(BSP_INIT_DEFAULT);
  
  /* Initialize LED status */
  uint32_t led_status = 0;
  
  /* Wait in this loop until led_staus reaches 15 */
  while(led_status < 15){
    /* Wait until joystick is pushed in any direction. */
    while(BSP_JoystickGet() == 0);
    while(BSP_JoystickGet() != 0);
  
    /* Increase count */
    led_status++;
  
    /* Put status on LEDs */
    BSP_LedsSet(led_status);
  }
  
  /* Run LED animation */
  while(1){
    led_status = 0; /* Reset LED status */
    while(led_status < 25){
      /* Turn leds off */  
      BSP_LedsSet(0x0000);
      Delay(led_status++); /* Wait */
      /* Turn leds on */ 
      BSP_LedsSet(0xFFFF);
      Delay(25-led_status); /* Wait */
    }
    led_status = 0; /* Reset LED status */
    while(led_status < 25){
      /* Turn leds off */  
      BSP_LedsSet(0x0000);
      Delay(25-led_status); /* Wait */
      /* Turn leds on */ 
      BSP_LedsSet(0xFFFF);
      Delay(led_status++); /* Wait */
    }
  }
}



