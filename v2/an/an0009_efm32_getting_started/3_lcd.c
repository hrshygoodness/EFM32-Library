/******************************************************************************
 * @file 3_lcd.c
 * @brief LCD Controller example
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
#include "em_lcd.h"
#include "segmentlcd.h"
#include "em_chip.h"



/******************************************************************************
 * @brief  Main function
 * Main is called from _program_start, see assembly startup file
 *****************************************************************************/
int main(void)
{  
  /* Initialize chip */
  CHIP_Init();  
  
  /* ENTER YOUR CODE HERE */
  
  /* Initialize LCD controller without boost*/
  
  /* Write something to text field on LCD display */
  
  /* Turn on Gecko symbol */
  
  /* Set up animation start data */
    
  /* Enable and configure animation feature */
  
  /* Stay in this loop forever at end of program */
  while (1) ;
}
