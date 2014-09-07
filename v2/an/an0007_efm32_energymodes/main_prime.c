/**************************************************************************//**
 * @file main_prime.c
 * @brief EM0 power consumption benchmark
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

#define PRIM_NUMS 64

void computePrimsForever(void)
{
  uint32_t i, d, n;
  uint32_t primes[PRIM_NUMS];

  /* Find prime numbers forever */
  while (1)
  {
    primes[0] = 1;
    for (i = 1; i < PRIM_NUMS;)
    {
      for (n = primes[i - 1] + 1; ;n++)
      {
        for (d = 2; d <= n; d++)
	      {
          if (n == d)
	        {
            primes[i] = n;
            goto nexti;
          }
          if (n%d == 0) break;
        }
      }
      nexti:
      i++;
    }
  }
}

/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{ 
  /* Initialize chip */
  CHIP_Init();

  /* Enable HFXO */
  CMU->OSCENCMD = CMU_OSCENCMD_HFXOEN;
  
  /* Wait for HFXO to stabilize */
  while( !(CMU->STATUS & CMU_STATUS_HFXORDY) );
  
  /* Switch HFCLK to HFXO */
  CMU->CMD = CMU_CMD_HFCLKSEL_HFXO;
  
  /* Disable HFRCO */
  CMU->OSCENCMD = CMU_OSCENCMD_HFRCODIS;
  
  /* Supress Conditional Branch Target Prefetch */
  MSC->READCTRL = MSC_READCTRL_MODE_WS1SCBTP;
  
  /* Disable all peripheral clocks */
  CMU->HFPERCLKEN0 = 0;
  CMU->HFCORECLKEN0 = 0;
  CMU->LFACLKEN0 = 0;
  CMU->LFBCLKEN0 = 0;
  
  /* Compute prime numbers forever */
  computePrimsForever();
  return 0;
}
