/**************************************************************************//**
 * @file debug_printf.c
 * @brief Prints hello world through the ITM module on SWO-pin.
 * @author Silicon Labs
 * @version 1.04
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

#include <stdio.h>
#include "em_device.h"

#define ITM_Port32(n) (*((volatile unsigned int *)(0xE0000000+4*n)))

/* Need to implement the  two Retarget IO functions with the read/write functions we want to use. */
int RETARGET_WriteChar(char c){
  return ITM_SendChar (c);
}

int RETARGET_ReadChar(void){
  return 0;
}

/**************************************************************************//**
 * @brief Configure SWO - serial wire output
 *****************************************************************************/
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

/**************************************************************************//**
 * @brief  Main function
 *         The main function demonstrates two ITM trace features;
 *         First printf is utilized to print strings to trace channel 0.
 *         Then a while loop demonstrates how to send data to another 
 *         ITM channel, specifically channel 1.
 *****************************************************************************/
int main(void)
{ 
  int i;
  
  /* Configures the SWO to output both printf-information, PC-samples and interrupt trace. */
  SWO_Setup();
  
  printf("hello world\n");
  
  /* If the energyAware Commander command line readswo feature us used, printing ENDSWO will abort the eAcommander session. */
  printf("ENDSWO");

  /* This while loop increments i and sends the variable to ITM trace channel 1, notice that printf used ITM channel 0. */
  while(1){
    /* First we need to wait until the ITM buffer is ready to accept new data. */
    while (ITM_Port32(0) == 0);
    /* Use the define at the top of this file to write to the correct ITM-port address. */
    ITM_Port32(1) = i++;
  }
}
