/******************************************************************************
 * @file swo_debug.c
 * @brief SWO output functions for printf over swo
 * @author Silicon Labs
 * @version 1.01
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 *
 ******************************************************************************/


#include "em_device.h"
#include "em_chip.h"

/* Need to implement the  two Retarget IO functions with the read/write functions we want to use. */
/* Use ITM for printf over SWO. */
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
