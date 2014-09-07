/**************************************************************************//**
 * @file boot.c
 * @brief Bootloader boot functions
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

#include "em_device.h"
#include "em_usb.h"
#include "boot.h"
#include "config.h"
#include "usart.h"

extern uint32_t flashSize;
extern uint32_t baseAddress;

/**************************************************************************//**
 * @brief Checks to see if the reset vector of the application is valid
 * @return false if the firmware is not valid, true if it is.
 *****************************************************************************/
bool BOOT_checkFirmwareIsValid(void)
{
  uint32_t pc;

  pc = *((uint32_t *)BOOTLOADER_SIZE + 1);

  if (pc < MAX_SIZE_OF_FLASH)
    return true;

  return false;
}

/**************************************************************************//**
 * @brief This function sets up the Cortex M-3 with a new SP and PC.
 *****************************************************************************/
#if defined ( __CC_ARM   )
__asm void BOOT_jump(uint32_t sp, uint32_t pc)
{
  /* Set new MSP, PSP based on SP (r0)*/
  msr msp, r0
  msr psp, r0

  /* Jump to PC (r1)*/
  bx r1
}
#else
void BOOT_jump(uint32_t sp, uint32_t pc)
{
  (void) sp;
  (void) pc;
  /* Set new MSP, PSP based on SP (r0)*/
  __asm("msr msp, r0");
  __asm("msr psp, r0");

  /* Jump to PC (r1)*/
  __asm("mov pc, r1");
}
#endif

/**************************************************************************//**
 * @brief Boots the application
 *****************************************************************************/
void BOOT_boot(void)
{
  uint32_t pc, sp;

  /* Clear all interrupts set. */
  NVIC->ICER[0] = 0xFFFFFFFF;
  NVIC->ICER[1] = 0xFFFFFFFF;

  /* Disable USB */
  USBH_Stop();
  USB->ROUTE = _USB_ROUTE_RESETVALUE;

  /* Reset to 14MHz HFRCO */
  CMU->OSCENCMD = CMU_OSCENCMD_HFRCOEN;  
  /* Wait for the HFRCO to stabilize */
  while (!(CMU->STATUS & CMU_STATUS_HFRCORDY))
    ;
  /* Switch to HFRCO */
  CMU->CMD = CMU_CMD_HFCLKSEL_HFRCO;
  /* Disable HFXO */  
  CMU->OSCENCMD = CMU_OSCENCMD_HFXODIS;

  /* Reset memory system controller settings. */
  MSC->READCTRL  = _MSC_READCTRL_RESETVALUE;
  MSC->LOCK = 0;
  MSC->WRITECTRL = _MSC_WRITECTRL_RESETVALUE;

  /* Reset GPIO settings. */
  GPIO->P[5].MODEL = _GPIO_P_MODEL_RESETVALUE;
  GPIO->P[5].MODEH = _GPIO_P_MODEH_RESETVALUE;
  GPIO->P[5].DOUT  = _GPIO_P_DOUT_RESETVALUE;

  /* Reset DMA controller settings. */
  DMA->CONFIG     = _DMA_CONFIG_RESETVALUE;
  DMA->CTRLBASE   = _DMA_CTRLBASE_RESETVALUE;
  DMA->CH[0].CTRL = _DMA_CH_CTRL_RESETVALUE;
  DMA->CHENC      = 0xFFFFFFFF;

#ifndef NOUART
  /* Reset USART */
  USART_USED->CLKDIV = _USART_CLKDIV_RESETVALUE;
  USART_USED->ROUTE = _USART_ROUTE_RESETVALUE;
  USART_USED->CMD = 0x0c0a;
  GPIO->P[USART_PORTNUM].MODEL = _GPIO_P_MODEL_RESETVALUE;
  GPIO->P[USART_PORTNUM].MODEH = _GPIO_P_MODEH_RESETVALUE;
  GPIO->P[USART_PORTNUM].DOUT  = _GPIO_P_DOUT_RESETVALUE;
#endif
  
  /* Reset clocks */
  CMU->CTRL &= ~_CMU_CTRL_HFLE_MASK;
  CMU->HFCORECLKDIV = _CMU_HFCORECLKDIV_RESETVALUE;
  CMU->HFCORECLKEN0 = _CMU_HFCORECLKEN0_RESETVALUE;
  CMU->HFPERCLKEN0 = _CMU_HFPERCLKEN0_RESETVALUE;

  /* Set new vector table */
  SCB->VTOR = (uint32_t) BOOTLOADER_SIZE;
  /* Read new SP and PC from vector table */
  sp = *((uint32_t *) BOOTLOADER_SIZE);
  pc = *((uint32_t *) BOOTLOADER_SIZE + 1);

  BOOT_jump(sp, pc);
}
