/**************************************************************************//**
 * @file flash.c
 * @brief Flash erasing and writing functions for Speex record
 * @author Silicon Labs
 * @version 1.06
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
#include "em_emu.h"
#include "em_msc.h"

/**************************************************************************//**
 * @brief MSC Interrupt handler
 *****************************************************************************/
void MSC_IRQHandler(void)
{
  MSC_IntClear(MSC_IFC_ERASE);
}
    
/***************************************************************************//**
* @brief
*   Initializes write for Flash > 512kB (read-while-write)
*******************************************************************************/
void flashInit512(void)
{
  /* Write MSC unlock code to enable interface */
  MSC->LOCK = MSC_UNLOCK_CODE;
  /* Enable AIDIS for flash write/erase, PREFETCH for encode */
  MSC->READCTRL |= MSC_READCTRL_AIDIS + MSC_READCTRL_PREFETCH;
  /* Enable memory controller with read-while-write */
  MSC->WRITECTRL = MSC_WRITECTRL_RWWEN + MSC_WRITECTRL_WREN + MSC_WRITECTRL_WDOUBLE;
}

/***************************************************************************//**
 * @brief
 *   Disables the flash controller for writing.
 ******************************************************************************/
void flashDeinit(void)
{
  /* Disable AIDIS and PREFETCH */
  MSC->READCTRL &= ~(MSC_READCTRL_AIDIS + MSC_READCTRL_PREFETCH);
  /* Disable writing to the MSC */
  MSC->WRITECTRL = 0;
  /* Lock the MSC */
  MSC->LOCK = 0;
}

/**************************************************************************//**
 * Mass erase a flash block.
 * This function will mass erase a 512K block on a Giant device.
 * This function will not return until the block has been erased.
 *
 * @param eraseCmd The mass erase command to write to MSC WRITECMD register.
 *                 Only ERASEMAIN1 can use in this application note.
 *                 Otherwise the code on lower 512 KB flash will be erased.     
 *                 Make sure there is no valid code or data on upper 512KB.
 *****************************************************************************/
void flashMassErase(uint32_t eraseCmd)
{
  /* Unlock mass erase */
  MSC->MASSLOCK = MSC_MASSLOCK_LOCKKEY_UNLOCK;

  /* Enable flash erase done interrupt */
  MSC_IntEnable(MSC_IF_ERASE);
  NVIC_ClearPendingIRQ(MSC_IRQn);
  NVIC_EnableIRQ(MSC_IRQn);
  
  /* Send Mass erase command */
  MSC->WRITECMD = eraseCmd;

  /* Waiting for erase to complete in EM1 */
  while((MSC->STATUS & MSC_STATUS_BUSY))
  {
    EMU_EnterEM1();
  }
  NVIC_DisableIRQ(MSC_IRQn);
  
  /* Lock mass erase */
  MSC->MASSLOCK = MSC_MASSLOCK_LOCKKEY_LOCK;
}
