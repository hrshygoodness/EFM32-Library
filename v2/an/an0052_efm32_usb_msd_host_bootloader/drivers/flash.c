/**************************************************************************//**
 * @file flash.c
 * @brief Bootloader flash writing functions.
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
#include "config.h"
#include "flash.h"

/* DMA Control block. We only need 1 block for transfers. */
/* This control block needs to be aligned to 256 byte boundaries. */
#if defined (__ICCARM__)
#pragma data_alignment=256
DMA_DESCRIPTOR_TypeDef descr;
#elif defined (__CC_ARM)
DMA_DESCRIPTOR_TypeDef descr __attribute__ ((aligned(256)));
#elif defined (__GNUC__)
DMA_DESCRIPTOR_TypeDef descr __attribute__ ((aligned(256)));
#else
#error Undefined toolkit
#endif

volatile uint16_t crcTemp;
volatile uint32_t flashSize;
volatile uint16_t bufferSize;
volatile uint32_t flashPageSize;

/**************************************************************************//**
 * @brief
 *   This function calculates the CRC-16-CCIT checksum of a memory range.
 *  
 * @note
 *   This implementation uses an initial value of 0, while some implementations
 *   of CRC-16-CCIT uses an initial value of 0xFFFF. If you wish to
 *   precalculate the CRC before uploading the binary to the bootloader you 
 *   can use this function. However, keep in mind that the 'v' and 'c' commands
 *   computes the crc of the entire flash, so any bytes not used by your
 *   application will have the value 0xFF.
 *
 * @param start
 *   Pointer to the start of the memory block
 *
 * @param end
 *   Pointer to the end of the block. This byte is not included in the computed
 *   CRC.
 *
 * @return
 *   The computed CRC value.
 *****************************************************************************/
uint16_t CRC_calc(uint8_t *start, uint8_t *end)
{
  uint16_t crc = crcTemp;
  uint8_t  *data;

  for (data = start; data < end; data++)
  {
    crc  = (crc >> 8) | (crc << 8);
    crc ^= *data;
    crc ^= (crc & 0xff) >> 4;
    crc ^= crc << 12;
    crc ^= (crc & 0xff) << 5;
  }
  return crc;
}

/***************************************************************************//**
* @brief
*   Initializes the Flash programmer
*******************************************************************************/
void FLASH_init(void)
{
  /* Write MSC unlock code to enable interface */
  MSC->LOCK = MSC_UNLOCK_CODE;
  /* Enable memory controller */
  if ((flashSize >= MASSERASE_BLOCK_SIZE))
  {
    MSC->WRITECTRL = MSC_WRITECTRL_WREN + MSC_WRITECTRL_WDOUBLE;
  }
  else
  {
    MSC->WRITECTRL = MSC_WRITECTRL_WREN;
  }
  
  /* Enable DMA */
  DMA->CONFIG = DMA_CONFIG_EN;
  /* Setup the DMA control block. */
  DMA->CTRLBASE = (uint32_t) &descr;
}

/***************************************************************************//**
* @brief
*   Calculate flash page size
*******************************************************************************/
void FLASH_CalcPageSize(void)
{
  uint8_t family = *(uint8_t*)0x0FE081FE;

  flashSize = *(uint16_t*)0x0FE081F8 * 1024;

  bufferSize = 4096;                    /* Assume Giant, 'H' */
  flashPageSize = 4096;                 
  
  if ( family == 74 )
  {
    bufferSize = 2048;                  /* Leopard, 'J' */
    flashPageSize = 2048;               
  }
}

/**************************************************************************//**
 *
 * Erase a block of flash.
 *
 * @param blockStart is the start address of the flash block to be erased.
 *
 * This function will erase one blocks on the on-chip flash.  After erasing,
 * the block will be filled with 0xff bytes.  Read-only and execute-only
 * blocks can not be erased.
 *
 * This function will not return until the block has been erased.
 *****************************************************************************/
__ramfunc void FLASH_eraseOneBlock(uint32_t blockStart)
{
  uint32_t acc = 0xFFFFFFFF;
  uint32_t *ptr;

  /* Optimization - check if block is allready erased.
   * This will typically happen when the chip is new. */
  for (ptr = (uint32_t *) blockStart; ptr < (uint32_t *)(blockStart + flashPageSize); ptr++)
    acc &= *ptr;

  /* If the accumulator is unchanged, there is no need to do an erase. */
  if (acc == 0xFFFFFFFF)
    return;

  /* Load address */
  MSC->ADDRB    = blockStart;
  MSC->WRITECMD = MSC_WRITECMD_LADDRIM;

  /* Send Erase Page command */
  MSC->WRITECMD = MSC_WRITECMD_ERASEPAGE;

  /* Waiting for erase to complete */
  while ((MSC->STATUS & MSC_STATUS_BUSY)) ;
}

/**************************************************************************//**
 *
 * Mass erase a flash block.
 *
 * @param eraseCmd The mass erase command to write to MSC WRITECMD register.
 *
 * This function will mass erase a 512K block on a Giant device.
 * This function will not return until the block has been erased.
 *****************************************************************************/
__ramfunc void FLASH_massErase(uint32_t eraseCmd)
{
  /* Unlock mass erase */
  MSC->MASSLOCK = MSC_MASSLOCK_LOCKKEY_UNLOCK;

  /* Send Mass erase command */
  MSC->WRITECMD = eraseCmd;

  /* Waiting for erase to complete */
  while ((MSC->STATUS & MSC_STATUS_BUSY)){}

  /* Lock mass erase */
  MSC->MASSLOCK = MSC_MASSLOCK_LOCKKEY_LOCK;
}

/**************************************************************************//**
 * Erase flash.
 *
 * @param baseAddress
 *   The address to start erasing from
 *****************************************************************************/
void FLASH_Erase(uint32_t baseAddress)
{
  uint32_t addr = baseAddress;

  /* Figure out correct flash geometry. */
  FLASH_CalcPageSize();
  /* Initialize flash for writing */
  FLASH_init();

  /* Erase flash, do first 512K block. */
  /* Check if it is possible to mass erase first block. */
  if (( addr == 0 ) && (flashSize >= MASSERASE_BLOCK_SIZE))
  {
    FLASH_massErase(MSC_WRITECMD_ERASEMAIN0);
    addr += MASSERASE_BLOCK_SIZE;
  }
  else
  {
    while (addr < flashSize)
    {
      FLASH_eraseOneBlock(addr);
      addr += flashPageSize;
    }
  }

  /* Do second 512K block. */
  if (flashSize > MASSERASE_BLOCK_SIZE)
  {
    /* Mass erase possible ? */
    if ((addr == MASSERASE_BLOCK_SIZE) && (flashSize >= 2 * MASSERASE_BLOCK_SIZE))
    {
      FLASH_massErase( MSC_WRITECMD_ERASEMAIN1 );
    }
    else
    {
      while (addr < 2 * MASSERASE_BLOCK_SIZE)
      {
        FLASH_eraseOneBlock(addr);
        addr += flashPageSize;
      }
    }
  }
}

/**************************************************************************//**
 *
 * Program flash.
 *
 * @param block_start is a pointer to the base of the flash.
 * @param offset_into_block is the offset to start writing.
 * @param count is the number of bytes to be programmed. Must be a multiple of
 * four.
 * @param buffer is a pointer to a buffer holding the data.
 *
 * This function uses DMA channel 0 to program a buffer of words into
 * onboard flash. It will start the DMA transfer, but will not wait for
 * it's completion. If a DMA transfer is alreay in progress when this
 * function is called, the function will stall waiting for the previous
 * transfer to complete.
 *
 * This function will program a sequence of words into the on-chip flash.
 * Programming consists of ANDing the new data with the existing data; in
 * other words bits that contain 1 can remain 1 or be changed to 0, but bits
 * that are 0 can not be changed to 1.  Therefore, a word can be programmed
 * multiple times so long as these rules are followed; if a program operation
 * attempts to change a 0 bit to a 1 bit, that bit will not have its value
 * changed.
 *
 * Since the flash is programmed a word at a time, the starting address and
 * byte count must both be multiples of four.  It is up to the caller to
 * verify the programmed contents, if such verification is required.
 *****************************************************************************/
__ramfunc void FLASH_writeBlock(void *block_start,
                                uint32_t offset_into_block,
                                uint32_t count,
                                uint8_t const *buffer)
{
  /* Set up a basic memory to peripheral DMA transfer. */
  /* Load the start address into the MSC */
  MSC->ADDRB    = ((uint32_t) block_start) + offset_into_block;
  MSC->WRITECMD = MSC_WRITECMD_LADDRIM;

  /* Set the MSC as the destination. */
  descr.DSTEND = (void *)(&(MSC->WDATA));

  /* Set up the end pointer to copy from the buffer. */
  descr.SRCEND = (void *)(buffer + count - 4);

  /* Control information */
  descr.CTRL = DMA_CTRL_DST_INC_NONE       /* Do not increment destination */
               | DMA_CTRL_DST_SIZE_WORD    /* Transfer whole words at a time */
               | DMA_CTRL_SRC_INC_WORD     /* Write one word at the time */
               | DMA_CTRL_SRC_SIZE_WORD    /* Transfer whole words at a time */
               | DMA_CTRL_R_POWER_1
               | DMA_CTRL_CYCLE_CTRL_BASIC /* Basic transfer */
                                           /* Number of transfers minus two. */
                                           /* This field contains the number of transfers minus 1. */
                                           /* Because one word is transerred using WRITETRIG we need to */
                                           /* Substract one more. */
               | (((count / 4) - 2) << _DMA_CTRL_N_MINUS_1_SHIFT);

  /* Set channel to trigger on MSC ready for data */
  DMA->CH[0].CTRL = DMA_CH_CTRL_SOURCESEL_MSC
                    | DMA_CH_CTRL_SIGSEL_MSCWDATA;

  /* Load first word into the DMA */
  MSC->WDATA = *((uint32_t *)(buffer));

  /* Activate channel 0 */
  DMA->CHENS = DMA_CHENS_CH0ENS;

  /* Trigger the transfer */
  MSC->WRITECMD = MSC_WRITECMD_WRITETRIG;
  
  /* Wait DMA transfer and write to complete */
  while ((DMA->CHENS & DMA_CHENS_CH0ENS) || (MSC->STATUS & MSC_STATUS_BUSY))
    ;
}
