/*****************************************************************************
 * @file main.c
 * @The examples requires a FAT32 formatted micro-SD card
 * @details
 *   On some DVK main boards, you need to remove the prototype board for this
 *   example to run successfully.
 * @author Silicon Labs
 * @version 1.08
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
#include "em_emu.h"
#include "em_timer.h"
#include "ff.h"
#include "microsd.h"
#include "diskio.h"
#include "bsp.h"

/* Ram buffers
 * BUFFERSIZE should be between 512 and 1024, depending on available ram on efm32
 */
#define BUFFERSIZE      1024
/* Filename to open/write/read from SD-card */
#define TEST_FILENAME    "test.txt"

FIL fsrc;				/* File objects */
FATFS Fatfs;				/* File system specific */
FRESULT res;				/* FatFs function common result code */
UINT br, bw;				/* File read/write count */
DSTATUS resCard;			/* SDcard status */
int8_t ramBufferWrite[BUFFERSIZE];	/* Temporary buffer for write file */
int8_t ramBufferRead[BUFFERSIZE];	/* Temporary buffer for read file */
int8_t StringBuffer[] = "EFM32 ...the world's most energy friendly microcontrollers !";

/* Counts 1ms timeTicks */
volatile uint32_t msTicks;

/***************************************************************************//**
 * @brief
 *   This function is required by the FAT file system in order to provide
 *   timestamps for created files. Since this example does not include a 
 *   reliable clock we hardcode a value here.
 *
 *   Refer to drivers/fatfs/doc/en/fattime.html for the format of this DWORD.
 * @return
 *    A DWORD containing the current time and date as a packed datastructure.
 ******************************************************************************/
DWORD get_fattime(void)
{
  return (28 << 25) | (2 << 21) | (1 << 16);
}

/**************************************************************************//**
 * @brief SysTick_Handler
 * Interrupt Service Routine for system tick counter.
 *****************************************************************************/
void SysTick_Handler(void)
{
  /* Increment counter necessary in Delay()*/
  msTicks++;
}

/**************************************************************************//**
 * @brief Delays number of msTick Systicks (typically 1 ms)
 * @param dlyTicks Number of ticks to delay
 *****************************************************************************/
void Delay(uint32_t dlyTicks)
{
  uint32_t curTicks;

  curTicks = msTicks;
  while ((msTicks - curTicks) < dlyTicks) ;
}

/**************************************************************************//**
 * @brief
 *   Main function.
 * @details
 *   Configures the DVK for SPI
 *****************************************************************************/
int main(void)
{
int16_t i;
int16_t filecounter;

  /* Use 32MHZ HFXO as core clock frequency*/
  CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);

  /* Initialize DVK board register access */
  BSP_Init(BSP_INIT_DEFAULT);

  /* Setup SysTick Timer for 10 msec interrupts  */
  if (SysTick_Config(CMU_ClockFreqGet(cmuClock_CORE) / 100))
  {
    while (1) ;
  }

  /* Enable SPI access to MicroSD card */
  BSP_RegisterWrite(BC_SPI_CFG, 1);
  BSP_PeripheralAccess(BSP_SPI, true);
  
  /*Step1*/
  /*Initialization file buffer write */
  filecounter = sizeof(StringBuffer);

  for(i = 0; i < filecounter ; i++)
  {
     ramBufferWrite[i] = StringBuffer[i];
  }
  
  /*Step2*/
  /*Detect micro-SD*/
  while(1)
  {
    MICROSD_Init();                     /*Initialize MicroSD driver */
    
    resCard = disk_initialize(0);       /*Check micro-SD card status */
    
    switch(resCard)
    {
    case STA_NOINIT:                    /* Drive not initialized */
      break;
    case STA_NODISK:                    /* No medium in the drive */
      break;
    case STA_PROTECT:                   /* Write protected */
      break;
    default:
      break;
    }
   
    if (!resCard) break;                /* Drive initialized. */
 
    Delay(1);    
  }

  /*Step3*/
  /* Initialize filesystem */
  if (f_mount(0, &Fatfs) != FR_OK)
  {
    /* Error.No micro-SD with FAT32 is present */
    while(1);
  }
   
  /*Step4*/
  /* Open  the file for write */
   res = f_open(&fsrc, TEST_FILENAME,  FA_WRITE); 
   if (res != FR_OK)
   {
     /*  If file does not exist create it*/ 
     res = f_open(&fsrc, TEST_FILENAME, FA_CREATE_ALWAYS | FA_WRITE ); 
      if (res != FR_OK) 
     {
      /* Error. Cannot create the file */
      while(1);
    }
   }
  
  /*Step5*/
  /*Set the file write pointer to first location */ 
  res = f_lseek(&fsrc, 0);
   if (res != FR_OK) 
  {
    /* Error. Cannot set the file write pointer */
    while(1);
  }

  /*Step6*/
  /*Write a buffer to file*/
   res = f_write(&fsrc, ramBufferWrite, filecounter, &bw);
   if ((res != FR_OK) || (filecounter != bw)) 
  {
    /* Error. Cannot write the file */
    while(1);
  }
  
  /*Step7*/
  /* Close the file */
  f_close(&fsrc);
  if (res != FR_OK) 
  {
    /* Error. Cannot close the file */
    while(1);
  }
  
  /*Step8*/
  /* Open the file for read */
  res = f_open(&fsrc, TEST_FILENAME,  FA_READ); 
   if (res != FR_OK) 
  {
    /* Error. Cannot create the file */
    while(1);
  }

   /*Step9*/
   /*Set the file read pointer to first location */ 
   res = f_lseek(&fsrc, 0);
   if (res != FR_OK) 
  {
    /* Error. Cannot set the file pointer */
    while(1);
  }
  
  /*Step10*/
  /* Read some data from file */
  res = f_read(&fsrc, ramBufferRead, filecounter, &br);
   if ((res != FR_OK) || (filecounter != br)) 
  {
    /* Error. Cannot read the file */
    while(1);
  }
	
  /*Step11*/
  /* Close the file */
  f_close(&fsrc);
  if (res != FR_OK) 
  {
    /* Error. Cannot close the file */
    while(1);
  }
  
  /*Step12*/
  /*Compare ramBufferWrite and ramBufferRead */
  for(i = 0; i < filecounter ; i++)
  {
    if ((ramBufferWrite[i]) != (ramBufferRead[i]))
    {
      /* Error compare buffers*/
      while(1);
    }
  }

  /*Set here a breakpoint*/
  /*If the breakpoint is trap here then write and read functions were passed */
  while (1)
  {
  }
}
