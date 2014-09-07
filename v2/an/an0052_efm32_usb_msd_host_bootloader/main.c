/**************************************************************************//**
 * @file main.c
 * @brief USB MSD Host Bootloader
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
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_usb.h"

#include "boot.h"
#include "config.h"
#include "ff.h"
#include "flash.h"
#include "msdh.h"
#include "usbconfig.h"
#include "usart.h"

/* USB related data */
STATIC_UBUF(tmpBuf, 1024);

/* File system */
FIL binFile;
FATFS Fatfs;
FRESULT res;
uint16_t byteRead;

/* Buffer for file read */
uint8_t fileReadBuffer[BUFFERSIZE];

extern uint16_t crcTemp;
extern uint32_t flashSize;
extern uint16_t bufferSize;
extern uint32_t flashPageSize;

/**************************************************************************//**
 * @brief Upgrade firmware from binary file in USB flash drive
 * @return 0: Error, 1: OK 
 *****************************************************************************/
static int16_t FirmwareUpgrade(void)
{
  uint32_t i;
  uint32_t fileSize;
  
  /* Initialize filesystem */
  res = f_mount( 0, &Fatfs );

  if (res != FR_OK)
  {
    printf("FAT-mount failed: %d\n", res);
    return 0;
  }
  else
  {
    printf("FAT-mount successful\n");
  }

  /* Open text file */
  res = f_open(&binFile, TEXT_FILENAME, FA_OPEN_EXISTING | FA_READ);
  if (res)
  {
    printf("\nNo text file: %d\n", res);
    goto defaultBinCheck;
  }

  /* Read text file */
  res = f_read(&binFile, fileReadBuffer, 20, &byteRead);
  if (res)
  {
    printf("\nText file read error: %d\n", res);
    f_close(&binFile);
    goto defaultBinCheck;
  }

  /* Close text file */
  f_close(&binFile);

  /* Open user binary file */
  res = f_open(&binFile, (char const *)fileReadBuffer, FA_OPEN_EXISTING | FA_READ);
defaultBinCheck:
  if (res)
  {
    /* Try default binary file if error on user binary file */
    res = f_open(&binFile, BIN_FILENAME, FA_OPEN_EXISTING | FA_READ);
    if (res)
    {
      printf("\nNo binary file: %d\n", res);
      return 0;
    }
    printf("\nUse default binary file\n");
  }
  
  /* Erase flash for program */
  __disable_irq();
  FLASH_Erase(BOOTLOADER_SIZE);
  __enable_irq();

  i = 0;
  crcTemp = 0;
  fileSize = 0;
  
  while (USBH_DeviceConnected())
  {
    res = f_read(&binFile, fileReadBuffer, bufferSize, &byteRead);
    if (res || byteRead == 0)
    {
      /* error or eof */
      break;
    }
      
    for (;;)
    {
      if (byteRead%8)
      {
        fileReadBuffer[byteRead] = 0xff;
        byteRead++;
      }
      else
      {
        break;
      }
    }
    
    /* Calculate binary file CRC */
    fileSize += byteRead;
    crcTemp = CRC_calc(fileReadBuffer, (fileReadBuffer + byteRead));
      
    __disable_irq();
    FLASH_writeBlock((void *)BOOTLOADER_SIZE, (i * bufferSize), byteRead, (uint8_t const *)(fileReadBuffer));
    __enable_irq();
      
    i++;
  }
  
  /* Calculate flash CRC */
  i = crcTemp;
  crcTemp = 0;
  crcTemp = CRC_calc((void *)BOOTLOADER_SIZE, (void *)(BOOTLOADER_SIZE + fileSize));
  
  /* Close binary file */
  f_close(&binFile);

  /* UNMOUNT drive */
  f_mount( 0, NULL );
  
  if (!fileSize || res || byteRead)
  {
    /* Error during flash write */
    printf("\nFlash write error!\n");
    return 0;
  }
  else
  {
    /* Check CRC */
    if (i == crcTemp)
    {
      printf("\nCRC:%04X\n", crcTemp);
      return 1;   
    }
    else
    {
      printf("\nCRC verify error!\n");
      return 0;   
    }
  }
}

/**************************************************************************//**
 * @brief main - the entrypoint after reset.
 *****************************************************************************/
int main(void)
{
  int connectionResult;
  USBH_Init_TypeDef is = USBH_INIT_DEFAULT;

  /* Enable HFXO and wait it stable */  
  CMU->OSCENCMD = CMU_OSCENCMD_HFXOEN;
  while (!(CMU->STATUS & CMU_STATUS_HFXORDY))
    ;
  /* Set 2 wait states */
  MSC->READCTRL = (MSC->READCTRL & ~_MSC_READCTRL_MODE_MASK) | MSC_READCTRL_MODE_WS2;
  /* Switch to selected oscillator */
  CMU->CMD = CMU_CMD_HFCLKSEL_HFXO;
  /* Disable HFRCO */  
  CMU->OSCENCMD = CMU_OSCENCMD_HFRCODIS;
  /* Update CMSIS core clock variable */
  SystemCoreClock = 48000000;
  /* Enable DMA interface */
  CMU->HFCORECLKEN0 = CMU_HFCORECLKEN0_DMA;

  /* Enable GPIO clock */
  CMU->HFPERCLKEN0 = CMU_HFPERCLKEN0_GPIO;

#ifndef NOUART
  /* Initialize USART */
  CMU->HFPERCLKEN0 |= USART_CLK;
  /* Set baudrate 115200 */
  USART_USED->CLKDIV = USART_BAUD;
  /* Enable TX pin and set location */
  USART_USED->ROUTE = USART_ROUTE_TXPEN | USART_LOCATION;
  /* Clear RX/TX buffers, enable TX */
  USART_USED->CMD = 0x0c04;
  /* To avoid false start, configure TX output as high */
  GPIO_PinModeSet(USART_TXPORT, USART_TXPIN, gpioModePushPull, 1);
#endif
  
  /* Initialize USB HOST stack */
  USBH_Init( &is );             
  printf("\nUSB MSD Host Bootloader V1.00");

  for (;;)
  {
    /* Wait for device connection */
    printf("\nWaiting for USB MSD device plug-in...\n");

    if (BOOT_checkFirmwareIsValid())
    {
      /* Set timeout if have valid applciation */
      connectionResult = USBH_WaitForDeviceConnectionB(tmpBuf, USB_WAIT_TIMEOUT);
    }
    else
    {
      /* Wait forever if no valid application */
      connectionResult = USBH_WaitForDeviceConnectionB(tmpBuf, 0);
    }      

    if (connectionResult == USB_STATUS_OK)
    {
      printf("\nA device was attached\n");

      if (MSDH_Init(tmpBuf, sizeof(tmpBuf)))
      {
        /* Execute firmware upgrade */
        if (FirmwareUpgrade())
        {
          printf("\nUpgrade Success!\n");
          /* start application */
          BOOT_boot();
        }
        else
        {
          printf("\nUpgrade Fail!\n");
          if (BOOT_checkFirmwareIsValid())
          {
            /* Try to boot old application, maybe just no binary file found */
            BOOT_boot();
          }
          else
          {
            printf( "\nPlease remove device.\n" );
          }
        }
      }
      else
      {
        printf( "\nPlease remove device.\n" );
      }
    }

    else if (connectionResult == USB_STATUS_DEVICE_MALFUNCTION)
    {
      printf("\nA malfunctioning device was attached, please remove device\n");
    }
    
    else if (connectionResult == USB_STATUS_TIMEOUT)
    {
      BOOT_boot();
    }

    while (USBH_DeviceConnected())
    {
      ;
    }
    printf("\n\nDevice removal detected...");
    USBH_Stop();
  }  
}
