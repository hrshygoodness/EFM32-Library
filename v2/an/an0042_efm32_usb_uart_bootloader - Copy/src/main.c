/**************************************************************************//**
 * @file main.c
 * @brief USB/USART0 bootloader.
 * @author Silicon Labs
 * @version 1.10
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
#include "em_usb.h"
#include "cdc.h"
#include "crc.h"
#include "flash.h"
#include "boot.h"
#include "autobaud.h"
#include "xmodem.h"
#include "bootldio.h"
#include "retargetdebug.h"

/*** Typedef's and defines. ***/

#define BULK_EP_SIZE    64      /* This is the max. ep size.              */

/** Version string, used when the user connects */
#define BOOTLOADER_VERSION_STRING "BOOTLOADER version 1.01, Chip ID "

#define USER_PAGE_START 0x0FE00000
#define USER_PAGE_END   0x0FE00200
#define LOCK_PAGE_START 0x0FE04000
#define LOCK_PAGE_END   0x0FE04200

#define DEBUG_LOCK_WORD (0x0FE04000 + (127 * 4))

/*** Function prototypes. ***/

static void commandlineLoop(  void );
static void verify(           uint32_t start, uint32_t end );
static void Disconnect(       int predelay, int postdelay );
static void StartRTC( void );

/*** The descriptors for a USB CDC device. ***/
#include "descriptors.h"

/*** Variables ***/

/*
 * This variable holds the computed CRC-16 of the bootloader and is used during
 * production testing to ensure the correct programming of the bootloader.
 * This can safely be omitted if you are rolling your own bootloader.
 * It is placed rigth after the interrupt vector table.
 */
#if defined ( __ICCARM__ )
#pragma location=0x200000dc
__no_init uint32_t bootloaderCRC;
#else
#error Undefined toolkit
#endif

/**************************************************************************//**
 * Strings.
 *****************************************************************************/
static const uint8_t crcString[]     = "\r\nCRC: ";
static const uint8_t newLineString[] = "\r\n";
static const uint8_t readyString[]   = "\r\nReady\r\n";
static const uint8_t okString[]      = "\r\nOK\r\n";
static const uint8_t failString[]    = "\r\nFail\r\n";
static const uint8_t unknownString[] = "\r\n?\r\n";

/**************************************************************************//**
 * The main entry point.
 *****************************************************************************/
int main(void)
{
  int msElapsed, i;

  /* Set new vector table pointer */
  SCB->VTOR = 0x20000000;

  /* Enable peripheral clocks. */
  CMU->HFPERCLKDIV = CMU_HFPERCLKDIV_HFPERCLKEN;
  CMU->HFPERCLKEN0 = CMU_HFPERCLKEN0_GPIO | BOOTLOADER_USART_CLOCK |
                     AUTOBAUD_TIMER_CLOCK ;

  /* Enable DMA interface */
  CMU->HFCORECLKEN0 = CMU_HFCORECLKEN0_DMA;

#if defined( BL_DEBUG )
  RETARGET_SerialInit();        /* Setup debug serialport etc. */
  USB_PUTS( "EFM32 USB/USART0 bootloader\r\n" );
#endif

  /* Calculate CRC16 for the bootloader itself and the Device Information page. */
  /* This is used for production testing and can safely be omitted in */
  /* your own code. */
  bootloaderCRC  = CRC_calc((void *) 0x0, (void *) BOOTLOADER_SIZE);
  bootloaderCRC |= CRC_calc((void *) 0x0FE081B2, (void *) 0x0FE08200) << 16;

  StartRTC();

#if !defined( SIMULATE_SWDCLK_PIN_HI )
  while ( SWDCLK_PIN_IS_LO() )
  {
    USB_PUTS( "SWDCLK is low\r\n" );

    if ( BOOT_checkFirmwareIsValid() )
    {
      USB_PUTS( "Booting application\r\n  " );
      BOOT_boot();
    }
    else
    {
      USB_PUTS( "No valid application, resetting EFM32... \r\n" );

      /* Go to EM2 and wait for RTC wakeup. */
      EMU_EnterEM2( false );
    }
  }
#endif

  NVIC_DisableIRQ( RTC_IRQn );

  /* Try to start HFXO. */

  CMU_OscillatorEnable( cmuOsc_HFXO, true, false );

  /* Wait approx. 1 second to see if HFXO starts. */
  i = 1500000;
  while ( i && !( CMU->STATUS & CMU_STATUS_HFXORDY ) )
  {
    i--;
  }

  USBTIMER_Init();

  if ( i == 0 )
  {
    CMU_HFRCOBandSet( cmuHFRCOBand_28MHz );
  }
  else
  {
    CMU_ClockSelectSet( cmuClock_HF, cmuSelect_HFXO );
    USBD_Init( &initstruct );       /* Start USB CDC functionality  */
  }

  AUTOBAUD_start();                 /* Start autobaud               */

  /* Wait 30 seconds for USART or USB connection */
  msElapsed = 0;
  while ( msElapsed < 30000 )
  {
    if ( AUTOBAUD_completed() )
      break;

    if ( CDC_Configured )
    {
      BOOTLDIO_setMode( CDC_Configured );
      break;
    }

    USBTIMER_DelayMs( 100 );
    msElapsed += 100;
  }
  AUTOBAUD_stop();

  if ( msElapsed >= 30000 )
  {
    USB_PUTS( "USART0/USB timeout, resetting EFM32...\r\n  " );
    Disconnect( 0, 2000 );
    SCB->AIRCR = 0x05FA0004;        /* Reset EFM32. */
  }

  /* Print a message to show that we are in bootloader mode */
  BOOTLDIO_printString("\r\n\r\n" BOOTLOADER_VERSION_STRING );

  /* Print the chip ID. This is useful for production tracking */
  BOOTLDIO_printHex(DEVINFO->UNIQUEH);
  BOOTLDIO_printHex(DEVINFO->UNIQUEL);
  BOOTLDIO_printString("\r\n");

  /* Figure out correct flash geometry. */
  FLASH_CalcPageSize();
  /* Initialize flash for writing */
  FLASH_init();

  /* Start executing command line */
  commandlineLoop();
}


/**************************************************************************//**
 * @brief
 *   The main command line loop. Placed in Ram so that it can still run after
 *   a destructive write operation.
 *   NOTE: __ramfunc is a IAR specific instruction to put code into RAM.
 *   This allows the bootloader to survive a destructive upload.
 *****************************************************************************/
static void commandlineLoop( void )
{
  uint8_t  c;

  /* The main command loop */
  while (1)
  {
    /* Retrieve new character */
    c = BOOTLDIO_rxByte();
    /* Echo */
    if (c != 0)
    {
      BOOTLDIO_txByte( c );
    }

    switch (c)
    {
    /* Bootloader version command */
    case 'i':
      /* Print version */
      BOOTLDIO_printString("\r\n\r\n" BOOTLOADER_VERSION_STRING );
      /* Print the chip ID */
      BOOTLDIO_printHex(DEVINFO->UNIQUEH);
      BOOTLDIO_printHex(DEVINFO->UNIQUEL);
      BOOTLDIO_printString("\r\n");
      break;

    /* Upload command */
    case 'u':
      BOOTLDIO_printString( readyString );
      XMODEM_download( BOOTLOADER_SIZE, flashSize );
      break;

    /* Destructive upload command */
    case 'd':
      BOOTLDIO_printString( readyString );
      XMODEM_download( 0, flashSize );
      break;

    /* Write to user page */
    case 't':
      BOOTLDIO_printString( readyString );
      XMODEM_download( USER_PAGE_START, USER_PAGE_END );
      break;

    /* Write to lock bits */
    case 'p':
      BOOTLDIO_printString( readyString );
      XMODEM_download( LOCK_PAGE_START, LOCK_PAGE_END );
      break;

    /* Boot into new program */
    case 'b':
      Disconnect( 5000, 2000 );
      BOOT_boot();
      break;

    /* Debug lock */
    case 'l':
#if defined( BL_DEBUG )
      /* We check if there is a debug session active in DHCSR. If there is we
       * abort the locking. This is because we wish to make sure that the debug
       * lock functionality works without a debugger attatched. */
      if ((CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) != 0x0)
      {
        USB_PUTS( "\r\n\r\n **** WARNING: DEBUG SESSION ACTIVE. NOT LOCKING!  **** \r\n\r\n" );
        BOOTLDIO_printString( "Debug active.\r\n" );
      }
      else
      {
        USB_PUTS( "Starting debug lock sequence.\r\n" );
#endif
        FLASH_writeWord( DEBUG_LOCK_WORD, 0x0 );
        if ( *(volatile uint32_t*)DEBUG_LOCK_WORD == 0x0 )
        {
          BOOTLDIO_printString( okString );
        }
        else
        {
          BOOTLDIO_printString( failString );
        }
#if defined( BL_DEBUG )
        USB_PRINTF( "Debug lock word: 0x%x \r\n", *((uint32_t *) DEBUG_LOCK_WORD) );
      }
#endif
      break;

    /* Verify content by calculating CRC of entire flash */
    case 'v':
      verify( 0, flashSize );
      break;

    /* Verify content by calculating CRC of application area */
    case 'c':
      verify( BOOTLOADER_SIZE, flashSize );
      break;

    /* Verify content by calculating CRC of user page.*/
    case 'n':
      verify( USER_PAGE_START, USER_PAGE_END );
      break;

    /* Verify content by calculating CRC of lock page */
    case 'm':
      verify( LOCK_PAGE_START, LOCK_PAGE_END );
      break;

    /* Reset command */
    case 'r':
      Disconnect( 5000, 2000 );

      /* Write to the Application Interrupt/Reset Command Register to reset
       * the EFM32. See section 9.3.7 in the reference manual. */
      SCB->AIRCR = 0x05FA0004;
      break;

    /* Unknown command */
    case 0:
      /* Timeout waiting for RX - avoid printing the unknown string. */
      break;

    default:
      BOOTLDIO_printString( unknownString );
    }
  }
}


/**************************************************************************//**
 * @brief
 *   Helper function to print flash write verification using CRC
 * @param start
 *   The start of the block to calculate CRC of.
 * @param end
 *   The end of the block. This byte is not included in the checksum.
 *****************************************************************************/
static void verify(uint32_t start, uint32_t end)
{
  BOOTLDIO_printString(crcString);
  BOOTLDIO_printHex(CRC_calc((void *) start, (void *) end));
  BOOTLDIO_printString(newLineString);
}


/**************************************************************************//**
 * Disconnect USB link with optional delays.
 *****************************************************************************/
static void Disconnect( int predelay, int postdelay )
{
  if ( BOOTLDIO_usbMode() )
  {
    if ( predelay )
    {
      /* Allow time to do a disconnect in a terminal program. */
      USBTIMER_DelayMs( predelay );
    }

    USBD_Disconnect();

    if ( postdelay )
    {
      /*
       * Stay disconnected long enough to let host OS tear down the
       * USB CDC driver.
       */
      USBTIMER_DelayMs( postdelay );
    }
  }
}

/**************************************************************************//**
 * @brief RTC IRQ Handler
 *****************************************************************************/
void RTC_IRQHandler( void )
{
  /* Clear interrupt flag */
  RTC->IFC = RTC_IFC_COMP1 | RTC_IFC_COMP0 | RTC_IFC_OF;
}

/**************************************************************************//**
 * Initialize and start RTC.
 *****************************************************************************/
static void StartRTC( void )
{
  /* Enable LE */
  CMU->HFCORECLKEN0 |= CMU_HFCORECLKEN0_LE;

  /* Enable LFRCO for RTC */
  CMU->OSCENCMD = CMU_OSCENCMD_LFRCOEN;
  /* Setup LFA to use LFRCRO */
  CMU->LFCLKSEL = CMU_LFCLKSEL_LFA_LFRCO;
  /* Enable RTC */
  CMU->LFACLKEN0 = CMU_LFACLKEN0_RTC;

  /* Clear interrupt flags */
  RTC->IFC = RTC_IFC_COMP1 | RTC_IFC_COMP0 | RTC_IFC_OF;
  /* 250 ms wakeup time */
  RTC->COMP0 = ( PIN_LOOP_INTERVAL * SystemLFRCOClockGet() ) / 1000;
  /* Enable Interrupts on COMP0 */
  RTC->IEN = RTC_IEN_COMP0;
  /* Enable RTC interrupts */
  NVIC_EnableIRQ(RTC_IRQn);
  /* Enable RTC */
  RTC->CTRL = RTC_CTRL_COMP0TOP | RTC_CTRL_DEBUGRUN | RTC_CTRL_EN;
}
