/**************************************************************************//**
 * @file xmodem.c
 * @brief XMODEM protocol
 * @author Silicon Labs
 * @version 1.03
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
#include <stdbool.h>
#include "em_device.h"
#include "em_msc.h"

#include "xmodem.h"
#include "crc.h"
#include "uart.h"
#include "flash.h"
#include "aes.h"
#include "bootloader_config.h"

#define ALIGNMENT(base,align) (((base)+((align)-1))&(~((align)-1)))

/* Packet storage. Double buffered to allow DMA to write to flash
 * while receiving the next packet. These buffers will be accessed
 * with both 8 and 32 bit alignment */
#if defined (__ICCARM__)
#pragma data_alignment=4
uint8_t rawPacket[2][ALIGNMENT(sizeof(XMODEM_packet),4)];
#pragma data_alignment=4
uint8_t rawBuffer[2][ALIGNMENT(XMODEM_DATA_SIZE,4)];
#elif defined (__CC_ARM)
uint8_t rawPacket[2][ALIGNMENT(sizeof(XMODEM_packet),4)] __attribute__ ((aligned(4)));
uint8_t rawBuffer[2][ALIGNMENT(XMODEM_DATA_SIZE,4)] __attribute__ ((aligned(4)));
#elif defined (__GNUC__)
uint8_t rawPacket[2][ALIGNMENT(sizeof(XMODEM_packet),4)] __attribute__ ((aligned(4)));
uint8_t rawBuffer[2][ALIGNMENT(XMODEM_DATA_SIZE,4)] __attribute__ ((aligned(4)));
#else
#error Undefined toolkit, need to define alignment
#endif



/**************************************************************************//**
 * @brief Verifies checksum, packet numbering and
 * @param pkt The packet to verify
 * @param sequenceNumber The current sequence number.
 * @returns -1 on packet error, 0 otherwise
 *****************************************************************************/
RAMFUNC int XMODEM_verifyPacketChecksum(XMODEM_packet *pkt, uint32_t sequenceNumber)
{
  uint16_t packetCRC;
  uint16_t calculatedCRC;

  /* Check the packet number integrity */
  if (pkt->packetNumber + pkt->packetNumberC != 255)
  {
    return -1;
  }

  /* Check that the packet number matches the excpected number */
  if (pkt->packetNumber != (sequenceNumber % 256))
  {
    return -1;
  }

  calculatedCRC = CRC_calc((uint8_t *) pkt->data, (uint8_t *) &(pkt->crcHigh));
  packetCRC     = pkt->crcHigh << 8 | pkt->crcLow;

  /* Check the CRC value */
  if (calculatedCRC != packetCRC)
  {
    return -1;
  }
  return 0;
}


/**************************************************************************//**
 * @brief Starts a XMODEM download.
 *****************************************************************************/
RAMFUNC bool XMODEM_download(void)
{
  XMODEM_packet *pkt;
  uint32_t      i;
  uint32_t      byte;
  uint32_t      sequenceNumber = 1;
  uint32_t writeAddress, endAddress;
  
  uint8_t *decryptedBuffer;
  
  if ( USE_TEMP_STORAGE ) {
    writeAddress = TEMP_START_ADDRESS;
    endAddress = TEMP_END_ADDRESS;
  } else {
    writeAddress = FIRMWARE_START_ADDRESS;
    endAddress = FIRMWARE_END_ADDRESS;
  }
  
  /* Initialize AES for decryption */
  startDecryptCBC256();
  
  /* Send one start transmission packet. Wait for a response. If there is no
   * response, we resend the start transmission packet. */
  while (1)
  {
    BLUART_send(XMODEM_NCG);
    
    i = 10000000;
    
    /* Wait until we receive a packet */
    while ( !(UART->STATUS & USART_STATUS_RXDATAV) ) {
      
      /* Time out */
      if ( --i == 0 )
        break;
    }
    
    /* We have received a packet */
    if ( UART->STATUS & USART_STATUS_RXDATAV ) {
      break;
    }
  }
    
  /* Start receiving XMODEM packets */
  while (1)
  {
    
    /* Swap buffer for packet buffer */
    pkt = (XMODEM_packet *) rawPacket[sequenceNumber & 1];
    decryptedBuffer = rawBuffer[sequenceNumber & 1];

    /* Fetch the first byte of the packet explicitly, as it defines the
     * rest of the packet */
    pkt->header = BLUART_receive();
    

    /* Check for end of transfer */
    if (pkt->header == XMODEM_EOT)
    {
      /* Acknowledge End of transfer */
      BLUART_send(XMODEM_ACK);
      break;
    }
    
    /* If header is cancel message, then cancel the transfer */
    if (pkt->header == XMODEM_CAN)
    {
      return false;
    }

    /* If the header is not a start of header (SOH), then cancel *
     * the transfer. */
    if (pkt->header != XMODEM_SOH)
    {
      BLUART_send(XMODEM_CAN);
      return false;
    }
    
    /* If the received file is larger than the allocated memory 
     * for the firmware, cancel the transfer. */
    if ( writeAddress > endAddress ) {
      BLUART_send(XMODEM_CAN);
      return false;
    }

    /* Fill the remaining bytes packet */
    /* Byte 0 is padding, byte 1 is header */
    for (byte = 2; byte < sizeof(XMODEM_packet); byte++)
    {
      *((uint8_t *)pkt + byte) = BLUART_receive();
    }

    /* Verify that the packet is valid */
    if (XMODEM_verifyPacketChecksum(pkt, sequenceNumber) != 0)
    {
      /* On a malformed packet, send a NAK and start over */
      BLUART_send(XMODEM_NAK);
      continue;
    }
    
    /* If we have reached a new page, first erase it */
    if ( writeAddress % FLASH_PAGE_SIZE == 0 ) {
      FLASH_erasePage(writeAddress);
    }
    
    /* Decrypt one XMODEM packet(32 words) */
    for ( i=0; i<XMODEM_DATA_SIZE/AES_BLOCKSIZE; i++ ) {
      
      /* Decrypt one AES block (4 words) */
      decryptBlockCBC256(&(pkt->data[i * AES_BLOCKSIZE]), &(decryptedBuffer[i * AES_BLOCKSIZE]));
    }
    
    /* Check if this is the header packet */
    if ( sequenceNumber == 1 ) {
      
      /* Get the header */
      FirmwareHeader *header = (FirmwareHeader *)decryptedBuffer;
      
      /* Set the state to not verified initially */
      header->verified = FW_NOT_VERIFIED;
      
      /* Write the header to flash */
      FLASH_write((uint32_t *)writeAddress, (uint32_t *)header, sizeof(FirmwareHeader) / 4);
      
      /* Start at the boot address for the next packet */
      writeAddress = writeAddress + FIRMWARE_HEADER_SIZE; 
    } else {
       
      /* Write decrypted packet to flash */
      FLASH_write((uint32_t *)writeAddress, (uint32_t *)decryptedBuffer, XMODEM_DATA_SIZE / 4);
      writeAddress += XMODEM_DATA_SIZE;
    }
    
    sequenceNumber++;
    
    /* Send ACK */
    BLUART_send(XMODEM_ACK);
  }
  
  endDecryptCBC256();
  
  /* Wait for the last DMA transfer to finish. */
  while (DMA->CHENS & DMA_CHENS_CH0ENS) ;
  
  /* Return success */
  return true;
}


