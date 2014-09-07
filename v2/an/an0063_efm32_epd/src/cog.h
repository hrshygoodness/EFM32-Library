/*******************************************************************************
 * @file cog.h
 * @brief COG driver
 * @author Silicon Labs
 * @version 1.02
 *******************************************************************************
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
#ifndef _COG_H_
#define _COG_H_

#include <stdbool.h>

typedef struct _EPD_Config 
{
  /* Channel Select value */
  uint8_t   channelSelect[8]; 
  
  /* Voltage Level value */
  uint8_t   voltageLevel;  
  
  /* Horizontal size in pixels */
  uint16_t  horizontalSize;   
  
  /* Vertical size in pixels */
  uint16_t  verticalSize;     
  
  /* Size of one line in bytes. Includes pixels, scan bytes and dummy bytes */
  uint8_t   lineSize;
  
  /* Stage Time in ms */
  uint16_t  stageTime;
  
  /* Whether to include a 0x00 dummy byte at the end of a line. 
   * Needed for some displays */
  bool dummyByte;
  
  /* Whether to include a 0x00 border byte at the start of a line. 
   * Needed for some displays */
  bool borderByte;
       
} EPD_Config;


void sendFrame(uint8_t *frameBuffer);
void cogInit(void);
void cogPowerUp(void);
void cogPowerOff(void);
void updateStage(uint8_t *frameBuffer);

#endif
