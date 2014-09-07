/**************************************************************************//**
 * @file lesense_lcsense_calibration.c
 * @brief LESENSE calibration routine
 * @author Silicon Labs
 * @version 1.07
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
#include "em_lesense.h"
#include "lesense_lcsense_calibration.h"

/***************************************************************************//**
 * @brief
 *   Calibrates an LC sensor on a given channel
 *
 * @details
 *   This function scans a channel until the buffer is full and
 *   uses the last result from the buffer to use as
 *   counter threshold
 *
 * @note
 *   This function assumes that there is no metal near the LC
 *   sensor, that way the number of pulses counted will
 *   be the maximum and can be used as threshold.
 *   If there is metal close the threshold will be lower and
 *   no trigger will occur.
 *   To make sure only the chosen channel is scanned all the
 *   other channels are disabled.
 *
 * @param[in] chIdx
 *   Identifier of the scan channel. Valid range: 0-15.
 *
 ******************************************************************************/
void lesenseCalibrateLC(uint8_t chIdx)
{
  uint8_t i;
    
  /* Enable scan and pin on selected channel */
  LESENSE_ChannelEnable(chIdx, true, true);
  
  /* Disable scan and pin on all other channels */
  for(i=0; i<LESENSE_MAX_CHANNELS; i++)
  {
    if(i!=chIdx)
        LESENSE_ChannelEnable(i, false, false);
  }
  
  /* Start scan. */
  LESENSE_ScanStart();
  
  /* Waiting for buffer to be full */
  while(!(LESENSE->STATUS & LESENSE_STATUS_BUFFULL));
  
  uint32_t calibValue = LESENSE_ScanResultDataBufferGet(BUFFER_INDEX_LAST);
  /*  Use last result as counter threshold */
  LESENSE_ChannelThresSet(chIdx, 0, calibValue);  
  
  /* Stop scan. */
  LESENSE_ScanStop();
  
  /* Clear result buffer */
  LESENSE_ResultBufferClear();
}
