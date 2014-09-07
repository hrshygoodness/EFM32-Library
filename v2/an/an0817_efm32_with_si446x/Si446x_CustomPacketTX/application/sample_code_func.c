/*! @file sample_code_func.c
 * @brief This file contains functions to manage behavior of basic human module interfaces (push-buttons, switches, LEDs).
 *
 * @b COPYRIGHT
 * @n Silicon Laboratories Confidential
 * @n Copyright 2012 Silicon Laboratories, Inc.
 * @n http://www.silabs.com
 */

#include "bsp_def.h"



/*------------------------------------------------------------------------*/
/*                          Global variables                              */
/*------------------------------------------------------------------------*/

/*------------------------------------------------------------------------*/
/*                          Function implementations                      */
/*------------------------------------------------------------------------*/
#ifdef HMI_DRIVER_EXTENDED_SUPPORT
  #if ((defined SILABS_PLATFORM_COMPONENT_PB) && (defined SILABS_PLATFORM_COMPONENT_LED) && (defined SILABS_PLATFORM_COMPONENT_BUZZER))
    /*!
     * This function is used to show the actual state of the push-buttons on the Buzzer.
     *
     * @return  None.
     */
    void vSampleCode_ShowPbOnBuzzer(void)
    {
      uint8_t boPbPushTrack;
      uint16_t woPbPushTime;
      uint8_t bPbLedCnt;

      gHmi_PbIsPushed(&boPbPushTrack, &woPbPushTime);

      for (bPbLedCnt = 1; bPbLedCnt <= 4; bPbLedCnt++)
      {
        if (boPbPushTrack)
        {
          vHmi_ChangeBuzzState(eHmi_BuzzOnce_c);
        }
        else
        {
          vHmi_ChangeBuzzState(eHmi_BuzzOff_c);
        }
      }
    }
  #endif
#endif

#if ((defined SILABS_PLATFORM_COMPONENT_PB) && (defined SILABS_PLATFORM_COMPONENT_LED))
/*!
 * This function is used to send fix payload length packet.
 *
 * @return  TRUE - Started a packet TX / FALSE - No packet sent.
 */
uint8_t vSampleCode_SendFixPacket(void)
{
  uint8_t boPbPushTrack;
  uint16_t woPbPushTime;

  gHmi_PbIsPushed(&boPbPushTrack, &woPbPushTime);

  if (eHmi_PbNo_c == boPbPushTrack)
  {
    return FALSE;
  }

  vRadio_StartTx(pRadioConfiguration->Radio_ChannelNumber, (uint8_t *) &pRadioConfiguration->Radio_CustomPayload[0u]);

  /* Packet sending initialized */
  return TRUE;
}

#endif
