/*! @file hmi.c
 * @brief This file contains functions to manage behavior of basic human module interfaces (push-buttons, switches, LEDs).
 *
 * @b COPYRIGHT
 * @n Silicon Laboratories Confidential
 * @n Copyright 2012 Silicon Laboratories, Inc.
 * @n http://www.silabs.com
 */

#include "bsp_def.h"


#define wTmr_8HzCntMax_c    125
#define wTmr_4HzCntMax_c    250
#define wTmr_2HzCntMax_c    500
#define wTmr_1HzCntMax_c    1000
#define wTmr_0Hz5CntMax_c   2000
#define wTmr_0Hz25CntMax_c  4000

/*------------------------------------------------------------------------*/
/*                          Global variables                              */
/*------------------------------------------------------------------------*/

#ifdef SILABS_PLATFORM_COMPONENT_PB
tHmi_PbData rHmi_PbData = { eHmi_PbStandBy_c, 0, 0, 0, 0, 0, 0};
eHmi_PbStates aqHmi_PbStates[] = {eHmi_PbNo_c, eHmi_Pb1_c, eHmi_Pb2_c}; /**< Array of instances of push-button definitions */
#endif

#ifdef SILABS_PLATFORM_COMPONENT_LED
tHmi_LedData arHmi_LedData[bHmi_NumOfLeds_c];// = {{eHmi_LedOff_c,},};  /**< Array of instances of Led data structure */

  #ifdef HMI_DRIVER_EXTENDED_SUPPORT
  uint16_t wHmi_LedMainCnt = 0; /**< Main time counter of Led handler */
  #endif
eHmi_Leds aqHmi_Leds[] = {eHmi_NoLed_c, eHmi_Led1_c, eHmi_Led2_c, eHmi_Led3_c, eHmi_Led4_c}; /**< Array of instances of Led definitions */
#endif

#ifdef SILABS_PLATFORM_COMPONENT_BUZZER
tHmi_BuzzData rHmi_BuzzData;/**< Instance of Buzzer data structure */
  #ifdef HMI_DRIVER_EXTENDED_SUPPORT
  uint16_t wHmi_BuzzMainCnt = 0;     /**< Main time counter of Buzzer handler */
  #endif
#endif

#ifdef SILABS_PLATFORM_COMPONENT_SWITCH
uint8_t bHmi_SwStateHolder = 0; /**< Holds the actual state of the switches */
#endif


/*------------------------------------------------------------------------*/
/*                          Function implementations                      */
/*------------------------------------------------------------------------*/

#ifdef SILABS_PLATFORM_COMPONENT_LED
/*!
 * This function is used to initialize the Led handler.
 *
 * @return  None.
 *
 * @note It has to be called from the initialization section.
 */
void vHmi_InitLedHandler(void)
{
  /*! NOTE: Re-initialization of LED Handler supported by the extended HMI driver */

#ifdef HMI_DRIVER_EXTENDED_SUPPORT
  uint8_t bLedCnt;

  for (bLedCnt = 1; bLedCnt <= bHmi_NumOfLeds_c; bLedCnt++)
  {
    arHmi_LedData[bLedCnt-1].gLedIsOn = FALSE;
    arHmi_LedData[bLedCnt-1].qLedState = eHmi_LedOff_c;
    arHmi_LedData[bLedCnt-1].wLedBlinkCnt = 0;
  }
#endif
}

/*!
 * This function is used to change state of selected Led.
 *
 * @param[in] qiLed Led to change its state.
 * @param[in] qiLedState New state of qiLed.
 *
 * @return  None.
 */
void vHmi_ChangeLedState(eHmi_Leds qiLed, eHmi_LedStates qiLedState)
{
  arHmi_LedData[qiLed-1].qLedState = qiLedState;
}

#ifdef HMI_DRIVER_EXTENDED_SUPPORT
/*!
 * This function is used to change state of all Leds.
 *
 * @param[in] qiLedState New state of all the Leds.
 *
 * @return  None.
 */
void vHmi_ChangeAllLedState(eHmi_LedStates qiLedState)
{
  uint8_t bLedCnt;

  for (bLedCnt = 1; bLedCnt <= bHmi_NumOfLeds_c; bLedCnt++)
  {
    arHmi_LedData[bLedCnt-1].qLedState = qiLedState;
  }
}

/*!
 * This function is used to force all Leds to off immediately.
 *
 * @return  None.
 */
void vHmi_ClearAllLeds(void)
{
  uint8_t bLedCnt;

  for (bLedCnt = 1; bLedCnt <= bHmi_NumOfLeds_c; bLedCnt++)
  {
    BSP_LedClear(bLedCnt-1);
    arHmi_LedData[bLedCnt-1].gLedIsOn = FALSE;
    arHmi_LedData[bLedCnt-1].qLedState = eHmi_LedStdBy_c;
  }
}
#endif

/*!
 * This function is used to handle Led management.
 *
 * @return  None.
 */
void vHmi_LedHandler(void)
{
#ifdef HMI_DRIVER_EXTENDED_SUPPORT
  uint8_t bLedChngReq = FALSE;
#else
  static
#endif
  uint8_t bLedCnt = 0u;

#ifdef HMI_DRIVER_EXTENDED_SUPPORT
  wHmi_LedMainCnt++;

  for (bLedCnt = 1; bLedCnt <= bHmi_NumOfLeds_c; bLedCnt++)
#else
  if (++bLedCnt > bHmi_NumOfLeds_c) bLedCnt = 1u;
#endif
  {
    switch (arHmi_LedData[bLedCnt-1].qLedState)
    {
      case eHmi_LedOff_c:
        BSP_LedClear(bLedCnt-1);
        arHmi_LedData[bLedCnt-1].gLedIsOn = FALSE;
        arHmi_LedData[bLedCnt-1].qLedState = eHmi_LedStdBy_c;
        break;

      case eHmi_LedOn_c:
        BSP_LedSet(bLedCnt-1);
        arHmi_LedData[bLedCnt-1].gLedIsOn = TRUE;
        arHmi_LedData[bLedCnt-1].qLedState = eHmi_LedStdBy_c;
        break;

      case eHmi_LedStdBy_c:
        break;

#ifdef HMI_DRIVER_EXTENDED_SUPPORT
      case eHmi_LedToggle_c:
          BSP_LedToggle(bLedCnt-1);
          arHmi_LedData[bLedCnt-1].gLedIsOn = \
                !arHmi_LedData[bLedCnt-1].gLedIsOn;
          arHmi_LedData[bLedCnt-1].qLedState = eHmi_LedStdBy_c;
        break;

      case eHmi_LedBlink2Hz_c:
        if ((wHmi_LedMainCnt == wTmr_2HzCntMax_c) ||
            (wHmi_LedMainCnt == wTmr_2HzCntMax_c + wTmr_1HzCntMax_c)
        )
        {
          bLedChngReq = TRUE;
        }
      case eHmi_LedBlink1Hz_c:
        if (wHmi_LedMainCnt == wTmr_1HzCntMax_c)
        {
          bLedChngReq = TRUE;
        }
      case eHmi_LedBlinkHalfHz_c:
        if (wHmi_LedMainCnt == wTmr_0Hz5CntMax_c)
        {
          bLedChngReq = TRUE;
        }
        break;
#endif

      case eHmi_LedBlinkOnce_c:
        BSP_LedSet(bLedCnt-1);
        arHmi_LedData[bLedCnt-1].gLedIsOn = TRUE;
        arHmi_LedData[bLedCnt-1].wLedBlinkCnt = 0;
        arHmi_LedData[bLedCnt-1].qLedState = eHmi_LedBlinkWait_c;
      case eHmi_LedBlinkWait_c:
        if ( arHmi_LedData[bLedCnt-1].wLedBlinkCnt++ > (wTmr_2HzCntMax_c / (2.0
#ifndef HMI_DRIVER_EXTENDED_SUPPORT
            * 4u
#endif
        )) )
        {
          arHmi_LedData[bLedCnt-1].qLedState = eHmi_LedOff_c;
        }
        break;

      default:
        break;
    }

  #ifdef HMI_DRIVER_EXTENDED_SUPPORT
    if (bLedChngReq)
    {
      if (arHmi_LedData[bLedCnt-1].gLedIsOn)
      {
        BSP_LedClear(bLedCnt-1);
        arHmi_LedData[bLedCnt-1].gLedIsOn = FALSE;
      }
      else
      {
        BSP_LedSet(bLedCnt-1);
        arHmi_LedData[bLedCnt-1].gLedIsOn = TRUE;
      }
    }
  #endif
  }
  #ifdef HMI_DRIVER_EXTENDED_SUPPORT
  if (wHmi_LedMainCnt == wTmr_0Hz5CntMax_c)
  {
    wHmi_LedMainCnt = 0;
  }
  #endif
}
#endif

#ifdef SILABS_PLATFORM_COMPONENT_PB
/*!
 * This function is used to initialize push-button handler.
 *
 * @return  None.
 *
 * @note It has to be called from the initialization section.
 */
void vHmi_InitPbHandler(void)
{
  /*! NOTE: Re-initialization of LED Handler supported by the extended HMI driver */

#ifdef HMI_DRIVER_EXTENDED_SUPPORT
  rHmi_PbData.qPbHandlerState = eHmi_PbStandBy_c;
  rHmi_PbData.wPbPushTime     = 0;
  rHmi_PbData.bPbPushTrack    = eHmi_PbNo_c;
  rHmi_PbData.bPbPushTrackAct = eHmi_PbNo_c;
  rHmi_PbData.bPbWaitTime     = 0;
#endif
}

/*!
 * This function is used to check if any of the push-buttons is pushed.
 *
 * @param[out] *boPbPushTrack Read value of pushed button.
 * @param[out] *woPbPushTime Push time of pushed button.
 *
 * @return  Pushed state of push-buttons.
 */
uint8_t gHmi_PbIsPushed(uint8_t *boPbPushTrack, uint16_t *woPbPushTime)
{
  if (rHmi_PbData.qPbHandlerState == eHmi_PbPushed_c)
  {
    *boPbPushTrack  = rHmi_PbData.bPbPushTrack;
    *woPbPushTime   = rHmi_PbData.wPbPushTime;
    return TRUE;
  }
  else
  {
    *boPbPushTrack  = eHmi_PbNo_c;
    *woPbPushTime   = 0;
    return FALSE;
  }
}

#ifdef HMI_DRIVER_EXTENDED_SUPPORT
/*!
 * This function is used to check if there is unhandled push-buttons event.
 *
 * @return  True if there is unhandled push-button event.
 */
uint8_t gHmi_IsPbUnHandled(void)
{
  if (rHmi_PbData.bPbPushTrackLast > eHmi_PbNo_c)
  {
    return TRUE;
  }
  else
  {
    return FALSE;
  }
}

/*!
 * This function is used to read last pushed button(s), push track holder is erased if button(s) was already released.
 *
 * @param[out] *woPbPushTime Push time of pushed button.

 * @return  Push track holder of last pushed button(s).
 */
uint8_t bHmi_PbGetLastButton(uint16_t *woPbPushTime)
{
  uint8_t bPbPushTrackTemp = rHmi_PbData.bPbPushTrackLast;

  *woPbPushTime = rHmi_PbData.wPbPushTimeLast;

  if (rHmi_PbData.bPbPushTrackLast > eHmi_PbNo_c)
  {
    rHmi_PbData.bPbPushTrackLast = eHmi_PbNo_c;
    rHmi_PbData.wPbPushTimeLast = 0;
  }

  return bPbPushTrackTemp;
}
#endif

/*!
 * This function is used to handle push-button management.
 *
 * @return  None.
 */
void vHmi_PbHandler(void)
{
#ifdef HMI_DRIVER_EXTENDED_SUPPORT
  static
#endif
  uint8_t bButtonCnt = 0u;
  uint8_t bTemp;

#ifdef SILABS_PLATFORM_WMB
  uint8_t lInvokeCnt = 0u;
#endif

#ifdef HMI_DRIVER_EXTENDED_SUPPORT
  if (rHmi_PbData.qPbHandlerState > eHmi_PbNoRun_c)
#endif
  {

#ifdef SILABS_PLATFORM_WMB
    if (++lInvokeCnt == 0u)
#endif
    {
      rHmi_PbData.bPbPushTrackAct = eHmi_PbNo_c;

      for (bButtonCnt = 1; bButtonCnt <= bHmi_NumOfPbs_c; bButtonCnt++)       // Store state of the buttons
      {
        bTemp = !BSP_GetPB(bButtonCnt);
        rHmi_PbData.bPbPushTrackAct += bTemp<<(bButtonCnt-1);
      }
    }

    switch (rHmi_PbData.qPbHandlerState)
    {
      case eHmi_PbStandByWaitAllReleased_c:
        if (rHmi_PbData.bPbPushTrackAct == eHmi_PbNo_c)
        {
          rHmi_PbData.qPbHandlerState = eHmi_PbStandBy_c;
        }
        break;

      case eHmi_PbStandBy_c:
        if (rHmi_PbData.bPbPushTrackAct > eHmi_PbNo_c)
        {
          rHmi_PbData.qPbHandlerState = eHmi_PbStateChanged_c;
        }
        break;

      case eHmi_PbStateChanged_c:
        rHmi_PbData.wPbPushTime = 0;
        rHmi_PbData.bPbPushTrack = rHmi_PbData.bPbPushTrackAct;

        if (rHmi_PbData.bPbPushTrackAct == eHmi_PbNo_c)
        {
          rHmi_PbData.qPbHandlerState = eHmi_PbStandBy_c;
        }
        else
        {
          rHmi_PbData.qPbHandlerState = eHmi_PbDebounceWait_c;
        }
        break;

      case eHmi_PbDebounceWait_c:
        if (rHmi_PbData.bPbWaitTime++ > bHmi_PbDebounceWaitTime_c)
        {
          rHmi_PbData.bPbWaitTime = 0;
          rHmi_PbData.wPbPushTime = 0;
          rHmi_PbData.qPbHandlerState = eHmi_PbPushed_c;
        }
        break;

      case eHmi_PbPushed_c:
        /* If button released, or one released but more were pushed store the state*/
        if (rHmi_PbData.bPbPushTrackAct < rHmi_PbData.bPbPushTrack)
        {
          rHmi_PbData.qPbHandlerState = eHmi_PbReleaseWait_c;
        }
        /* If one more button is pushed goto change state */
        else if (rHmi_PbData.bPbPushTrackAct > rHmi_PbData.bPbPushTrack)
        {
          rHmi_PbData.qPbHandlerState = eHmi_PbStateChanged_c;
        }
        /* Increase counter if no change in button states */
        else
        {
          rHmi_PbData.wPbPushTime++;
        }
        break;

      case eHmi_PbReleaseWait_c:
        if (rHmi_PbData.bPbWaitTime++ > bHmi_PbDebounceWaitTime_c)
        {
          rHmi_PbData.bPbWaitTime = 0;
          rHmi_PbData.qPbHandlerState   = eHmi_PbStandByWaitAllReleased_c;
          rHmi_PbData.bPbPushTrackLast  = rHmi_PbData.bPbPushTrack;
          rHmi_PbData.wPbPushTimeLast   = rHmi_PbData.wPbPushTime;
        }
        break;
      case eHmi_PbNoRun_c:
    	  break;
    }
  }
}
#endif

#if ((defined SILABS_PLATFORM_COMPONENT_LED) && (defined SILABS_PLATFORM_COMPONENT_PB))
  #ifdef HMI_DRIVER_EXTENDED_SUPPORT
  /*!
   * This function is used to show the actual state of the push-buttons on the Leds.
   *
   * @return  None.
   */
  void vHmi_ShowPbOnLeds(void)
  {
    
    uint8_t boPbPushTrack;
    uint16_t woPbPushTime;
    uint8_t bPbLedCnt;

    gHmi_PbIsPushed(&boPbPushTrack, &woPbPushTime);

    for (bPbLedCnt = 1; bPbLedCnt <= 4; bPbLedCnt++)
    {
      if (boPbPushTrack & aqHmi_PbStates[bPbLedCnt])
      {
        vHmi_ChangeLedState(aqHmi_Leds[bPbLedCnt], eHmi_LedOn_c);
      }
      else
      {
        vHmi_ChangeLedState(aqHmi_Leds[bPbLedCnt], eHmi_LedOff_c);
      }
    }
  }
  #endif
#endif

#ifdef SILABS_PLATFORM_COMPONENT_SWITCH
//NO Switch on EFM boards
#endif

#ifdef SILABS_PLATFORM_COMPONENT_BUZZER
//NO buzzer on EFM board.

#endif


