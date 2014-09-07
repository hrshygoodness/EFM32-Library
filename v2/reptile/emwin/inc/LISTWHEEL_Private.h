/*********************************************************************
*                SEGGER Microcontroller GmbH & Co. KG                *
*        Solutions for real time microcontroller applications        *
**********************************************************************
*                                                                    *
*        (c) 1996 - 2012  SEGGER Microcontroller GmbH & Co. KG       *
*                                                                    *
*        Internet: www.segger.com    Support:  support@segger.com    *
*                                                                    *
**********************************************************************

** emWin V5.14 - Graphical user interface for embedded applications **
All  Intellectual Property rights  in the Software belongs to  SEGGER.
emWin is protected by  international copyright laws.  Knowledge of the
source code may not be used to write a similar product.  This file may
only be used in accordance with the following terms:

The software has been licensed to Energy Micro AS whose registered office
is situated at  Sandakerveien 118, N-0484 Oslo, NORWAY solely
for  the  purposes  of  creating  libraries  for Energy Micros ARM Cortex-M3, M4F
processor-based  devices,  sublicensed  and distributed  under the terms and
conditions  of  the   End  User  License Agreement supplied by Energy Micro AS. 
Full source code is available at: www.segger.com

We appreciate your understanding and fairness.
----------------------------------------------------------------------
File        : LISTWHEEL_Private.h
Purpose     : Private LISTWHEEL include
--------------------END-OF-HEADER-------------------------------------
*/

#ifndef LISTWHEEL_PRIVATE_H
#define LISTWHEEL_PRIVATE_H

#include "LISTWHEEL.h"
#include "WM.h"
#include "GUI_ARRAY.h"
#include "WIDGET.h"

#if GUI_WINSUPPORT

/*********************************************************************
*
*       Defines
*
**********************************************************************
*/
#define LISTWHEEL_STATE_PRESSED WIDGET_STATE_USER0

/*********************************************************************
*
*       Object definition
*
**********************************************************************
*/

typedef struct {
  void * pData;
  char acText[1];
} LISTWHEEL_ITEM;

typedef struct {
  const GUI_FONT GUI_UNI_PTR * pFont;
  GUI_COLOR aBackColor[2];
  GUI_COLOR aTextColor[2];
  I16 Align;
} LISTWHEEL_PROPS;

typedef struct {
  WIDGET Widget;
  GUI_ARRAY ItemArray;
  WIDGET_DRAW_ITEM_FUNC * pfOwnerDraw;
  LISTWHEEL_PROPS Props;
  #if GUI_DEBUG_LEVEL > 1
    U32 DebugId;
  #endif
  WM_HMEM hTimer;
  unsigned LBorder;
  unsigned RBorder;
  unsigned LineHeight;
  int Sel;
  int TimeTouched;   // Time stamp of last touch event
  int Pos;           // Current position in pixels
  int Velocity;      // Motion in pixels
  int SnapPosition;  // Snap position in pixels
  int TouchPos;      // Y-position of last touch event
  int ySizeData;     // Data size in pixels
  int Destination;   // Destination position in pixels
  unsigned TimerPeriod; // Period of timer events
} LISTWHEEL_OBJ;

/*********************************************************************
*
*       Macros for internal use
*
**********************************************************************
*/
#if GUI_DEBUG_LEVEL >= GUI_DEBUG_LEVEL_CHECK_ALL
  #define LISTWHEEL_INIT_ID(p) p->DebugId = LISTWHEEL_ID
#else
  #define LISTWHEEL_INIT_ID(p)
#endif

#if GUI_DEBUG_LEVEL >= GUI_DEBUG_LEVEL_CHECK_ALL
  LISTWHEEL_OBJ * LISTWHEEL_LockH(LISTWHEEL_Handle h);
  #define LISTWHEEL_LOCK_H(h)   LISTWHEEL_LockH(h)
#else
  #define LISTWHEEL_LOCK_H(h)   (LISTWHEEL_OBJ *)GUI_LOCK_H(h)
#endif

/*********************************************************************
*
*       Private (module internal) data
*
**********************************************************************
*/
extern LISTWHEEL_PROPS LISTWHEEL_DefaultProps;

/*********************************************************************
*
*       Private (module internal) functions
*
**********************************************************************
*/
const char * LISTWHEEL__GetpStringLocked(LISTWHEEL_Handle hObj, int Index, LISTWHEEL_ITEM ** ppItem);

#endif /* GUI_WINSUPPORT */

#else                            /* Avoid problems with empty object modules */
  void LISTWHEEL_C(void) {}
#endif

/*************************** End of file ****************************/
