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
File        : SLIDER_Private.h
Purpose     : SLIDER private header file
--------------------END-OF-HEADER-------------------------------------
*/

#ifndef SLIDER_PRIVATE_H
#define SLIDER_PRIVATE_H

#include "WM.h"
#include "WIDGET.h"
#include "SLIDER.h"

#if GUI_WINSUPPORT

/*********************************************************************
*
*       Object definition
*
**********************************************************************
*/
typedef struct {
  WIDGET_DRAW_ITEM_FUNC * pfDrawSkin;
} SLIDER_SKIN_PRIVATE;

typedef struct {
  GUI_COLOR BkColor;
  GUI_COLOR BarColor;
  GUI_COLOR FocusColor;
  GUI_COLOR TickColor;
  SLIDER_SKIN_PRIVATE SkinPrivate;
} SLIDER_PROPS;

typedef struct {
  WIDGET Widget;
  SLIDER_PROPS Props;
  WIDGET_SKIN const * pWidgetSkin;
  int Min, Max, v;
  int Flags;
  int NumTicks;
  I16 Width;
  #if GUI_DEBUG_LEVEL >1
    U32 DebugId;
  #endif
} SLIDER_Obj;

/*********************************************************************
*
*       Macros for internal use
*
**********************************************************************
*/
#if GUI_DEBUG_LEVEL >= GUI_DEBUG_LEVEL_CHECK_ALL
  #define SLIDER_INIT_ID(p) p->DebugId = SLIDER_ID
#else
  #define SLIDER_INIT_ID(p)
#endif

#if GUI_DEBUG_LEVEL >= GUI_DEBUG_LEVEL_CHECK_ALL
  SLIDER_Obj * SLIDER_LockH(SLIDER_Handle h);
  #define SLIDER_LOCK_H(h)   SLIDER_LockH(h)
#else
  #define SLIDER_LOCK_H(h)   (SLIDER_Obj *)GUI_LOCK_H(h)
#endif

#ifndef   SLIDER_SUPPORT_TRANSPARENCY
  #define SLIDER_SUPPORT_TRANSPARENCY WM_SUPPORT_TRANSPARENCY
#endif

/*********************************************************************
*
*       Public data (internal defaults)
*
**********************************************************************
*/
extern SLIDER_PROPS SLIDER__DefaultProps;

extern const WIDGET_SKIN SLIDER__SkinClassic;
extern       WIDGET_SKIN SLIDER__Skin;

extern WIDGET_SKIN const * SLIDER__pSkinDefault;

#endif

#endif
