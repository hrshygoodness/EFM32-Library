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
File        : MULTIPAGE_Private.h
Purpose     : Private MULTIPAGE include
--------------------END-OF-HEADER-------------------------------------
*/

#ifndef MULTIPAGE_PRIVATE_H
#define MULTIPAGE_PRIVATE_H

#include "GUI_Debug.h"
#include "GUI_ARRAY.h"
#include "MULTIPAGE.h"

#if GUI_WINSUPPORT

/*********************************************************************
*
*       Defines
*
**********************************************************************
*/

#define MULTIPAGE_STATE_ENABLED     (1<<0)
#define MULTIPAGE_STATE_SCROLLMODE  WIDGET_STATE_USER0

/* Define colors */
#define MULTIPAGE_NUMCOLORS 2

/*********************************************************************
*
*       Object definition
*
**********************************************************************
*/

typedef struct {
  WM_HWIN hWin;
  U8      Status;
  char    acText;
} MULTIPAGE_PAGE;

typedef struct {
  const GUI_FONT GUI_UNI_PTR * pFont;
  unsigned        Align;
  GUI_COLOR       aBkColor[MULTIPAGE_NUMCOLORS];
  GUI_COLOR       aTextColor[MULTIPAGE_NUMCOLORS];
} MULTIPAGE_PROPS;

typedef struct MULTIPAGE_Obj MULTIPAGE_Obj;

struct MULTIPAGE_Obj {
  WIDGET          Widget;
  void (* pfDrawTextItem)(MULTIPAGE_Obj* pObj, const char* pText, unsigned Index,
                          const GUI_RECT* pRect, int y0, int w, int ColorIndex);
  WM_HWIN         hClient;
  GUI_ARRAY       Handles;
  unsigned        Selection;
  int             ScrollState;
  MULTIPAGE_PROPS Props;
  #if GUI_DEBUG_LEVEL >1
    U32 DebugId;
  #endif  
};

/*********************************************************************
*
*       Macros for internal use
*
**********************************************************************
*/
#if GUI_DEBUG_LEVEL >= GUI_DEBUG_LEVEL_CHECK_ALL
  #define MULTIPAGE_INIT_ID(p) p->DebugId = MULTIPAGE_ID
#else
  #define MULTIPAGE_INIT_ID(p)
#endif

#if GUI_DEBUG_LEVEL >= GUI_DEBUG_LEVEL_CHECK_ALL
  MULTIPAGE_Obj * MULTIPAGE_LockH(MULTIPAGE_Handle h);
  #define MULTIPAGE_LOCK_H(h)   MULTIPAGE_LockH(h)
#else
  #define MULTIPAGE_LOCK_H(h)   (MULTIPAGE_Obj *)GUI_LOCK_H(h)
#endif

/*********************************************************************
*
*       Externals
*
**********************************************************************
*/
extern GUI_COLOR MULTIPAGE__aEffectColor[2];
extern MULTIPAGE_PROPS MULTIPAGE__DefaultProps;

/*********************************************************************
*
*       Private inteface
*
**********************************************************************
*/
void MULTIPAGE__DeleteScrollbar(MULTIPAGE_Handle hObj);
void MULTIPAGE__CalcClientRect (MULTIPAGE_Handle hObj, GUI_RECT * pRect);
void MULTIPAGE__UpdatePositions(MULTIPAGE_Handle hObj);
void MULTIPAGE__DrawTextItemH  (MULTIPAGE_Obj * pObj, const char * pText, unsigned Index, const GUI_RECT * pRect, int x0, int w, int ColorIndex);

#endif /* GUI_WINSUPPORT */

#endif /* MULTIPAGE_PRIVATE_H */
