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
File        : SPINBOX.h
Purpose     : SpinButton include
--------------------END-OF-HEADER-------------------------------------
*/

#ifndef SPINBOX_H
#define SPINBOX_H

#include "WM.h"
#include "DIALOG_Intern.h" /* Req. for Create indirect data structure */
#include "WIDGET.h"        /* Req. for Create indirect data structure */
#include "GUI_Debug.h"
#include "EDIT.h"


#if GUI_WINSUPPORT

#if defined(__cplusplus)
extern "C" {     /* Make sure we have C-declarations in C++ programs */
#endif

/*********************************************************************
*
*       Defines
*
**********************************************************************
*/
/*********************************************************************
*
*       States
*
**********************************************************************
*/
#define SPINBOX_STATE_PRESSED(x)       (1 << x)             // These flags are stored in (SPINBOX_OBJ->State)        | x must be 0 or 1
#define SPINBOX_STATE_FOCUS             WIDGET_STATE_FOCUS  // This is read from         (SPINBOX_OBJ->Widget.State)

#define SPINBOX_EDGE_RIGHT              0
#define SPINBOX_EDGE_LEFT               1
#define SPINBOX_EDGE_CENTER             2

/*********************************************************************
*
*       Color indices
*
*  
*/
#define SPINBOX_CI_DISABLED EDIT_CI_DISABLED
#define SPINBOX_CI_ENABLED  EDIT_CI_ENABLED
#define SPINBOX_CI_PRESSED  2

/*********************************************************************
*
*       Skinning property indices
*/
#define SPINBOX_SKINFLEX_PI_PRESSED  0
#define SPINBOX_SKINFLEX_PI_FOCUSSED 1
#define SPINBOX_SKINFLEX_PI_ENABLED  2
#define SPINBOX_SKINFLEX_PI_DISABLED 3

#define SPINBOX_SKIN_FLEX_RADIUS     2

/*********************************************************************
*
*       Public Types
*
**********************************************************************
*/
typedef WM_HMEM SPINBOX_Handle;

typedef struct {
  GUI_COLOR aColorFrame[2];    // [0] Outer color of surrounding frame.
                               // [1] Inner color of surrounding frame.
  GUI_COLOR aColorUpper[2];    // [0] Upper color of gradient for upper button.
                               // [1] Lower color of gradient for upper button.
  GUI_COLOR aColorLower[2];    // [0] Upper color of gradient for lower button.
                               // [1] Lower color of gradient for lower button.
  GUI_COLOR ColorArrow;        // Color of the button arrow.
  GUI_COLOR ColorBk;           // Color of the background.                      // See WIDGET_ITEM_CREATE in SPINBOX_DrawSkinFlex()
  GUI_COLOR ColorText;         // Color of the text.                            // See WIDGET_ITEM_CREATE in SPINBOX_DrawSkinFlex()
  GUI_COLOR ColorButtonFrame;  // Color of the button frame.
} SPINBOX_SKINFLEX_PROPS;

/*********************************************************************
*
*       Standard member functions
*
**********************************************************************
*/
#define SPINBOX_EnableMemdev(hObj)  WM_EnableMemdev(hObj)
#define SPINBOX_DisableMemdev(hObj) WM_DisableMemdev(hObj)
#define SPINBOX_Delete(hObj)        WM_DeleteWindow(hObj)
#define SPINBOX_Paint(hObj)         WM_Paint(hObj)
#define SPINBOX_Invalidate(hObj)    WM_InvalidateWindow(hObj)

/*********************************************************************
*
*       Member functions: Skinning
*
**********************************************************************
*/
void SPINBOX_GetSkinFlexProps     (SPINBOX_SKINFLEX_PROPS * pProps, int Index);
void SPINBOX_SetSkinClassic       (SPINBOX_Handle hObj);
void SPINBOX_SetSkin              (SPINBOX_Handle hObj, WIDGET_DRAW_ITEM_FUNC * pfDrawSkin);
int  SPINBOX_DrawSkinFlex         (const WIDGET_ITEM_DRAW_INFO * pDrawItemInfo);
void SPINBOX_SetSkinFlexProps     (const SPINBOX_SKINFLEX_PROPS * pProps, int Index);
void SPINBOX_SetDefaultSkinClassic(void);
WIDGET_DRAW_ITEM_FUNC * SPINBOX_SetDefaultSkin(WIDGET_DRAW_ITEM_FUNC * pfDrawSkin);

#define SPINBOX_SKIN_FLEX    SPINBOX_DrawSkinFlex

/*********************************************************************
*
*       Create functions
*
**********************************************************************
*/
SPINBOX_Handle SPINBOX_CreateEx      (int x0, int y0, int xSize, int ySize, WM_HWIN hParent, int WinFlags, int Id, int Min, int Max);
SPINBOX_Handle SPINBOX_CreateUser    (int x0, int y0, int xSize, int ySize, WM_HWIN hParent, int WinFlags, int Id, int Min, int Max, int NumExtraBytes);
SPINBOX_Handle SPINBOX_CreateIndirect(const GUI_WIDGET_CREATE_INFO * pCreateInfo, WM_HWIN hWinParent, int x0, int y0, WM_CALLBACK * cb);

/*********************************************************************
*
*       The callback ...
*
* Do not call it directly ! It is only to be used from within an
* overwritten callback.
*
**********************************************************************
*/
void SPINBOX_Callback(WM_MESSAGE * pMsg);

/*********************************************************************
*
*       Member functions
*
* Methods returning or changing properties
*
**********************************************************************
*/
void        SPINBOX_EnableBlink     (SPINBOX_Handle hObj, int Period, int OnOff);
GUI_COLOR   SPINBOX_GetBkColor      (SPINBOX_Handle hObj, unsigned int Index);
GUI_COLOR   SPINBOX_GetButtonBkColor(SPINBOX_Handle hObj, unsigned int Index);
EDIT_Handle SPINBOX_GetEditHandle   (SPINBOX_Handle hObj);
int         SPINBOX_GetUserData     (SPINBOX_Handle hObj, void       * pDest, int NumBytes);
int         SPINBOX_GetValue        (SPINBOX_Handle hObj);
void        SPINBOX_SetBkColor      (SPINBOX_Handle hObj, unsigned int Index, GUI_COLOR Color);
void        SPINBOX_SetButtonBkColor(SPINBOX_Handle hObj, unsigned int Index, GUI_COLOR Color);
void        SPINBOX_SetButtonSize   (SPINBOX_Handle hObj, unsigned ButtonSize);
void        SPINBOX_SetEdge         (SPINBOX_Handle hObj, U8  Edge);
void        SPINBOX_SetFont         (SPINBOX_Handle hObj, const GUI_FONT GUI_UNI_PTR * pFont);
void        SPINBOX_SetTextColor    (SPINBOX_Handle hObj, unsigned int Index, GUI_COLOR Color);
int         SPINBOX_SetUserData     (SPINBOX_Handle hObj, const void * pSrc,  int NumBytes);
void        SPINBOX_SetValue        (SPINBOX_Handle hObj, int v);

/*********************************************************************
*
*       Functions for default value management
*
**********************************************************************
*/
U16  SPINBOX_GetDefaultButtonSize(void);
void SPINBOX_SetDefaultButtonSize(U16 ButtonSize);

#if defined(__cplusplus)
  }
#endif

#endif   /* if GUI_WINSUPPORT */
#endif   /* SPINBOX_H */
