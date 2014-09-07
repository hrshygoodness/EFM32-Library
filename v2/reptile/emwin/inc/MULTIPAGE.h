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
File        : MULTIPAGE.h
Purpose     : MULTIPAGE include
--------------------END-OF-HEADER-------------------------------------
*/

#ifndef MULTIPAGE_H
#define MULTIPAGE_H

#include "WM.h"
#include "DIALOG.h"      /* Req. for Create indirect data structure */

#if GUI_WINSUPPORT

#if defined(__cplusplus)
extern "C" {     /* Make sure we have C-declarations in C++ programs */
#endif

/*********************************************************************
*
*       defines
*
**********************************************************************
*/
/*********************************************************************
*
*       Create / Status flags
*/

#define MULTIPAGE_ALIGN_LEFT    (0<<0)
#define MULTIPAGE_ALIGN_RIGHT   (1<<0)
#define MULTIPAGE_ALIGN_TOP     (0<<2)
#define MULTIPAGE_ALIGN_BOTTOM  (1<<2)

#define MULTIPAGE_CF_ROTATE_CW WIDGET_CF_VERTICAL

#define MULTIPAGE_CI_DISABLED 0
#define MULTIPAGE_CI_ENABLED  1

/*********************************************************************
*
*       Public Types
*
**********************************************************************
*/

typedef WM_HMEM MULTIPAGE_Handle;

/*********************************************************************
*
*       Create functions
*
**********************************************************************
*/

MULTIPAGE_Handle MULTIPAGE_Create        (int x0, int y0, int xsize, int ysize, WM_HWIN hParent, int Id, int Flags, int SpecialFlags);
MULTIPAGE_Handle MULTIPAGE_CreateIndirect(const GUI_WIDGET_CREATE_INFO* pCreateInfo, WM_HWIN hWinParent, int x0, int y0, WM_CALLBACK* cb);
MULTIPAGE_Handle MULTIPAGE_CreateEx      (int x0, int y0, int xsize, int ysize, WM_HWIN hParent,
                                          int WinFlags, int ExFlags, int Id);
MULTIPAGE_Handle MULTIPAGE_CreateUser    (int x0, int y0, int xsize, int ysize, WM_HWIN hParent,
                                          int WinFlags, int ExFlags, int Id, int NumExtraBytes);

/*********************************************************************
*
*       The callback ...
*
* Do not call it directly ! It is only to be used from within an
* overwritten callback.
*/
void MULTIPAGE_Callback(WM_MESSAGE *pMsg);

/*********************************************************************
*
*       Member functions
*
**********************************************************************
*/

/* Methods changing properties */
void        MULTIPAGE_AddPage       (MULTIPAGE_Handle hObj, WM_HWIN hWin ,const char* pText);
void        MULTIPAGE_DeletePage    (MULTIPAGE_Handle hObj, unsigned Index, int Delete);
void        MULTIPAGE_DisablePage   (MULTIPAGE_Handle hObj, unsigned Index);
void        MULTIPAGE_EnablePage    (MULTIPAGE_Handle hObj, unsigned Index);
int         MULTIPAGE_GetSelection  (MULTIPAGE_Handle hObj);
WM_HWIN     MULTIPAGE_GetWindow     (MULTIPAGE_Handle hObj, unsigned Index);
int         MULTIPAGE_GetUserData   (MULTIPAGE_Handle hObj, void * pDest, int NumBytes);
int         MULTIPAGE_IsPageEnabled (MULTIPAGE_Handle hObj, unsigned Index);
void        MULTIPAGE_SelectPage    (MULTIPAGE_Handle hObj, unsigned Index);
void        MULTIPAGE_SetAlign      (MULTIPAGE_Handle hObj, unsigned Align);
void        MULTIPAGE_SetBkColor    (MULTIPAGE_Handle hObj, GUI_COLOR Color, unsigned Index);
void        MULTIPAGE_SetFont       (MULTIPAGE_Handle hObj, const GUI_FONT GUI_UNI_PTR * pFont);
void        MULTIPAGE_SetRotation   (MULTIPAGE_Handle hObj, unsigned Rotation);
void        MULTIPAGE_SetText       (MULTIPAGE_Handle hObj, const char* pText, unsigned Index);
void        MULTIPAGE_SetTextColor  (MULTIPAGE_Handle hObj, GUI_COLOR Color, unsigned Index);
int         MULTIPAGE_SetUserData   (MULTIPAGE_Handle hObj, const void * pSrc, int NumBytes);

/*********************************************************************
*
*       Get/Set defaults
*
**********************************************************************
*/

unsigned                     MULTIPAGE_GetDefaultAlign(void);
GUI_COLOR                    MULTIPAGE_GetDefaultBkColor(unsigned Index);
const GUI_FONT GUI_UNI_PTR * MULTIPAGE_GetDefaultFont(void);
GUI_COLOR                    MULTIPAGE_GetDefaultTextColor(unsigned Index);

void                         MULTIPAGE_SetDefaultAlign(unsigned Align);
void                         MULTIPAGE_SetDefaultBkColor(GUI_COLOR Color, unsigned Index);
void                         MULTIPAGE_SetDefaultFont(const GUI_FONT GUI_UNI_PTR * pFont);
void                         MULTIPAGE_SetDefaultTextColor(GUI_COLOR Color, unsigned Index);

void      MULTIPAGE_SetEffectColor    (unsigned Index, GUI_COLOR Color);
GUI_COLOR MULTIPAGE_GetEffectColor    (unsigned Index);
int       MULTIPAGE_GetNumEffectColors(void);

#if defined(__cplusplus)
  }
#endif

#endif   /* if GUI_WINSUPPORT */
#endif   /* MULTIPAGE_H */
