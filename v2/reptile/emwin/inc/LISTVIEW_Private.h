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
File        : LISTVIEW_Private.h
Purpose     : Private LISTVIEW include
--------------------END-OF-HEADER-------------------------------------
*/

#ifndef LISTVIEW_PRIVATE_H
#define LISTVIEW_PRIVATE_H

#include <stddef.h>

#include "WM.h"
#include "LISTVIEW.h"
#include "GUI_ARRAY.h"

#if GUI_WINSUPPORT

/*********************************************************************
*
*       Defines
*
**********************************************************************
*/
#define SORT_TYPE U16

#define LISTVIEW_CELL_INFO_COLORS (1 << 0)
#define LISTVIEW_CELL_INFO_BITMAP (1 << 1)

/*********************************************************************
*
*       Object definition
*
**********************************************************************
*/

typedef struct {
  GUI_COLOR                    aBkColor[4];
  GUI_COLOR                    aTextColor[4];
  GUI_COLOR                    GridColor;
  const GUI_FONT GUI_UNI_PTR * pFont;
  U16                          ScrollStepH;
  GUI_WRAPMODE                 WrapMode;
} LISTVIEW_PROPS;

typedef struct {
  void (* pfDraw)(LISTVIEW_Handle hObj, unsigned Column, unsigned Row, GUI_RECT * pRect);
  void  * pData;
  GUI_COLOR aBkColor[4];
  GUI_COLOR aTextColor[4];
  I16 xOff, yOff;
  U8 Flags;
} LISTVIEW_CELL_INFO;

typedef struct {
  WM_HMEM hCellInfo;                /* Optional cell info. If 0, there is no cell info. (Defaults used) */
  char acText[1];
} LISTVIEW_CELL;

typedef struct {
  GUI_ARRAY CellArray;
  char Disabled;
  U32 UserData;
} LISTVIEW_ROW;

typedef struct {
  U8 Align;
  int      (* fpCompare)    (const void * p0, const void * p1);          /* User function to be called to compare the contents of 2 cells */
} LISTVIEW_COLUMN;

typedef struct LISTVIEW_Obj LISTVIEW_Obj;

typedef struct {
  WM_HMEM   hSortArray;
  SORT_TYPE SortArrayNumItems;
  int      (* fpSort)       (LISTVIEW_Handle hObj);                      /* Function to be called to get a sorted array */
  void     (* fpFree)       (WM_HMEM hObj);                              /* Function to be called to free the sort object */
  U8  Reverse;
} LISTVIEW_SORT;

struct LISTVIEW_Obj {
  WIDGET          Widget;
  HEADER_Handle   hHeader;
  GUI_ARRAY       RowArray;         /* Each entry is a handle of LISTVIEW_ROW structure. */
  GUI_ARRAY       ColumnArray;      /* Each entry is a handle of LISTVIEW_COLUMN structure. */
  LISTVIEW_PROPS  Props;
  int             Sel;
  int             ShowGrid;
  int             SortIndex;        /* Column for sorting */
  unsigned        RowDistY;
  unsigned        LBorder;
  unsigned        RBorder;
  unsigned        Fixed;
  WM_SCROLL_STATE ScrollStateV;
  WM_SCROLL_STATE ScrollStateH;
  WM_HMEM         hSort;
  U8              IsSorted;
  U8              IsPresorted;
  U8              ReverseSort;      /* Set to 1 if reverse sorting is reqired */
  U8              Flags;
  #if GUI_DEBUG_LEVEL > 1
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
  #define LISTVIEW_INIT_ID(p) p->DebugId = LISTVIEW_ID
#else
  #define LISTVIEW_INIT_ID(p)
#endif

#if GUI_DEBUG_LEVEL >= GUI_DEBUG_LEVEL_CHECK_ALL
  LISTVIEW_Obj * LISTVIEW_LockH(LISTVIEW_Handle h);
  #define LISTVIEW_LOCK_H(h)   LISTVIEW_LockH(h)
#else
  #define LISTVIEW_LOCK_H(h)   (LISTVIEW_Obj *)GUI_LOCK_H(h)
#endif

/*********************************************************************
*
*       Private (module internal) data
*
**********************************************************************
*/

extern LISTVIEW_PROPS LISTVIEW_DefaultProps;

/*********************************************************************
*
*       Private (module internal) functions
*
**********************************************************************
*/
int       LISTVIEW__UpdateScrollParas    (LISTVIEW_Handle hObj);
void      LISTVIEW__InvalidateInsideArea (LISTVIEW_Handle hObj);
unsigned  LISTVIEW__GetRowDistY          (const LISTVIEW_Obj* pObj);
void      LISTVIEW__InvalidateRow        (LISTVIEW_Handle hObj, LISTVIEW_Obj* pObj, int Sel);
int       LISTVIEW__UpdateScrollPos      (LISTVIEW_Handle hObj);
unsigned  LISTVIEW__GetNumRows           (LISTVIEW_Obj* pObj);
unsigned  LISTVIEW__GetNumColumns        (LISTVIEW_Obj* pObj);
int       LISTVIEW__GetSel               (LISTVIEW_Obj* pObj);
void      LISTVIEW__SetSel               (LISTVIEW_Handle hObj, int NewSel);
void      LISTVIEW__Sort                 (LISTVIEW_Handle hObj, LISTVIEW_Obj* pObj);
void      LISTVIEW__MoveSel              (LISTVIEW_Handle hObj, int Dir);
void      LISTVIEW__InvalidateRowAndBelow(LISTVIEW_Handle hObj, LISTVIEW_Obj* pObj, int Sel);

#endif /* GUI_WINSUPPORT */

#else                            /* Avoid problems with empty object modules */
  void LISTVIEW_C(void) {}
#endif

/*************************** End of file ****************************/
