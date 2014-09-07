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
File        : WIDGET_Window.c
Purpose     : Example demonstrating the use of a WINDOW widget
----------------------------------------------------------------------
*/

#include "GUI.h"
#include "DIALOG.h"

/*********************************************************************
*
*       Static data
*
**********************************************************************
*/
/*********************************************************************
*
*       _aDialog
*
* Purpose
*   Dialog resource using a WINDOW widget
*/
static const GUI_WIDGET_CREATE_INFO _aDialog[] = {
  { WINDOW_CreateIndirect, "",       0,                0,   0, 260, 200, 0},
  { TEXT_CreateIndirect,   "Dialog", 0,               80,   5, 100, 20, TEXT_CF_HCENTER },
  { BUTTON_CreateIndirect, "Close",  GUI_ID_BUTTON0,  80, 160, 100, 20}
};

/*********************************************************************
*
*       Static code
*
**********************************************************************
*/
/*********************************************************************
*
*       _cbDialog
*
* Purpose
*   Callback routine of the dialog
*/
static void _cbDialog(WM_MESSAGE * pMsg) {
  int NCode, Id;
  switch (pMsg->MsgId) {
  case WM_PAINT:
    GUI_SetBkColor(GUI_GREEN);
    GUI_Clear();
    break;
  case WM_NOTIFY_PARENT:
    Id    = WM_GetId(pMsg->hWinSrc);    /* Id of widget */
    NCode = pMsg->Data.v;               /* Notification code */
    switch (NCode) {
    case WM_NOTIFICATION_RELEASED:      /* React only if released */
      switch (Id) {
      case GUI_ID_BUTTON0:
        GUI_EndDialog(pMsg->hWin, 0);
        break;
      }
      break;
    }
    break;
  default:
    WM_DefaultProc(pMsg);
  }
}

/*********************************************************************
*
*       Public code
*
**********************************************************************
*/
/*********************************************************************
*
*       MainTask
*/
void MainTask(void) {
  GUI_Init();
  while(1) {
    GUI_DispStringHCenterAt("WIDGET_Window sample\nshows how to use a WINDOW widget", 160, 5);
    GUI_ExecDialogBox(_aDialog, GUI_COUNTOF(_aDialog), _cbDialog, WM_HBKWIN, 30, 30);
    GUI_Clear();
    GUI_DispStringHCenterAt("Dialog has been closed", 160, 5);
    GUI_Delay(1000);
    GUI_Clear();
  }
}

/*************************** End of file ****************************/

