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
File        : LCDConf.c
Purpose     : Display controller configuration (single layer)
---------------------------END-OF-HEADER------------------------------
*/

#include <string.h>
#include "GUI.h"
#include "GUIDRV_Lin.h"
#include "em_cmu.h"
#include "em_ebi.h"
#include "em_rtc.h"
#include "config.h"



extern uint8_t emwinFrameBuffer[];

/*********************************************************************
*
*       LCD_X_Config
*
* Purpose:
*   Called during the initialization process in order to set up the
*   display driver configuration.
*   
*/
void LCD_X_Config(void) 
{
 
  /* Set display driver and color conversion */
  GUI_DEVICE_CreateAndLink(GUIDRV_LIN_1, GUICC_1, 0, 0);
  
  /* Display driver configuration, required for Lin-driver */
  LCD_SetSizeEx (0, EMWIN_SIZE(PANEL_WIDTH), EMWIN_SIZE(PANEL_HEIGHT));
  LCD_SetVSizeEx(0, EMWIN_SIZE(PANEL_WIDTH), EMWIN_SIZE(PANEL_HEIGHT));

  
  /* Make emwin draw to the frame buffer */ 
  LCD_SetVRAMAddrEx(0, (void *)emwinFrameBuffer);
}

/*********************************************************************
*
*       LCD_X_DisplayDriver
*
* Purpose:
*   This function is called by the display driver for several purposes.
*   To support the according task the routine needs to be adapted to
*   the display controller. Please note that the commands marked with
*   'optional' are not cogently required and should only be adapted if 
*   the display controller supports these features.
*
* Parameter:
*   LayerIndex - Index of layer to be configured
*   Cmd        - Please refer to the details in the switch statement below
*   pData      - Pointer to a LCD_X_DATA structure
*
* Return Value:
*   < -1 - Error
*     -1 - Command not handled
*      0 - Ok
*/
int LCD_X_DisplayDriver(unsigned LayerIndex, unsigned Cmd, void * pData) 
{
  (void) LayerIndex;
  (void) pData;
  
  int r;

  switch (Cmd) 
  {
  
    /* 
     * Called during the initialization process in order to set up the
     * display controller and put it into operation. 
     */  
    case LCD_X_INITCONTROLLER: 
    {
     
      return 0;
    }
    
  
    default:
      r = -1;
  }
  return r;
}

/*************************** End of file ****************************/
