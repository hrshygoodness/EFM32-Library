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

#include <stdio.h>
#include "GUI.h"
#include "string.h"
#include "bsp.h"
#include "em_cmu.h"
#include "em_ebi.h"
#include "bsp_trace.h"
#include "touch.h"
#include "LCDConf.h"
#include "tftdirect.h"
#include "GUIDRV_Lin.h"

/* Application specific defines */
#include "display_conf.h"

/* Use triple buffering */
#define NUM_BUFFERS  3 

/* Initialization function prototype */
void initDisplayController(void);


/* 
 * Index of a pending (ready to be shown) frame. When no frame is 
 * pending this value will be -1 
 */
volatile int pendingFrameIndex = -1;



/* VSYNC interrupt handler. This performs the actual flipping 
 * of the buffers. 
 */
void EBI_IRQHandler(void) {
  uint32_t flags;
  
   flags = EBI_IntGet();
  EBI_IntClear(flags);
  
  if ( flags & EBI_IF_VSYNC ) {
    if ( pendingFrameIndex >= 0 ) {

      /* Set Direct Drive frame buffer address to the new frame */
      EBI_TFTFrameBaseSet(FRAME_BUFFER_SIZE * pendingFrameIndex);
    
      /* Send a confirmation to emWin that the old back buffer is visible now */
      GUI_MULTIBUF_Confirm(pendingFrameIndex);
      
      /* Reset the pending flag */
      pendingFrameIndex = -1;
    }
  }
}
 

/*********************************************************************
*
*       LCD_X_Config
*
* Purpose:
*   Called during the initialization process in order to set up the
*   display driver configuration.
*   
*/
void LCD_X_Config(void) {
  
  /* Enable triple buffering */
  //GUI_MULTIBUF_Config(NUM_BUFFERS);
 
  /* Set display driver and color conversion */
  GUI_DEVICE_CreateAndLink(GUIDRV_LIN_16, GUICC_M565, 0, 0);
  
  /* Display driver configuration, required for Lin-driver */
  if (LCD_GetSwapXY()) {
    LCD_SetSizeEx (0, LCD_HEIGHT, LCD_WIDTH);
    LCD_SetVSizeEx(0, LCD_HEIGHT, LCD_WIDTH);
  } else {
    LCD_SetSizeEx (0, LCD_WIDTH, LCD_HEIGHT);
    LCD_SetVSizeEx(0, LCD_WIDTH, LCD_HEIGHT);
  }
  
  /* Point video memory to start of PSRAM, drawing will start at frame 0 */
  LCD_SetVRAMAddrEx(0, (void *)VRAM_ADDR_START);
}

void initLcdDriver(void)
{
  initDisplayController();
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
  
  int r;

  switch (Cmd) {

  /* 
   * Called during the initialization process in order to set up the
   * display controller and put it into operation. 
   */ 
  case LCD_X_INITCONTROLLER: 
  {

    initDisplayController();
    return 0;
  }
  
  /* This command is received every time the GUI is done drawing a frame */
  case LCD_X_SHOWBUFFER:
  {
    /* Get the data object */
    LCD_X_SHOWBUFFER_INFO * p;
    p = (LCD_X_SHOWBUFFER_INFO *)pData;

    /* Save the pending buffer index. The interrupt routine will flip buffer on next VSYNC. */    
    pendingFrameIndex = p->Index;
    
  }
  default:
    r = -1;
  }
  return r;
}

/*************************** End of file ****************************/
