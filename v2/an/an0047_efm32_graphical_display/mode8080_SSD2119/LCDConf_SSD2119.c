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

** emWin V5.16 - Graphical user interface for embedded applications **
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

#include "GUI.h"

#include "GUIDRV_Lin.h"
#include "bsp.h"
#include "em_cmu.h"
#include "em_ebi.h"
#include "em_rtc.h"
#include "bsp_trace.h"
#include "touch.h"
#include "LCDConf.h"
#include "ssd2119.h"

#define XSIZE_PHYS 320
#define YSIZE_PHYS 240

void LCD_InitializeLCD(void);


extern const GUI_DEVICE_API GUIDRV_SSD2119;
static bool runOnce = true;
static GUI_DEVICE *guiDevice;




/* Reset the TFT controller and enable access to it */
void initDisplayController(void) {
  int i;
  CMU_ClockEnable( cmuClock_GPIO, true);
  
  /* If we are in BC_UIF_AEM_EFM state, we can redraw graphics */
  if (BSP_RegisterRead(&BC_REGISTER->UIF_AEM) == BC_UIF_AEM_EFM)
  {
    /* If we're not BC_ARB_CTRL_EBI state, we need to reconfigure display controller */
    if ((BSP_RegisterRead(&BC_REGISTER->ARB_CTRL) != BC_ARB_CTRL_EBI) || runOnce)
    {
      /* Configure for EBI mode and reset display */
      BSP_DisplayControl(BSP_Display_EBI);
      BSP_DisplayControl(BSP_Display_ResetAssert);
      BSP_DisplayControl(BSP_Display_PowerDisable);
      /* Short delay */
      for (i = 0; i < 10000; i++) ;
      /* Configure display for Direct Drive + SPI mode */
      BSP_DisplayControl(BSP_Display_Mode8080);
      BSP_DisplayControl(BSP_Display_PowerEnable);
      BSP_DisplayControl(BSP_Display_ResetRelease);

      runOnce = false;
    }
  }
}

 
/* Initialize LCD driver. This function is called during initialization and when exiting from AEM mode */
void initLcdDriver(void)
{ 
  int (* pInit) (GUI_DEVICE *);
  pInit = (int (*)(GUI_DEVICE *)) guiDevice->pDeviceAPI->pfGetDevFunc(&guiDevice, LCD_DEVFUNC_INIT);
  pInit(guiDevice);
}



/* Called during the initialization process in order to set up the
 * display driver configuration. See the emWin documentation for details */
void LCD_X_Config(void) {
  
  guiDevice = GUI_DEVICE_CreateAndLink(&GUIDRV_SSD2119, GUICC_M565, 0, 0);

  if (LCD_GetSwapXY()) {
    LCD_SetSizeEx (0, YSIZE_PHYS, XSIZE_PHYS);
    LCD_SetVSizeEx(0, YSIZE_PHYS, XSIZE_PHYS);
  } else {
    LCD_SetSizeEx (0, XSIZE_PHYS, YSIZE_PHYS);
    LCD_SetVSizeEx(0, XSIZE_PHYS, YSIZE_PHYS);
  }

  if(guiDevice)
  { 
    void (* pSetControllerAddress) (GUI_DEVICE *, uint32_t reg, uint32_t data);
    
    /* now let's take function for setting controller address */
    pSetControllerAddress = (void (*) (GUI_DEVICE *, uint32_t, uint32_t))guiDevice->pDeviceAPI->pfGetDevFunc(&guiDevice, LCD_DEVFUNC_CONTRADDR);
    
    /* and configure addresses - for writing register and data */
    pSetControllerAddress(guiDevice, BC_SSD2119_BASE, BC_SSD2119_BASE + 2);
  }

}

/* Called by emWin for various driver tasks. See the emWin documentation for details. */
int LCD_X_DisplayDriver(unsigned LayerIndex, unsigned Cmd, void * pData) 
{
  (void) LayerIndex;
  (void) pData;
  
  switch (Cmd) {
    case LCD_X_INITCONTROLLER:
      initDisplayController();
      return 0;
    case LCD_X_ON:
      return 0;
    case LCD_X_OFF:
      return 0;
    default:
      return -1;
  }
}

