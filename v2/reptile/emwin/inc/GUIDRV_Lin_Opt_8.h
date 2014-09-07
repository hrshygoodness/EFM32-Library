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
File        : GUIDRV_Lin_Opt_8.h
Purpose     : Optimized routines, included by GUIDRV_Lin_..._8.c
---------------------------END-OF-HEADER------------------------------
*/

/*********************************************************************
*
*       Static functions
*
**********************************************************************
*/
/*********************************************************************
*
*       _FillRectOpt8
*
* Purpose:
*   Optimized filling routine for 8 bpp
*/
static void _FillRectOpt8(GUI_DEVICE * pDevice, int x0, int y0, int x1, int y1) {
  DRIVER_CONTEXT * pContext;
  U32 Data, ColorMask, AndMask, Off, Off0, OffLine;
  int NumPixel_0, NumPixel_1, RemPixels, NumLines, RemLines, RemItems;
  LCD_PIXELINDEX ColorIndex;

  pContext   = (DRIVER_CONTEXT *)pDevice->u.pContext;
  ColorIndex = LCD__GetColorIndex();
  Off0       = XY2OFF32(pContext->vxSizePhys, x0, y0);
  RemPixels  = x1 - x0 + 1;
  NumLines   = y1 - y0 + 1;
  OffLine    = pContext->vxSizePhys >> 2;
  NumPixel_0 = x0 & 3;
  NumPixel_1 = x1 & 3;
  if (GUI_Context.DrawMode & LCD_DRAWMODE_XOR) {
    //
    // First DWORD
    //
    if (NumPixel_0) {
      for (RemLines = NumLines; RemLines; RemLines--) {
        Off     = Off0 + OffLine * (RemLines - 1);
        AndMask = ~(0xFFFFFFFF << (8 * NumPixel_0));
        if ((RemPixels < 3) && (NumPixel_1)) {
          AndMask |= ~(0xFFFFFFFF >> (8 * (3 - NumPixel_1)));
        }
        #if (LCD_ENDIAN_BIG == 1)
          MIRROR(AndMask);
        #endif
        Data = READ_MEM32(pContext->VRAMAddr, Off);
        Data ^= ~AndMask;
        WRITE_MEM32(pContext->VRAMAddr, Off, Data);
        Off++;
      }
      Off0++;
      RemPixels -= (4 - NumPixel_0);
    }
    //
    // Complete DWORDS
    //
    if (RemPixels >= 4) {
      for (RemLines = NumLines; RemLines; RemLines--) {
        RemItems = RemPixels;
        Off      = Off0 + OffLine * (RemLines - 1);
        while (RemItems >= 4) {
          Data  = READ_MEM32(pContext->VRAMAddr, Off);
          Data ^= 0xFFFFFFFF;
          LCD_WRITE_MEM32(pContext->VRAMAddr, Off, Data);
          Off++;
          RemItems -= 4;
        }
      }
      Off0       = Off;
      RemPixels -= (RemPixels >> 2) << 2;
    }
    //
    // Last DWORD
    //
    if (RemPixels > 0) {
      for (RemLines = NumLines; RemLines; RemLines--) {
        Off     = Off0 + OffLine * (RemLines - 1);
        AndMask = 0xFFFFFF00 << (8 * NumPixel_1);
        #if (LCD_ENDIAN_BIG == 1)
          MIRROR(AndMask);
        #endif
        Data = READ_MEM32(pContext->VRAMAddr, Off);
        Data ^= ~AndMask;
        WRITE_MEM32(pContext->VRAMAddr, Off, Data);
      }
    }
  } else {
    //
    // First DWORD
    //
    if (NumPixel_0) {
      for (RemLines = NumLines; RemLines; RemLines--) {
        Off     = Off0 + OffLine * (RemLines - 1);
        AndMask = ~(0xFFFFFFFF << (8 * NumPixel_0));
        if ((RemPixels < 3) && (NumPixel_1)) {
          AndMask |= ~(0xFFFFFFFF >> (8 * (3 - NumPixel_1)));
        }
        ColorMask = (ColorIndex * 0x01010101) & ~AndMask;
        #if (LCD_ENDIAN_BIG == 1)
          MIRROR(AndMask);
          MIRROR(ColorMask);
        #endif
        Data = READ_MEM32(pContext->VRAMAddr, Off);
        Data &= AndMask;
        Data |= ColorMask;
        WRITE_MEM32(pContext->VRAMAddr, Off, Data);
        Off++;
      }
      Off0++;
      RemPixels -= (4 - NumPixel_0);
    }
    //
    // Complete DWORDS
    //
    if (RemPixels >= 4) {
      ColorMask = ColorIndex * 0x01010101;
      for (RemLines = NumLines; RemLines; RemLines--) {
        RemItems = RemPixels;
        Off      = Off0 + OffLine * (RemLines - 1);
        while (RemItems >= 4) {
          LCD_WRITE_MEM32(pContext->VRAMAddr, Off, ColorMask);
          Off++;
          RemItems -= 4;
        }
      }
      Off0       = Off;
      RemPixels -= (RemPixels >> 2) << 2;
    }
    //
    // Last DWORD
    //
    if (RemPixels > 0) {
      for (RemLines = NumLines; RemLines; RemLines--) {
        Off     = Off0 + OffLine * (RemLines - 1);
        AndMask = 0xFFFFFF00 << (8 * NumPixel_1);
        ColorMask = (ColorIndex * 0x01010101) & ~AndMask;
        #if (LCD_ENDIAN_BIG == 1)
          MIRROR(AndMask);
          MIRROR(ColorMask);
        #endif
        Data = READ_MEM32(pContext->VRAMAddr, Off);
        Data &= AndMask;
        Data |= ColorMask;
        WRITE_MEM32(pContext->VRAMAddr, Off, Data);
      }
    }
  }
}

/*************************** End of file ****************************/
