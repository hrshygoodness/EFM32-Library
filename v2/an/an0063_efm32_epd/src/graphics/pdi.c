/*********************************************************************
*                SEGGER Microcontroller GmbH & Co. KG                *
*        Solutions for real time microcontroller applications        *
*                           www.segger.com                           *
**********************************************************************
*                                                                    *
* C-file generated by                                                *
*                                                                    *
*        Bitmap converter for emWin V5.16.                           *
*        Compiled Jun  4 2012, 15:48:30                              *
*        (C) 1998 - 2010 Segger Microcontroller GmbH & Co. KG        *
*                                                                    *
**********************************************************************
*                                                                    *
* Source file: pdi                                                   *
* Dimensions:  150 * 24                                              *
* NumColors:   2                                                     *
*                                                                    *
**********************************************************************
*/

#include <stdlib.h>

#include "GUI.h"

#ifndef GUI_CONST_STORAGE
  #define GUI_CONST_STORAGE const
#endif

/*   Palette
The following are the entries of the palette table.
Every entry is a 32-bit value (of which 24 bits are actually used)
the lower   8 bits represent the Red component,
the middle  8 bits represent the Green component,
the highest 8 bits (of the 24 bits used) represent the Blue component
as follows:   0xBBGGRR
*/

static GUI_CONST_STORAGE GUI_COLOR Colorspdi[] = {
     0xFFFFFF,0x000000
};

static GUI_CONST_STORAGE GUI_LOGPALETTE Palpdi = {
  2,	/* number of entries */
  0, 	/* No transparency */
  &Colorspdi[0]
};

static GUI_CONST_STORAGE unsigned char acpdi[] = {
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXX__,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, ________, ________, ________, __XXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXX__,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XX______, ________, ________, ________, ______XX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXX__,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, X___XXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXX___X, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXX__,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, __XXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXX__, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXX__,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXX__,
  XX_____X, XXX_____, XXX_____, _XXX__XX, XXXXXXXX, X_XXXXXX, X____XXX, X_XX___X, XXXXX_XX, ______XX, XX______, _XXXXXXX, XXXX_X__, XXX_____, _XX__XXX, XXXXXXXX, _XXX__XX, XXXX_XXX, ____XX__,
  X___XX__, _X___XXX, XXX__XX_, __XX___X, XXXX_XXX, __XXXXXX, __XX_XXX, __XXX__X, XXXXX_XX, __XXXXXX, XX_XXXXX, X__XXX__, XXX_XXX_, XXX_XXXX, __X__XXX, XXXXXXX_, _XXXX__X, XXX__XX_, XXX_XX__,
  X___XX__, _X___XXX, XXX__XXX, ___X___X, XXXX_XXX, ___XXXX_, _XXXXXXX, __XXX___, XXXX_XXX, __XXXXXX, XX_XXXXX, XX__XX__, XX__XXXX, XXX_XXXX, __XX_XXX, XXXXXXX_, __XXX__X, XXX_XX__, XXXXXX__,
  XX__XX__, _X___XXX, XXX__XXX, __XXX__X, XXX_XXX_, ___XXXX_, __XXXXXX, __XXX___, XXXX_XXX, __XXXXXX, XX_XXXXX, XX__XX__, XX__XXXX, XXX_XXXX, __XX_XXX, XXXXXX_X, __XXXX__, XX_XXX__, XXXXXX__,
  XX__XX__, _X___XXX, XXX__XXX, __XXX___, XXX_XXX_, X___XXXX, ___XXXXX, __XXXX__, XXX_XXXX, __XXXXXX, XX_XXXXX, XXX_XX__, XXX__XXX, XXX_XXXX, __XX_XXX, XXXXXX_X, X_XXXX__, XX_XXX__, _XXXXX__,
  XX__X___, XX______, _XX___X_, _XXXXX__, XXX_XXX_, XX__XXXX, _____XXX, __XXXX__, _XX_XXXX, ______XX, XX_XXXXX, XXX_XX__, XXX____X, XXX_XX__, _XXX_XXX, XXXXX_XX, X__XXXX_, __XXXXX_, ___XXX__,
  XX__XXXX, XX___XXX, XXX__X__, XXXXXX__, XX_XXX_X, XX__XXXX, X_____XX, __XXXXX_, _XX_XXXX, __XXXXXX, XX_XXXXX, XXX_XX__, XXXXXX__, XXX_XXXX, XXXX_XXX, XXXXX___, ___XXXXX, __XXXXXX, X___XX__,
  XX__XXXX, XX___XXX, XXX__X__, _XXXXX__, _X_XXX__, _____XXX, XXX___XX, __XXXXX_, _X_XXXXX, __XXXXXX, XX_XXXXX, XX__XX__, XXXXXXX_, _XX_XXXX, XXXX_XXX, XXXX__XX, XX__XXXX, _XXXXXXX, XXX__X__,
  X___XXXX, XX___XXX, XXX__XX_, _XXXXXX_, __XXX_XX, XXX__XXX, XXXX__XX, __XXXXX_, ___XXXXX, __XXXXXX, XX_XXXXX, XX__XX__, XXXXXXX_, _XX_XXXX, XXXX_XXX, XXXX_XXX, XX__XXXX, _XXXXXXX, XXX__X__,
  X___XXXX, XX___XXX, XXX__XX_, __XXXXX_, __XXX_XX, XXX___XX, XXXX__XX, __XXXXXX, ___XXXXX, __XXXXXX, XX_XXXXX, X__XXX__, XXXXXXX_, XXX_XXXX, XXXX_XXX, XXXX_XXX, XXX__XXX, __XXXX_X, XXX__X__,
  X___XXXX, XX______, _XX__XXX, ___XXXX_, __XX__XX, XXX___X_, _XX__XXX, __XXXXXX, __XXXXXX, ______XX, XX_XXXXX, __XXXX__, XX_XXX__, XXX_XXXX, XXX__XXX, XXX__XXX, XXX__XXX, __XXXX__, XX__XX__,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, X__XXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXX__X, XXXXXXXX, XXX___XX, XXXXXXXX, XXXX____, _XX_XXXX, XXXX_XXX, _XXXXXX_, __XXXX__,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXX__,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, ___XXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXX___, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXX__,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, X_______, ________, ________, ________, _______X, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXX__,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XX______, ________, ________, ________, ______XX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXX__,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXX__,
  XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXX__
};

GUI_CONST_STORAGE GUI_BITMAP bmpdi = {
  150, /* XSize */
  24, /* YSize */
  19, /* BytesPerLine */
  1, /* BitsPerPixel */
  acpdi,  /* Pointer to picture data (indices) */
  &Palpdi,   /* Pointer to palette */
  NULL
};

/* *** End of file *** */