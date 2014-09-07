/**************************************************************************//**
 * @file digital_watch.c
 * @brief Draws a digital watch using emWin
 * @author Silicon Labs
 * @version 1.05
 * 
 * The following optimizations are done: 
 *   - The numbers 00-59 are prerendered and stored in a separate buffer
 *   - The date part is rendered at init and only redrawn whenever the date changes
 *   - When drawing a new frame the seconds/minutes/hours are copied from 
 *     the prerendered buffer in memory
 *   - Only the rows containing the time part is sent to display (unless the date 
 *     has changed)
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Labs has no
 * obligation to support this Software. Silicon Labs is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Silicon Labs will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 ******************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <time.h>
#include "GUI.h"
#include "em_device.h"
#include "memlcd.h"

/* Height in pixels of the used font */
#define FONT_HEIGHT 16

/* Display size */
#define BUFFER_VIRTUAL_WIDTH    160
#define DISPLAY_HEIGHT          128

#define FRAME_BUFFER_BYTES_PER_LINE 20

/* 1 byte per digit, 2 digits per number, 60 numbers */
#define NUMBER_BUFFER_BYTES_PER_LINE 120

/* Width of number buffer in pixels */
#define NUMBER_BUFFER_WIDTH (NUMBER_BUFFER_BYTES_PER_LINE * 8)


/* Position of the time string */
#define TIME_ROW        100
#define SECOND_COL      10
#define MINUTE_COL      7 
#define HOUR_COL        4

/* Frame buffer prototypes */
uint16_t *FB_getActiveBuffer(void);
void FB_activateFirstBuffer(void);
void FB_writeControlSignals(uint8_t *buffer);
uint8_t *FB_getFirstBuffer(void);
uint8_t *FB_getSecondBuffer(void);
void FB_clearBuffer(void);

/* Buffer holding the prerendered numbers 00-59 */
uint16_t numberBuffer[60 * FONT_HEIGHT];

/* Holds the last time that was shown on the display */
struct tm lastTime;


char *monthNames[] = {
  "January", 
  "February", 
  "March", 
  "April", 
  "May", 
  "June", 
  "July", 
  "August", 
  "September", 
  "October", 
  "November", 
  "December"
};

char *dayNames[] = {
  "Sunday",
  "Monday", 
  "Tuesday", 
  "Wednesday",
  "Thursday",
  "Friday",
  "Saturday"
};


/** This function wil draw a number from the prerendered
  * list at the given position. The drawing operation
  * is reduced to simple memory copy operations. */
static void drawNumber(int number, int line, int col)
{
  int i;
  
  /* Get start address of number. Width of each digit is 1 byte and 
   * each number is made of 2 digits */
  uint8_t *src = (uint8_t *)numberBuffer + number * 2;
  
  /* Get address in frame buffer where number should be drawn */
  uint8_t *dest = (uint8_t *)FB_getActiveBuffer() + FRAME_BUFFER_BYTES_PER_LINE * line + col;
  
  /* Copy pixels from number buffer to frame buffer */
  for ( i=0; i<FONT_HEIGHT; i++ ) 
  {
    /* Copy number pixels to frame buffer.
     * Width of each number (2 digits) is 16 pixels, or 2 bytes */
    *dest = *src;
    *(dest+1) = *(src+1);
    
    /* Go to next line */
    src += NUMBER_BUFFER_BYTES_PER_LINE;
    dest += FRAME_BUFFER_BYTES_PER_LINE;
  }
}




/** Tells emWin to draw to the number buffer */
static void activateNumberBuffer(void)
{
  LCD_SetVRAMAddrEx(0, (void *)numberBuffer);
}

/** Tell emWin the size of the number buffer */
static void setNumberBufferScreenSize(void)
{
  LCD_SetSizeEx (0, NUMBER_BUFFER_WIDTH, FONT_HEIGHT);
  LCD_SetVSizeEx(0, NUMBER_BUFFER_WIDTH, FONT_HEIGHT);
}

/** Tell emWin the size of the frame buffer */
static void setFrameBufferScreenSize(void) 
{
  LCD_SetSizeEx (0, BUFFER_VIRTUAL_WIDTH, DISPLAY_HEIGHT);
  LCD_SetVSizeEx(0, BUFFER_VIRTUAL_WIDTH, DISPLAY_HEIGHT); 
}


static void drawDate(struct tm *t)
{
  char day[15];
  char date[15];
  char year[15];
  
  /* Format the date */
  sprintf(day, "%s", dayNames[t->tm_wday]);
  sprintf(date, "%s %d", monthNames[t->tm_mon], t->tm_mday);
  sprintf(year, "%d", 1900 + t->tm_year);
  
  /* Draw to display */
  GUI_DispStringHCenterAt(day,  63, 30);
  GUI_DispStringHCenterAt(date, 63, 50);
  GUI_DispStringHCenterAt(year, 63, 70);  
}

/** Returns the first and last row touch by this frame update */
static RowLimits getDrawingLimits(struct tm *t)
{
  RowLimits limits;
  if ( t->tm_yday != lastTime.tm_yday ) {
    limits.min = 0;
    limits.max = 127;
  } else {
    limits.min = TIME_ROW;
    limits.max = TIME_ROW + FONT_HEIGHT;
  }
  return limits;
}

/** Use emWin to prerender all the necessary numbers
  * from 00-59 to a separate buffer. This will later
  * be used to speed up the process of drawing them 
  * to the display */
static void prerenderNumbers(void)
{
  int i;
  
  /* Tell emWin to draw to the prerender number buffer */
  activateNumberBuffer();
  
  /* Change the width of the window to match the buffer size */
  setNumberBufferScreenSize();
  
  /* Drawing each number 8 px wide and 16 px high */
  GUI_SetFont(&GUI_Font8x16);
  
  /* Draw all the numbers to the buffer */
  for ( i=0; i<60; i++ ) {
    GUI_DispDecAt(i, i*16, 0, 2);
  }
  
  /* Set the display width back to normal */
  setFrameBufferScreenSize();
  
  /* Tell emWin to draw to the normal frame buffer */
  FB_activateFirstBuffer(); 
}

void DIGITAL_drawBackground(struct tm *t)
{
  drawDate(t);
  
  /* Initialize the current time */
  drawNumber(t->tm_hour, TIME_ROW, HOUR_COL);
  drawNumber(t->tm_min,  TIME_ROW, MINUTE_COL);
  drawNumber(t->tm_sec,  TIME_ROW, SECOND_COL);
  
  GUI_DispStringAt(":",  6 * 8, TIME_ROW);
  GUI_DispStringAt(":",  9 * 8, TIME_ROW);
}



/** Draws a digital watch */
RowLimits DIGITAL_drawWatchFace(struct tm *t)
{
  RowLimits ret = getDrawingLimits(t);
  
  if ( lastTime.tm_yday != t->tm_yday ) {
    
    /* Date changed, update everything */
    FB_clearBuffer();
    DIGITAL_drawBackground(t);
  }
  
  /* Always draw seconds */
  drawNumber(t->tm_sec, TIME_ROW, SECOND_COL);
  
  /* Draw minutes whenever the number changes */
  if ( lastTime.tm_min != t->tm_min ) {
    drawNumber(t->tm_min, TIME_ROW, MINUTE_COL);
  }
  
  /* Draw hours whenever the number changes */
  if ( lastTime.tm_hour != t->tm_hour ) {
    drawNumber(t->tm_hour, TIME_ROW, HOUR_COL);
  }

  /* Save the rendered time */
  lastTime = *t;
  
  return ret;
}


/** Initialize the digital watch face */
void DIGITAL_init(void)
{  
  FB_activateFirstBuffer();
  
  memset(&lastTime, 0, sizeof(struct tm));
  
  prerenderNumbers();
}
