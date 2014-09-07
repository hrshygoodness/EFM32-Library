/**************************************************************************//**
 * @file analog_watch.c
 * @brief Draws an analog watch using emWin
 * @author Silicon Labs
 * @version 1.05
 * 
 * The following optimizations are done: 
 *   - The background image is only rendered once and stored in a separate buffer.
 *   - In initialization each of the 60 positions for each pointer is calculated
 *     and the polygons are rotated and stored in RAM. 
 *   - Also the first and last row which are changed is calculated for each position
 *   - When drawing a frame the background image is copied and the rotated
 *     polygons are found and drawn on top
 *   - Only the changed rows are sent to display
 *   
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

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <stdio.h>
#include <time.h>
#include "GUI.h"
#include "em_device.h"
#include "em_chip.h"
#include "em_rtc.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "memlcd.h"
#include "analog_watch.h"
#include "framebuffers.h"
#include "clock.h"
#include "anim_timer.h"


/* Screen center */
#define CENTER_X      64
#define CENTER_Y      64

/* Length of the 'second pointer' */
#define SEC_START 10
#define SEC_END   35

/* Distance screen center to center of number label */
#define LABEL_LENGTH  55

#define PI 3.14159265358979

#define HOUR_POLYGON_SIZE   3 
#define MINUTE_POLYGON_SIZE 3
#define SECOND_POLYGON_SIZE 2
   
   
extern volatile bool spiTransferActive;


/* Clip rectangle for emWin. Since the frame buffer is wider than the display 
 * itself to accommodate control signals we need to clip the display 
 * so that emWin will not overwrite the control signals */
static GUI_RECT clipRect = {0, 0, 127, 127};

static uint32_t frameCounter = 0;

static DrawLimits drawLimits;
   

/* Drawing limits. Indicates which rows have been changed when 
 * drawing a pointer at the given position. These limits are used
 * to determine which lines actually has to be transferred to the 
 * display for each frame. */
static DrawLimits secondLimits[60];
static DrawLimits minuteLimits[60];
static DrawLimits hourLimits[60];

/* Polygon for the 'hour pointer' */
static const GUI_POINT shortArrow[] = {
  {-4, -5},
  {0, -30},
  {4, -5}
};


/* Polygon for the 'minute pointer' */
static const GUI_POINT longArrow[] = {
  {-4, -5},
  {0, -45},
  {4, -5}
};

/* Precalculated pointer polygons. One for each 
 * possible position. The 'second pointer' is simply a line. */
static GUI_POINT shortArrows[60][HOUR_POLYGON_SIZE];
static GUI_POINT longArrows[60][MINUTE_POLYGON_SIZE];
static GUI_POINT secondArrows[60][SECOND_POLYGON_SIZE];


/** Generate pointers for all 60 positions around the watch. 
  * This is done once at initialization to minimize 
  * calculation time each second */
static void generatePointers(void)
{
  int i;
  double a;
  
  for ( i=0; i<60; i++ ) {
    
    /* Get angle */
    a = (double)i / 30.0 * PI;
    
    /* Second pointer is just a line. Calculate start and end point. */
    secondArrows[i][0].x = CENTER_X + (int)(SEC_START * sin(a));
    secondArrows[i][0].y = CENTER_X - (int)(SEC_START * cos(a));
    
    secondArrows[i][1].x = CENTER_X + (int)(SEC_END * sin(a));
    secondArrows[i][1].y = CENTER_Y - (int)(SEC_END * cos(a));
    
    /* Rotate the polygons */
    GUI_RotatePolygon(longArrows[i],  longArrow,  MINUTE_POLYGON_SIZE, -a);
    GUI_RotatePolygon(shortArrows[i], shortArrow, HOUR_POLYGON_SIZE, -a);
  }
}

/* Draws the background watch face (number plate) */
void ANALOG_drawBackground(void)
{
  int i,x,y;
  double a;
  char hourLabel[3];
  
  /* Clear the screen */
  GUI_SetColor(GUI_BLACK);
  GUI_FillRect(0,0,128,128);
  GUI_SetColor(GUI_WHITE);
  
  /* Draw a circle at center */
  GUI_FillCircle(CENTER_X,CENTER_Y,2);
  
  /* Loop through all the 12 hour labels */
  for ( i=1; i<=12; i++ ) {
    
    /* Get angle */
    a = (double)i / 6.0 * PI;
    
    /* Calculate position for the label */
    x = CENTER_X - 8 + (int)(LABEL_LENGTH * sin(a));
    y = CENTER_Y - 8 + (int)(LABEL_LENGTH * -cos(a));
  
    /* Print number */
    sprintf(hourLabel, "%02d", i);
    GUI_DispStringAt(hourLabel, x, y);
  }  
}

static void drawSecondArrow(int second)
{
  GUI_DrawLine(secondArrows[second][0].x, secondArrows[second][0].y, secondArrows[second][1].x, secondArrows[second][1].y);
}

static void drawHourArrow(int hour, int minute)
{
  GUI_FillPolygon(shortArrows[hour * 5 + minute / 12], HOUR_POLYGON_SIZE, CENTER_X, CENTER_Y);   
}

static void drawMinuteArrow(int minute)
{
  GUI_FillPolygon(longArrows[minute], MINUTE_POLYGON_SIZE, CENTER_X, CENTER_Y);  
}



static DrawLimits generateSecondLimit(int second)
{
  int min,max, i;
  DrawLimits ret = DRAWLIMITS_DEFAULT;
  
  /* Set min/max to extreme values */
  min = 200;
  max = -200; 
  
  /* Find highest/lowest point in polygon */
  for ( i=0; i<SECOND_POLYGON_SIZE; i++ ) {
    int prev = (second == 0 ? 59 : second-1);
    
    if ( secondArrows[second][i].y < min ) {
      min = secondArrows[second][i].y;
    }
    if ( secondArrows[prev][i].y < min ) {
      min = secondArrows[prev][i].y < min;
    }
    
    if ( secondArrows[second][i].y > max ) {
      max = secondArrows[second][i].y;
    }
    if ( secondArrows[prev][i].y > max ) {
      max = secondArrows[prev][i].y;
    }
    
  }
  
  ret.max = max;
  ret.min = min;
  
  return ret;
}


/** Calculates drawing limits (first and last row) when 
  * drawing a minute pointer at the given position */
static DrawLimits generateMinuteLimit(int minute)
{
  int min,max,i;
  DrawLimits ret = DRAWLIMITS_DEFAULT;
  
  /* Set min/max to extreme values */
  min = 200;
  max = -200; 
  
  /* Find highest/lowest point in polygon */
  for ( i=0; i<MINUTE_POLYGON_SIZE; i++ ) {
    int prev = (minute == 0 ? 59 : minute-1);
    
    if ( longArrows[minute][i].y < min ) {
      min = longArrows[minute][i].y;
    }
    if ( longArrows[prev][i].y < min ) {
      min = longArrows[prev][i].y;
    }
    
    if ( longArrows[minute][i].y > max ) {
      max = longArrows[minute][i].y;
    }
    if ( longArrows[prev][i].y > max ) {
      max = longArrows[prev][i].y;
    }
  }
  
  ret.max = CENTER_Y + max;
  ret.min = CENTER_Y + min;
  
  return ret;
}

/** Calculates drawing limits (first and last row) when 
  * drawing a hour pointer at the given position */
static DrawLimits generateHourLimit(int hour)
{
  int min,max,i;
  DrawLimits ret = DRAWLIMITS_DEFAULT;
    
  /* Set min/max to extreme values */
  min = 200;
  max = -200; 
  
  /* Find highest/lowest point in polygon */
  for ( i=0; i<HOUR_POLYGON_SIZE; i++ ) {
     int prev = (hour == 0 ? 59 : hour-1);
    
    if ( shortArrows[hour][i].y < min ) {
      min = shortArrows[hour][i].y;
    }
    if ( shortArrows[prev][i].y < min ) {
      min = shortArrows[prev][i].y;
    }
    
    if ( shortArrows[hour][i].y > max ) {
      max = shortArrows[hour][i].y;
    }
    if ( shortArrows[prev][i].y > max ) {
      max = shortArrows[prev][i].y;
    }
  }
  
  ret.max = CENTER_Y + max;
  ret.min = CENTER_Y + min;
  
  return ret;
}



/* Calculates drawing limits (first and last row) when 
 * drawing pointers. This help energy consumption by only
 * updating the rows that were changed */
static void generateLimits(void)
{
  int i;
  for ( i=0; i<60; i++ ) {
    secondLimits[i] = generateSecondLimit(i);
    minuteLimits[i] = generateMinuteLimit(i);
    hourLimits[i] = generateHourLimit(i);
  }
}


/* Returns drawing limits (first and last row) when 
 * drawing the given time. This helps energy consumption by only
 * updating the rows that were changed */
static DrawLimits getDrawingLimits(struct tm *t)
{
  /* The hour pointer depends on both hour and minute variables */
  int hourIndex = (t->tm_hour%12) * 5 + t->tm_min % 5;
  
  /* For seconds 1-59, only the second pointer changes */
  DrawLimits ret = secondLimits[t->tm_sec];  
  if ( t->tm_sec != 0 ) {
    return ret;
  } 
  
  /* At second == 0, all pointers change. Find the total min/max */
  
  if ( minuteLimits[t->tm_min].min < ret.min ) {
    ret.min = minuteLimits[t->tm_min].min;
  }
  
  if ( hourLimits[hourIndex].min < ret.min ) {
    ret.min = hourLimits[hourIndex].min;
  }
  
  if ( minuteLimits[t->tm_min].max > ret.max ) {
    ret.max = minuteLimits[t->tm_min].max;
  }
  
  if ( hourLimits[hourIndex].max > ret.max ) {
    ret.max = hourLimits[hourIndex].max;
  }
   
  return ret; 
}


/* Initialize the frame buffer for drawing an 
 * analog watch. First write SPI control signals (line addresses)
 * to the frame buffer, then draw background in a separate buffer
 * and copy the background to the frame buffer. 
 * Also precalculate and render the position for each pointer
 * along with the minimum and maximum rows touched by each pointer */
void ANALOG_init(void)
{   
  /* Write display control signals to background buffer */
  FB_writeControlSignals(FB_getBackgroundBuffer());

  /* Tell emWin to draw to the background buffer */
  FB_activateBackgroundBuffer();  
  
  /* Draw background image (number plate) */
  ANALOG_drawBackground();
  
  /* Copy background (including control signals) to frame buffer */
  FB_copyBackgroundToFrameBuffer();
  
  /* Tell emWin to draw to frame buffer again */
  FB_activateFrameBuffer();
  
  /* Prerender pointer arrows */
  generatePointers();
  
  /* Generate row limits for drawing each frame */
  generateLimits();
}

/* Draw the pointers for the analog watch face at the given time */
static void drawWatchFace(struct tm *t)
{ 
  GUI_SetColor(GUI_WHITE);
    
  drawHourArrow(t->tm_hour % 12, t->tm_min);
  drawMinuteArrow(t->tm_min);
  drawSecondArrow(t->tm_sec);
}


void ANALOG_preview(void)
{
  struct tm *time = getTimeMinusOne();
  
  ANALOG_init();
  
  GUI_SetClipRect(&clipRect); 

  drawWatchFace(time);
}


void ANALOG_prepare(void) 
{
  struct tm *time = getTimeMinusOne();
  
  GUI_SetClipRect(&clipRect); 
  
  ANALOG_init();
  
  drawWatchFace(time);
  
  frameCounter = 0;
  
  startAnimTimer(1000);
}


/** Shows an analog watch on the display */
void ANALOG_draw(void) 
{
  struct tm *time;
    
  FB_copyBackgroundToFrameBuffer();

  /* Increase time with 1s */
  time = getTime();
  
  /* Draw watch face to frame buffer */
  drawWatchFace(time);
  
  /* Only update the part of display that was changed */
  if ( frameCounter == 0 ) {
    drawLimits.min = 0;
    drawLimits.max = 127;
  } else {
    drawLimits = getDrawingLimits(time);
  }

  frameCounter++;
}

void ANALOG_finish(void)
{
  stopAnimTimer();
}

DrawLimits ANALOG_getLimits(void)
{
  return drawLimits;
}
