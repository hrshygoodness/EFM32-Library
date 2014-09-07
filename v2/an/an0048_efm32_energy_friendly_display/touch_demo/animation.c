/**************************************************************************//**
 * @file animation.c
 * @brief Display Animation 
 * @author Silicon Labs
 * @version 1.05
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

#include <string.h>
#include "GUI.h"
#include "em_device.h"
#include "em_timer.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_msc.h"
#include "memlcd.h"
#include "animation.h"
#include "framebuffers.h"
#include "anim_timer.h"

#define NUM_FRAMES 60
#define FRAME_DELAY 15


extern GUI_CONST_STORAGE GUI_BITMAP bmdodeca1;
extern GUI_CONST_STORAGE GUI_BITMAP bmdodeca2;
extern GUI_CONST_STORAGE GUI_BITMAP bmdodeca3;
extern GUI_CONST_STORAGE GUI_BITMAP bmdodeca4;
extern GUI_CONST_STORAGE GUI_BITMAP bmdodeca5;
extern GUI_CONST_STORAGE GUI_BITMAP bmdodeca6;
extern GUI_CONST_STORAGE GUI_BITMAP bmdodeca7;
extern GUI_CONST_STORAGE GUI_BITMAP bmdodeca8;
extern GUI_CONST_STORAGE GUI_BITMAP bmdodeca9;
extern GUI_CONST_STORAGE GUI_BITMAP bmdodeca10;
extern GUI_CONST_STORAGE GUI_BITMAP bmdodeca11;
extern GUI_CONST_STORAGE GUI_BITMAP bmdodeca12;
extern GUI_CONST_STORAGE GUI_BITMAP bmdodeca13;
extern GUI_CONST_STORAGE GUI_BITMAP bmdodeca14;
extern GUI_CONST_STORAGE GUI_BITMAP bmdodeca15;
extern GUI_CONST_STORAGE GUI_BITMAP bmdodeca16;
extern GUI_CONST_STORAGE GUI_BITMAP bmdodeca17;
extern GUI_CONST_STORAGE GUI_BITMAP bmdodeca18;
extern GUI_CONST_STORAGE GUI_BITMAP bmdodeca19;
extern GUI_CONST_STORAGE GUI_BITMAP bmdodeca20;
extern GUI_CONST_STORAGE GUI_BITMAP bmdodeca21;
extern GUI_CONST_STORAGE GUI_BITMAP bmdodeca22;
extern GUI_CONST_STORAGE GUI_BITMAP bmdodeca23;
extern GUI_CONST_STORAGE GUI_BITMAP bmdodeca24;
extern GUI_CONST_STORAGE GUI_BITMAP bmdodeca25;
extern GUI_CONST_STORAGE GUI_BITMAP bmdodeca26;
extern GUI_CONST_STORAGE GUI_BITMAP bmdodeca27;
extern GUI_CONST_STORAGE GUI_BITMAP bmdodeca28;
extern GUI_CONST_STORAGE GUI_BITMAP bmdodeca29;
extern GUI_CONST_STORAGE GUI_BITMAP bmdodeca30;
extern GUI_CONST_STORAGE GUI_BITMAP bmdodeca31;
extern GUI_CONST_STORAGE GUI_BITMAP bmdodeca32;
extern GUI_CONST_STORAGE GUI_BITMAP bmdodeca33;
extern GUI_CONST_STORAGE GUI_BITMAP bmdodeca34;
extern GUI_CONST_STORAGE GUI_BITMAP bmdodeca35;
extern GUI_CONST_STORAGE GUI_BITMAP bmdodeca36;
extern GUI_CONST_STORAGE GUI_BITMAP bmdodeca37;
extern GUI_CONST_STORAGE GUI_BITMAP bmdodeca38;
extern GUI_CONST_STORAGE GUI_BITMAP bmdodeca39;
extern GUI_CONST_STORAGE GUI_BITMAP bmdodeca40;
extern GUI_CONST_STORAGE GUI_BITMAP bmdodeca41;
extern GUI_CONST_STORAGE GUI_BITMAP bmdodeca42;
extern GUI_CONST_STORAGE GUI_BITMAP bmdodeca43;
extern GUI_CONST_STORAGE GUI_BITMAP bmdodeca44;
extern GUI_CONST_STORAGE GUI_BITMAP bmdodeca45;
extern GUI_CONST_STORAGE GUI_BITMAP bmdodeca46;
extern GUI_CONST_STORAGE GUI_BITMAP bmdodeca47;
extern GUI_CONST_STORAGE GUI_BITMAP bmdodeca48;
extern GUI_CONST_STORAGE GUI_BITMAP bmdodeca49;
extern GUI_CONST_STORAGE GUI_BITMAP bmdodeca50;
extern GUI_CONST_STORAGE GUI_BITMAP bmdodeca51;
extern GUI_CONST_STORAGE GUI_BITMAP bmdodeca52;
extern GUI_CONST_STORAGE GUI_BITMAP bmdodeca53;
extern GUI_CONST_STORAGE GUI_BITMAP bmdodeca54;
extern GUI_CONST_STORAGE GUI_BITMAP bmdodeca55;
extern GUI_CONST_STORAGE GUI_BITMAP bmdodeca56;
extern GUI_CONST_STORAGE GUI_BITMAP bmdodeca57;
extern GUI_CONST_STORAGE GUI_BITMAP bmdodeca58;
extern GUI_CONST_STORAGE GUI_BITMAP bmdodeca59;
extern GUI_CONST_STORAGE GUI_BITMAP bmdodeca60;




typedef GUI_CONST_STORAGE GUI_BITMAP *pBitmap;

pBitmap frames[] = {&bmdodeca1,
&bmdodeca2,
&bmdodeca3,
&bmdodeca4,
&bmdodeca5,
&bmdodeca6,
&bmdodeca7,
&bmdodeca8,
&bmdodeca9,
&bmdodeca10,
&bmdodeca11,
&bmdodeca12,
&bmdodeca13,
&bmdodeca14,
&bmdodeca15,
&bmdodeca16,
&bmdodeca17,
&bmdodeca18,
&bmdodeca19,
&bmdodeca20,
&bmdodeca21,
&bmdodeca22,
&bmdodeca23,
&bmdodeca24,
&bmdodeca25,
&bmdodeca26,
&bmdodeca27,
&bmdodeca28,
&bmdodeca29,
&bmdodeca30,
&bmdodeca31,
&bmdodeca32,
&bmdodeca33,
&bmdodeca34,
&bmdodeca35,
&bmdodeca36,
&bmdodeca37,
&bmdodeca38,
&bmdodeca39,
&bmdodeca40,
&bmdodeca41,
&bmdodeca42,
&bmdodeca43,
&bmdodeca44,
&bmdodeca45,
&bmdodeca46,
&bmdodeca47,
&bmdodeca48,
&bmdodeca49,
&bmdodeca50,
&bmdodeca51,
&bmdodeca52,
&bmdodeca53,
&bmdodeca54,
&bmdodeca55,
&bmdodeca56,
&bmdodeca57,
&bmdodeca58,
&bmdodeca59,
&bmdodeca60};

/* Index to the current frame */
volatile uint32_t curFrame = 0;

/* Which lines to send to display */
static DrawLimits drawLimits = {0, 127};

#define FRAMES_START 0x7D000

#define FRAME_BUFFER_SIZE_BYTES (160 * 128 / 8)

#define FRAME_BUFFER_SIZE_WORDS (160 * 128 / 32)




void ANIM_prerenderFrames(void)
{
  uint32_t i;
  uint32_t magicWord = 0xef32;
  
  /* Get the starting address of the prerendered frames */
  uint32_t *addr = (uint32_t *)FRAMES_START;
  
  /* Get the end of the prerendered frames */
  uint32_t *lastAddr = addr + FRAME_BUFFER_SIZE_WORDS * NUM_FRAMES;
  
  /* Get the address of the current frame buffer */
  uint16_t *frameBuffer = FB_getActiveBuffer();
  
  /* Check the last address for the magic word. 
   * If this has been written the frames are already in flash 
   * and we can skip the entire prerendering step */
  if ( *lastAddr == magicWord ) {
    return;
  }
  
  MSC_Init();
  
  /* Make emWin draw to the frame buffer */
  FB_activateFrameBuffer();
  
  /* Add meta data to frame buffer */
  FB_writeControlSignals((uint8_t*)frameBuffer);
  
  /* Erase all the pages we are going to write to */
  while ( addr < lastAddr ) {
    MSC_ErasePage(addr);
    addr += FRAME_BUFFER_SIZE_WORDS;
  }
  
  addr = (uint32_t *)FRAMES_START;
    
  for ( i=0; i<NUM_FRAMES; i++ ) {
    
    /* Render a frame */
    GUI_DrawBitmap(frames[i], 0, 0);
    
    /* Copy entire frame buffer to flash */
    MSC_WriteWord(addr, frameBuffer, FRAME_BUFFER_SIZE_BYTES);
    addr += FRAME_BUFFER_SIZE_WORDS;
  }
  
  /* Write magic word to indicate that the frames are ready in Flash */
  MSC_WriteWord(lastAddr, &magicWord, 4);
  
  MSC_Deinit();
}


void ANIM_preview(void)
{
  FB_clearBuffer();
  GUI_DrawBitmap(frames[0], 0, 0);
}

void ANIM_prepare(void)
{ 
  startAnimTimer(30);
  curFrame = 0;
}

void ANIM_draw(void)
{    
  uint16_t *frame = (uint16_t*)FRAMES_START + curFrame * 1280;
  
  FB_setCustomBuffer(frame);
  
  if ( ++curFrame >= NUM_FRAMES ) {
    curFrame = 0;
  }
}

void ANIM_finish(void)
{
  stopAnimTimer();
  FB_disableCustomBuffer();
}
  
DrawLimits ANIM_getLimits(void)
{
  return drawLimits;
}
