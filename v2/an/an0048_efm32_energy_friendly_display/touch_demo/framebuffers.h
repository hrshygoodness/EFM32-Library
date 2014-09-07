/**************************************************************************//**
 * @file framebuffers.h
 * @brief Manages different frame buffers
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
#ifndef _FRAMEBUFFERS_H_
#define _FRAMEBUFFERS_H_

uint16_t *FB_getActiveBuffer(void);

uint8_t *FB_getBackgroundBuffer(void);

void FB_activateBackgroundBuffer(void);

void FB_activateFrameBuffer(void);

void FB_copyBackgroundToFrameBuffer(void);

void FB_copyPrevBuffer(void);

void FB_flipBuffer(void);

void FB_copyRenderBufferToDisplayBuffer(void);

void FB_clearBuffer(void);

void FB_writeControlSignals(uint8_t *buffer);

void FB_setCustomBuffer(uint16_t *addr);

void FB_disableCustomBuffer(void);

#endif
