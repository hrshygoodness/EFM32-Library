/**************************************************************************//**
 * @file speexfunc.h
 * @brief Functions for Speex record and playback
 * @author Silicon Labs
 * @version 1.06
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 *
 ******************************************************************************/

#ifndef __SPEEXFUNC_H
#define __SPEEXFUNC_H

#include <stdbool.h>

struct speexEncFile
{
  char *frameStart;
  uint16_t frameNum;
};

#define ADC_OFFSET      0x8000
#define DAC_OFFSET      0x0800

void _speex_putc(int ch, void *file);
void _speex_fatal(const char *str, const char *file, int line);
void speexPlayBack(bool flashRecord, uint8_t freqBand, uint8_t encFrameSize, struct speexEncFile *ptrStruct);
void speexRecordFlash512(uint32_t complexity, uint32_t numOfFrames);

#endif
