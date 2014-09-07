/**************************************************************************//**
 * @file dma.h
 * @brief Setup DMA for ADC, DAC and FLASH write
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

#ifndef __DMA_H
#define __DMA_H

#include "config.h"

#define SRC_INC8_DST_INC8     0x00000000
#define SRC_FIX_DST_INC8      0x0c000000
#define SRC_INC8_DST_FIX      0xc0000000
#define SRC_FIX_DST_FIX_8     0xcc000000
#define SRC_INC16_DST_INC16   0x55000000
#define SRC_FIX_DST_INC16     0x5d000000
#define SRC_INC16_DST_FIX     0xd5000000
#define SRC_FIX_DST_FIX_16    0xdd000000
#define SRC_INC32_DST_INC32   0xaa000000
#define SRC_FIX_DST_INC32     0xae000000
#define SRC_INC32_DST_FIX     0xea000000
#define SRC_FIX_DST_FIX_32    0xee000000

#if SPEEX_BAND < NB8KCODEC
#define DMA_CHANNEL_NUMBER      1
#define DMA_ALT_CHANNEL_NUMBER  1
#define DMA_CHANNEL_DAC         0
#define DMA_CHANNEL_DACMASK     0x01
#else
#define DMA_CHANNEL_NUMBER      3
#define DMA_ALT_CHANNEL_NUMBER  2
#define DMA_CHANNEL_DAC         0
#define DMA_CHANNEL_ADC         1
#define DMA_CHANNEL_FLASH       2
#define DMA_CHANNEL_DACMASK     0x01
#define DMA_CHANNEL_ADCMASK     0x02
#define DMA_CHANNEL_FLHMASK     0x04
#endif

#define ADC_PP_START  SRC_FIX_DST_INC16 + ((SAMPLE_SIZE - 1) << _DMA_CTRL_N_MINUS_1_SHIFT) + DMA_CTRL_CYCLE_CTRL_PINGPONG;
#define FLASH_WR_START  SRC_INC32_DST_FIX + ((FLASH_WORD_SIZE - 2) << _DMA_CTRL_N_MINUS_1_SHIFT) + DMA_CTRL_CYCLE_CTRL_BASIC;

void initDma(void);
void initAdcDma(void);
void startAdcDma(void);
void stopAdcDma(void);
void initFlashDma(void);
void startFlashDma(void);
void initDacDma(void);
void startDacDma(uint16_t bufferSize);
void stopDacDma(void);

#endif
