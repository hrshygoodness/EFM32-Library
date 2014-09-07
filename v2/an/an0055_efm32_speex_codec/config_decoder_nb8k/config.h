/***************************************************************************//**
 * @file config.h
 * @brief SPEEX codec configuration
 * @author Silicon Labs
 * @version 1.06
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 *
 ******************************************************************************/

#ifndef __CONFIG_H
#define __CONFIG_H

/* Giant Gecko NB 8K decoder configuration file */

#ifdef __IAR_SYSTEMS_ICC__     /* IAR Compiler */
#define inline inline
#endif

#ifdef  __CC_ARM               /* ARM Compiler */
#define inline __inline
#endif

#define EXPORT 

#define FIXED_POINT
#define DISABLE_WIDEBAND
#define DISABLE_FLOAT_API
#define DISABLE_VBR
#define DISABLE_NOTIFICATIONS
#define DISABLE_WARNINGS
#define RELEASE
#define OVERRIDE_SPEEX_PUTC
#define OVERRIDE_SPEEX_FATAL
#define MAX_CHARS_PER_FRAME (20/BYTES_PER_CHAR)

#define NB_DEC_STACK    1000

#define SAMPLE_SIZE     160
#define FRAME_SIZE_8K   20

#define NARROWBAND8K    0
#define NARROWBAND      1
#define WIDEBAND        2
#define ULTRAWIDEBAND   3
#define NB8KCODEC       4

#define SPEEX_BAND      NARROWBAND8K

/* Address for DMA primary and alternate channel control data structure */
#define PRI_ADDRESS     0x20001000
#define ALT_ADDRESS     0x20001100

/* HFXO crystal frequency */
#define SPEEX_HFXO_FREQ 48000000

/* Define play list size */
#define LIST_SIZE       10

/* Configure DAC channel */
#define DAC_CHANNEL     0
#define DAC_CHDATA      CH0DATA
#define DAC_CHDMA       DMAREQ_DAC0_CH0
#define DAC_CHCTRL      CH0CTRL
#define DAC_CHCTRLEN    DAC_CH0CTRL_EN
#define DAC_CHPRSEN     DAC_CH0CTRL_PRSEN

#define DAC_PRS_CH      0
#define DAC_PRS_SEL     dacPRSSELCh0
#define DAC_CLOCK       1000000
#define REF_SELECT      0

/* Configure Timer */
#define TIMER_USED      TIMER1
#define TIMER_CLK       CMU_HFPERCLKEN0_TIMER1
#define TIMER_PRS       PRS_CH_CTRL_SOURCESEL_TIMER1
#define TIMER_SRC       PRS_CH_CTRL_SIGSEL_TIMER1OF

#define SAMPLE_8K       999
#define BASE_FREQ       8000000

extern void _speex_fatal(const char *str, const char *file, int line);
extern void _speex_putc(int ch, void *file);

#endif
