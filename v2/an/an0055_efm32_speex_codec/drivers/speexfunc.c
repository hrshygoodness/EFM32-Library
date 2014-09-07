/**************************************************************************//**
 * @file speexfunc.c
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

#include <stdlib.h>

#include "config.h"
#include <speex/speex.h>
#include "speexfunc.h"

#include "em_device.h"
#include "em_emu.h"
#include "dma.h"
#include "flash.h"

/* Speex bit-packing struct */
SpeexBits bits;
/* Speex decoder state */
void *dec_state;

#if defined (__ICCARM__)
#pragma data_alignment=4
volatile int16_t sampleBuffer[2][SAMPLE_SIZE];
#elif defined (__CC_ARM)
volatile int16_t sampleBuffer[2][SAMPLE_SIZE] __attribute__((aligned(4)));
#else
#error Undefined toolkit, need to define alignment 
#endif

#if SPEEX_BAND == NB8KCODEC
/* Speex encoder state */
void *enc_state;

#if defined (__ICCARM__)
#pragma data_alignment=4
volatile uint8_t encodeFrameBuffer[ENCODE_SIZE*2];
#elif defined (__CC_ARM)
volatile uint8_t encodeFrameBuffer[ENCODE_SIZE*2] __attribute__((aligned(4)));
#else
#error Undefined toolkit, need to define alignment 
#endif
#endif

/* Check frameCount (odd or even frame finish) to exit EM1 */
extern volatile uint32_t frameCount;

/**************************************************************************//**
 * @brief Overide the _speex_putc function of the speex library
 *****************************************************************************/
void _speex_putc(int ch, void *file)
{
  while(1)
  {
    ;
  }
}

/**************************************************************************//**
 * @brief Overide the _speex_fatal function of the speex library
 *****************************************************************************/
void _speex_fatal(const char *str, const char *file, int line)
{
  while(1)
  {
    ;
  }
}

/**************************************************************************//**
 * @brief Speex playback function, data in play list
 * @param flashRecord - Play from flash record if true
 * @param freqBand - Frequency band (NB or WB or UWB) of encoded data
 * @param encFrameSize - Frame size of encoded data
 * @param *ptrStruct - Pointer to structure of play list
 *****************************************************************************/
void speexPlayBack(bool flashRecord, uint8_t freqBand, uint8_t encFrameSize, struct speexEncFile *ptrStruct)
{
  uint32_t i;
  uint16_t sampleBufferSize;
  
  char *ptrMem;
  uint32_t numOfFrames;

  ptrMem = ptrStruct->frameStart;
  numOfFrames = ptrStruct->frameNum;
  
  /* Initialization of the structure that holds the bits */
  speex_bits_init(&bits);

#if (SPEEX_BAND < WIDEBAND || SPEEX_BAND == NB8KCODEC)
  /* Create a new decoder state in narrowband mode */
  dec_state = speex_decoder_init(&speex_nb_mode);
  sampleBufferSize = SAMPLE_SIZE;
#elif SPEEX_BAND == WIDEBAND  
  /* Setup decoder for WB to play NB or WB */  
  if (freqBand < WIDEBAND)
  {
    /* Create a new decoder state in narrowband mode */
    dec_state = speex_decoder_init(&speex_nb_mode);
    sampleBufferSize = SAMPLE_SIZE/2;
  }
  else if (freqBand == WIDEBAND)
  {
    /* Create a new decoder state in wideband mode */
    dec_state = speex_decoder_init(&speex_wb_mode);
    sampleBufferSize = SAMPLE_SIZE;
  }
  else
  {
    /* Frequency band not support */
    return;
  }
#else
  /* Setup decoder for UWB to play NB or WB or UWB */  
  if (freqBand < WIDEBAND)
  {
    /* Create a new decoder state in narrowband mode */
    dec_state = speex_decoder_init(&speex_nb_mode);
    sampleBufferSize = SAMPLE_SIZE/4;
  }
  else if (freqBand == WIDEBAND)
  {
    /* Create a new decoder state in wideband mode */
    dec_state = speex_decoder_init(&speex_wb_mode);
    sampleBufferSize = SAMPLE_SIZE/2;
  }  
  else if (freqBand == ULTRAWIDEBAND)
  {
    /* Create a new decoder state in ultra-wideband mode */
    dec_state = speex_decoder_init(&speex_uwb_mode);
    sampleBufferSize = SAMPLE_SIZE;
  }
  else
  {
    /* Frequency band not support */
    return;
  }
#endif

  /* Prepare two buffers of decoded data: 1st one */
  /* Copy the data into the bit-stream struct */  
  speex_bits_read_from(&bits, ptrMem, encFrameSize);
  /* Decode the data */   
  speex_decode_int(dec_state, &bits, (int16_t *)sampleBuffer[0]);

  /* Copy to buffer for output (convert to 12 bit) */
  for(i=0; i<sampleBufferSize; i++)
  {
    sampleBuffer[0][i] = (sampleBuffer[0][i] >> 4) + DAC_OFFSET;
  }
  numOfFrames--;
  ptrMem += encFrameSize;
  
  /* 2nd one */
  /* Copy the data into the bit-stream struct */  
  speex_bits_read_from(&bits, ptrMem, encFrameSize);
  /* Decode the data */   
  speex_decode_int(dec_state, &bits, (int16_t *)sampleBuffer[1]);

  /* Copy to buffer for output (convert to 12 bit) */
  for(i=0; i<sampleBufferSize; i++)
  {
    sampleBuffer[1][i] = (sampleBuffer[1][i] >> 4) + DAC_OFFSET;
  }
  numOfFrames--;
  ptrMem += encFrameSize;

  /* Start output data */
  startDacDma(sampleBufferSize);

  while(1)
  {
    /* Wait odd frame finish */
    while (!(frameCount & 0x01))
    {
      EMU_EnterEM1();
    }
    /* Copy the data into the bit-stream struct */  
    speex_bits_read_from(&bits, ptrMem, encFrameSize);
    /* Decode the data */   
    speex_decode_int(dec_state, &bits, (int16_t *)sampleBuffer[0]);
    
    /* Copy to buffer for output (convert to 12 bit) */
    for(i=0; i<sampleBufferSize; i++)
    {
      sampleBuffer[0][i] = (sampleBuffer[0][i] >> 4) + DAC_OFFSET;
    }
    numOfFrames--;
    if (numOfFrames)
    {
      /* Play next frame if not last frame */
      ptrMem += encFrameSize;
    }
    else
    {
      /* End of list check if last frame */
      ptrStruct++;
      if (ptrStruct->frameStart != NULL)
      {
        /* Play next file if not end of list */
        ptrMem = ptrStruct->frameStart;
        numOfFrames = ptrStruct->frameNum;
      }
      else
      {
        /* Wati last 2 frames finish if end of list */
        while (frameCount & 0x01)
        {
          EMU_EnterEM1();
        }
        while (!(frameCount & 0x01))
        {
          EMU_EnterEM1();
        }
        /* Stop output data */
        stopDacDma();

        /* Destroy the bit-stream struct */
        speex_bits_destroy(&bits);
        /* Destroy the decoder state */
        speex_decoder_destroy(dec_state);
        return;
      }
    }
  
    /* Wait even frame finish */
    while (frameCount & 0x01)
    {
      EMU_EnterEM1();
    }
    /* Copy the data into the bit-stream struct */  
    speex_bits_read_from(&bits, ptrMem, encFrameSize);
    /* Decode the data */   
    speex_decode_int(dec_state, &bits, (int16_t *)sampleBuffer[1]);
        
    /* Copy to buffer for output (convert to 12 bit) */
    for(i=0; i<sampleBufferSize; i++)
    {
      sampleBuffer[1][i] = (sampleBuffer[1][i] >> 4) + DAC_OFFSET;
    }
    numOfFrames--;
    if (numOfFrames)
    {
      /* Play next frame if not last frame */
      ptrMem += encFrameSize;
#if SPEEX_BAND == NB8KCODEC      
      /* Frames record in flash must in even number */
      /* Check Flash page boundary if play from record */
      if (flashRecord)
      {
        if (((uint32_t)ptrMem & PAGE_MASK) == PAGE_END)
        {
          ptrMem += PAGE_OFFSET;
        }
      }
#endif
    }
    else
    {
      /* End of list check if last frame */
      ptrStruct++;
      if (ptrStruct->frameStart != NULL)
      {
        /* Play next file if not end of list */
        ptrMem = ptrStruct->frameStart;
        numOfFrames = ptrStruct->frameNum;
      }
      else
      {
        /* Wati last 2 frames finish if end of list */
        while (!(frameCount & 0x01))
        {
          EMU_EnterEM1();
        }
        while (frameCount & 0x01)
        {
          EMU_EnterEM1();
        }
        /* Stop output data */
        stopDacDma();

        /* Destroy the bit-stream struct */
        speex_bits_destroy(&bits);
        /* Destroy the decoder state */
        speex_decoder_destroy(dec_state);
        return;
      }
    }
  }
}

#if SPEEX_BAND == NB8KCODEC
/**************************************************************************//**
 * @brief Speex record function, encoded data place in Flash upper 512KB
 * @param complexity - MCU resources allowed for the encoder (1 or 2)
 * @param numOfFrames - Number of frames to record
 *****************************************************************************/
void speexRecordFlash512(uint32_t complexity, uint32_t numOfFrames)
{
  uint32_t i;
  uint32_t flashAddr = FLASHSTART;

  /* Check complexity */
  if (complexity > 2)
  {
    return;
  }
  
  /* Make sure even frame number */
  if (numOfFrames & 0x01)
  {
    numOfFrames++;
  }
  if (numOfFrames > FRAME_MAXIMUM)
  {
    return;
  }
  
  /* Initialization of the structure that holds the bits */
  speex_bits_init(&bits);
  
  /* Create a new encoder state in narrowband mode */
  enc_state = speex_encoder_init(&speex_nb_mode);

  /* Set parameters for encoder */
  i = 4;
  speex_encoder_ctl(enc_state, SPEEX_SET_QUALITY, &i);
  speex_encoder_ctl(enc_state, SPEEX_SET_COMPLEXITY, &complexity);
  
  /* Erase flash for writing */
  flashInit512();
  flashMassErase(MSC_WRITECMD_ERASEMAIN1);
  
  /* Start input data */
  startAdcDma();

  while(numOfFrames)
  {
    /* Wait odd frame finish */
    while (!(frameCount & 0x01))
    {
      EMU_EnterEM1();
    }

    /* Covnert ADC value to signed 16 bit */
    for(i=0; i<SAMPLE_SIZE; i++)
    {
      sampleBuffer[0][i] ^= ADC_OFFSET;
    }

    /* Flush all the bits in the struct so we can encode a new frame */
    speex_bits_reset(&bits);
    /* Encode the frame */
    speex_encode_int(enc_state, (int16_t *)sampleBuffer[0], &bits);
    /* Copy the bits to an array of char that can be written */
    speex_bits_write(&bits, (char *)encodeFrameBuffer, ENCODE_SIZE);
    numOfFrames--;
    
    /* Wait even frame finish */
    while (frameCount & 0x01)
    {
      EMU_EnterEM1();
    }

    /* Covnert ADC value to signed 16 bit */
    for(i=0; i<SAMPLE_SIZE; i++)
    {
      sampleBuffer[1][i] ^= ADC_OFFSET;
    }
    
    /* Flush all the bits in the struct so we can encode a new frame */
    speex_bits_reset(&bits);
    /* Encode the frame */
    speex_encode_int(enc_state, (int16_t *)sampleBuffer[1], &bits);
    /* Copy the bits to an array of char that can be written */
    speex_bits_write(&bits, (char *)(encodeFrameBuffer + ENCODE_SIZE), ENCODE_SIZE);
    numOfFrames--;

    /* Load the start address into the MSC */
    MSC->ADDRB    = flashAddr;
    MSC->WRITECMD = MSC_WRITECMD_LADDRIM;
    startFlashDma();
    
    /* Check Flash page boundary */
    flashAddr +=  FLASH_WR_SIZE;
    if ((flashAddr & PAGE_MASK) == PAGE_END)
    {
      flashAddr += PAGE_OFFSET;
    }
  }

  /* Stop input data */
  stopAdcDma();
  /* Ensure Flash write finish */
  while ((DMA->CHENS & DMA_CHANNEL_FLHMASK) || (MSC->STATUS & MSC_STATUS_BUSY))
  {
    ;
  }
  flashDeinit();
  /* Destroy the bit-packing struct */
  speex_bits_destroy(&bits);
  /* Destroy the encoder state */
  speex_encoder_destroy(enc_state);
}
#endif
