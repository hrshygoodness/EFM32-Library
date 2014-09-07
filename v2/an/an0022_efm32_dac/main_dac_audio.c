/*****************************************************************************
 * @file main_dac_audio.c
 * @brief Digital to Analog converter, wav audio output example
 * @author Silicon Labs
 * @version 1.11
 * @note
 *   WARNING: Do not attach or use headphones with this example. Use small
 *   loadspeakers with built in amplification, ensuring volume is at an
 *   acceptable level. Exposure to loud noises from any source for extended
 *   periods of time may temporarily or permanently affect your hearing. The
 *   louder the volume sound level, the less time is required before your
 *   hearing could be affected. Hearing damage from loud noise is sometimes
 *   undetectable at first and can have a cumulative effect.
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
#include <string.h>
#include <stdint.h>

#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_dac.h"
#include "em_prs.h"
#include "em_timer.h"
#include "em_dma.h"
#include "dmactrl.h"
#include "ff.h"
#include "microsd.h"
#include "diskio.h"
#include "bsp.h"

/* Function prototypes */

void FillBufferFromSDcard(bool stereo, bool primary);
void PingPongTransferComplete(unsigned int channel, bool primary, void *user);

void DMA_setup(void);
void DAC_setup(void);
void TIMER_setup(void);

int initFatFS(void);

DWORD get_fattime(void);
void SysTick_Handler(void);
void Delay(uint32_t dlyTicks);

/* counts 1ms timeTicks */
volatile uint32_t msTicks;

/* DMA callback structure */
DMA_CB_TypeDef DMAcallBack;

/* Filename to open from SD-card */
#define WAV_FILENAME    "soundfile.wav"

/* Ram buffers */
/* BUFFERSIZE should be between 512 and 1024, depending on available ram on efm32 */
#define BUFFERSIZE    512

/* Temporary buffer for use when source is mono, can't put samples directly in
 * stereo DMA buffer with f_read(). */
int16_t ramBufferTemporaryMono[BUFFERSIZE];

/* Buffers for DMA transfer, 32 bits are transfered at a time with DMA.
 * The buffers are twice as large as BUFFERSIZE to hold both left and right
 * channel samples. */
int16_t ramBufferDacData0Stereo[2 * BUFFERSIZE];
int16_t ramBufferDacData1Stereo[2 * BUFFERSIZE];

/* Bytecounter, need to stop DMA when finished reading file */
uint32_t ByteCounter;

/* File system specific */
FATFS Fatfs;

/* File to read bmp data from */
FIL WAVfile;

/* WAV header structure */
typedef struct
{
  char     id[4];                /* should always contain "RIFF" */
  uint32_t totallength;          /* total file length minus 8 */
  char     wavefmt[8];           /* should be "WAVEfmt " */
  uint32_t format;               /* 16 for PCM format */
  uint16_t pcm;                  /* 1 for PCM format */
  uint16_t channels;             /* channels */
  uint32_t frequency;            /* sampling frequency */
  uint32_t bytes_per_second;
  uint16_t bytes_per_capture;
  uint16_t bits_per_sample;
  char     data[4];             /* should always contain "data" */
  uint32_t bytes_in_data;
} WAV_Header_TypeDef;

/* Header info is used in several functions, make global */
WAV_Header_TypeDef wavHeader;


/**************************************************************************//**
 * @brief  Main function
 * Configures the DVK for sound output, reads the wav header and fills the data
 * buffers. After the DAC, DMA, Timer and PRS are set up to perform playback
 * the mainloop just enters em1 continuously. A microSD-card is needed to
 * store the audio *.wav file.
 *
 * This example demonstrates how to play sound data with a fixed sampling
 * frequency of 44.1kHz. This can be useful for generating sounds or signals
 * both for audio-playback or custom signal generation.
 *****************************************************************************/
int main(void)
{
  /* Initialize chip */
  CHIP_Init();

  UINT bytes_read;
  ByteCounter = 0;

  /* Use 32MHZ HFXO as core clock frequency, need high speed for 44.1kHz stereo */
  CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);

  /* Initialize DVK board register access */
  BSP_Init(BSP_INIT_DEFAULT);

  /* Setup SysTick Timer for 10 msec interrupts  */
  if (SysTick_Config(CMU_ClockFreqGet(cmuClock_CORE) / 100))
  {
    while (1) ;
  }

  /* Enable SPI access to MicroSD card */
  BSP_RegisterWrite(BC_SPI_CFG, 1);
  BSP_PeripheralAccess(BSP_SPI, true);

  /* Enable Audio out driver */
  BSP_PeripheralAccess(BSP_AUDIO_OUT, true);

  /* Initialize filesystem */
  initFatFS();

  /* Open wav file from SD-card */
  f_open(&WAVfile, WAV_FILENAME, FA_READ);

  /* Read header and place in header struct */
  f_read(&WAVfile, &wavHeader, sizeof(wavHeader), &bytes_read);

  /* Start clocks */
  CMU_ClockEnable(cmuClock_DMA, true);
  CMU_ClockEnable(cmuClock_DAC0, true);
  CMU_ClockEnable(cmuClock_TIMER0, true);
  CMU_ClockEnable(cmuClock_PRS, true);

  /* Fill both primary and alternate RAM-buffer before start */
  FillBufferFromSDcard((bool) wavHeader.channels, true);
  FillBufferFromSDcard((bool) wavHeader.channels, false);

  /* Setup DMA and peripherals */
  DMA_setup();
  TIMER_setup();
  DAC_setup();

  /* Enable DAC channel 0 and 1, located on pin PB11 and PB12 */
  DAC_Enable(DAC0, 0, true);
  DAC_Enable(DAC0, 1, true);

  while (1)
  {
    /* Enter EM1 while the DAC, Timer, PRS and DMA is working */
    EMU_EnterEM1();
  }
}

/**************************************************************************//**
 * @brief  Used for filling up a memory buffer with data from the sd-card
 *****************************************************************************/
void FillBufferFromSDcard(bool stereo, bool primary)
{
  UINT    bytes_read;
  int16_t * buffer;
  int     i, j;

  /* Set buffer pointer correct ram buffer */
  if (primary)
  {
    buffer = ramBufferDacData0Stereo;
  }
  else /* Alternate */
  {
    buffer = ramBufferDacData1Stereo;
  }

  if (stereo)
  {
    /* Stereo, Store Left and Right data interlaced as in wavfile */
    /* DMA is writing the data to the combined register as interlaced data*/

    /* First buffer 0 is filled from SD-card */
    f_read(&WAVfile, buffer, 4 * BUFFERSIZE, &bytes_read);
    ByteCounter += bytes_read;

    /* Make samples 12 bits and unsigned */
    for (i = 0; i < 2 * BUFFERSIZE; i++)
    {
      buffer[i] = (buffer[i] + 0x7fff) >> 4;
    }
  }
  else /* Mono */
  {
    /* Read data for first buffer */
    f_read(&WAVfile, buffer, 2 * BUFFERSIZE, &bytes_read);
    ByteCounter += bytes_read;

    j = 0;
    for (i = 0; i < (2 * BUFFERSIZE) - 1; i += 2)
    {
      /* Mono, make samples 12 bit unsigned, put value in both left and right channel */
      buffer[i]     = (ramBufferTemporaryMono[j] + 0x7fff) >> 4;
      buffer[i + 1] = (ramBufferTemporaryMono[j] + 0x7fff) >> 4;
      j++;
    }
  }
}

/**************************************************************************//**
 * @brief  Called When DAC PingPong Transfer is Complete in Stereo mode
 *****************************************************************************/
void PingPongTransferComplete(unsigned int channel, bool primary, void *user)
{
  (void) channel;
  (void) user;
  
  FillBufferFromSDcard((bool) wavHeader.channels, primary);

  /* Stop DMA if bytecounter is equal to datasize or larger */
  bool stop = false;
  if (ByteCounter >= wavHeader.bytes_in_data)
  {
    stop = true;
  }

  /* Refresh the DMA control structure */
  DMA_RefreshPingPong(0,
                      primary,
                      false,
                      NULL,
                      NULL,
                      BUFFERSIZE - 1,
                      stop);
}

/**************************************************************************//**
 * @brief Setup DAC
 * Configures the DAC
 *****************************************************************************/
void DAC_setup(void)
{
  /* Use default settings */
  DAC_Init_TypeDef        init        = DAC_INIT_DEFAULT;
  DAC_InitChannel_TypeDef initChannel = DAC_INITCHANNEL_DEFAULT;


  /* Calculate the DAC clock prescaler value that will result in a DAC clock
   * close to 1 MHz. Second parameter is zero, if the HFPERCLK value is 0, the
   * function will check what the HFPERCLK actually is. */
  init.prescale = DAC_PrescaleCalc(1000000, 0);

  /* Set reference voltage to 1.25V */
  init.reference = dacRef1V25;

  /* Initialize the DAC. */
  DAC_Init(DAC0, &init);

  /* Enable prs to trigger samples at the right time with the timer */
  initChannel.prsEnable = true;
  initChannel.prsSel    = dacPRSSELCh0;

  /* Both channels can be configured the same
   * and be triggered by the same prs-signal. */
  DAC_InitChannel(DAC0, &initChannel, 0);
  DAC_InitChannel(DAC0, &initChannel, 1);
}

/**************************************************************************//**
 * @brief  Setup TIMER for prs triggering of DAC conversion
 *****************************************************************************/
void TIMER_setup(void)
{
  uint32_t timerTopValue;
  /* Use default timer configuration, overflow on counter top and start counting
   * from 0 again. */
  TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;

  TIMER_Init(TIMER0, &timerInit);

  /* PRS setup */
  /* Select TIMER0 as source and TIMER0OF (Timer0 overflow) as signal */
  PRS_SourceSignalSet(0, PRS_CH_CTRL_SOURCESEL_TIMER0, PRS_CH_CTRL_SIGSEL_TIMER0OF, prsEdgeOff);

  /* Calculate the proper overflow value */
  timerTopValue = CMU_ClockFreqGet(cmuClock_TIMER0) / wavHeader.frequency;

  /* Write new topValue */
  TIMER_TopBufSet(TIMER0, timerTopValue);
}

/**************************************************************************//**
 * @brief  Setup DMA in ping pong mode
 *****************************************************************************/
void DMA_setup(void)
{
  /* DMA configuration structs */
  DMA_Init_TypeDef       dmaInit;
  DMA_CfgChannel_TypeDef chnlCfg;
  DMA_CfgDescr_TypeDef   descrCfg;

  /* Initializing the DMA */
  dmaInit.hprot        = 0;
  dmaInit.controlBlock = dmaControlBlock;
  DMA_Init(&dmaInit);

  /* Set the interrupt callback routine */
  DMAcallBack.cbFunc = PingPongTransferComplete;

  /* Callback doesn't need userpointer */
  DMAcallBack.userPtr = NULL;

  /* Setting up channel */
  chnlCfg.highPri   = false; /* Can't use with peripherals */
  chnlCfg.enableInt = true;  /* Interrupt needed when buffers are used */

  /* channel 0 and 1 will need data at the same time,
   * can use channel 0 as trigger */
  chnlCfg.select = DMAREQ_DAC0_CH0;
  chnlCfg.cb     = &DMAcallBack;
  DMA_CfgChannel(0, &chnlCfg);

  /* Setting up channel descriptor */
  /* Destination is DAC register and doesn't move */
  descrCfg.dstInc = dmaDataIncNone;

  /* Transfer 32 bit each time to DAC_COMBDATA register*/
  descrCfg.srcInc = dmaDataInc4;
  descrCfg.size   = dmaDataSize4;

  /* We have time to arbitrate again for each sample */
  descrCfg.arbRate = dmaArbitrate1;

  descrCfg.hprot = 0; /* No need to have high priority */

  /* Configure both primary and secondary descriptor alike */
  DMA_CfgDescr(0, true, &descrCfg);
  DMA_CfgDescr(0, false, &descrCfg);


  /* Enabling PingPong Transfer*/
  DMA_ActivatePingPong(0,
                       false,
                       (void *) &(DAC0->COMBDATA),
                       (void *) &ramBufferDacData0Stereo,
                       BUFFERSIZE - 1,
                       (void *) &(DAC0->COMBDATA),
                       (void *) &ramBufferDacData1Stereo,
                       BUFFERSIZE - 1);
}

/***************************************************************************//**
 * @brief
 *   Initialize MicroSD driver.
 * @return
 *   Returns 0 if initialization succeded, non-zero otherwise.
 ******************************************************************************/
int initFatFS(void)
{
  MICROSD_Init();
  if (f_mount(0, &Fatfs) != FR_OK)
    return -1;
  return 0;
}

/***************************************************************************//**
 * @brief
 *   This function is required by the FAT file system in order to provide
 *   timestamps for created files. Since we do not have a reliable clock we
 *   hardcode a value here.
 *
 *   Refer to drivers/fatfs/doc/en/fattime.html for the format of this DWORD.
 * @return
 *    A DWORD containing the current time and date as a packed datastructure.
 ******************************************************************************/

DWORD get_fattime(void)
{
  return (28 << 25) | (2 << 21) | (1 << 16);
}

/**************************************************************************//**
 * @brief SysTick_Handler
 * Interrupt Service Routine for system tick counter.
 *****************************************************************************/
void SysTick_Handler(void)
{
  /* Increment counter necessary in Delay()*/
  msTicks++;
}

/**************************************************************************//**
 * @brief Delays number of msTick Systicks (typically 1 ms)
 * @param dlyTicks Number of ticks to delay
 *****************************************************************************/

void Delay(uint32_t dlyTicks)
{
  uint32_t curTicks;

  curTicks = msTicks;
  while ((msTicks - curTicks) < dlyTicks) ;
}
