/*******************************************************************************
 * @file cog.c
 * @brief COG driver
 * @author Silicon Labs
 * @version 1.02
 *******************************************************************************
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
#include "em_device.h"
#include "em_gpio.h"
#include "config.h"
#include "cog.h"
#include "pwm.h"
#include "delay.h"
#include "spi.h"
#include "temp.h"

   
#if defined(PANEL_144)
   
EPD_Config epdConfig =
{
  .channelSelect        = {0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xFF, 0x00},  /* Panel specific param */
  .voltageLevel         = 0x03,                 /* Panel specific param */
  .horizontalSize       = 128,                  /* Panel width in pixels */
  .verticalSize         = 96,                   /* Panel height in pixels */
  .lineSize             = 128/4 + 96/4 + 1,     /* Line size in bytes */
  .stageTime            = 120,                  /* Time per stage in ms */
  .dummyByte            = false,                /* No dummy byte */
  .borderByte           = true                  /* Include border byte */
};   
   
#elif defined(PANEL_200)

EPD_Config epdConfig =
{
  .channelSelect        = {0x00, 0x00, 0x00, 0x00, 0x01, 0xFF, 0xE0, 0x00},  /* Panel specific param */
  .voltageLevel         = 0x03,                 /* Panel specific param */
  .horizontalSize       = 200,                  /* Panel width in pixels */
  .verticalSize         = 96,                   /* Panel height in pixels */
  .lineSize             = 200/4 + 96/4 + 1,     /* Line size in bytes */
  .stageTime            = 120,                  /* Time per stage in ms */
  .dummyByte            = true,                 /* Include dummy byte */
  .borderByte           = false                 /* No border byte */
};
   
#elif defined(PANEL_270)

EPD_Config epdConfig =
{
  .channelSelect        = {0x00, 0x00, 0x00, 0x7F, 0xFF, 0xFE, 0x00, 0x00},   /* Panel specific param */
  .voltageLevel         = 0x00,                 /* Panel specific param */
  .horizontalSize       = 264,                  /* Panel width in pixels */
  .verticalSize         = 176,                  /* Panel height in pixels */
  .lineSize             = 264/4 + 176/4 + 1,    /* Line size in bytes */
  .stageTime            = 240,                  /* Time per stage in ms */
  .dummyByte            = true,                 /* Include dummy byte */
  .borderByte           = false                 /* No border byte */
};
   
#endif
   

/**********************************************************
 * COG power-up sequence. 
 **********************************************************/
void cogPowerUp(void)
{
  measureTemperature();
  
  GPIO_PinOutSet(EPD_PIN_PANEL_VDD);
  
  delayMs(5);
  
  pwmEnable();
  
  delayMs(5);
  
  GPIO_PinOutSet(EPD_PIN_PANEL_ON);
  
  delayMs(10);
  
  GPIO_PinOutSet(EPD_PIN_CS);
  
  GPIO_PinOutSet(EPD_PIN_BORDER);
  
  GPIO_PinOutSet(EPD_PIN_RESET);
  
  delayMs(5);
  
  GPIO_PinOutClear(EPD_PIN_RESET);
  
  delayMs(5);
  
  GPIO_PinOutSet(EPD_PIN_RESET);
  
  delayMs(5);
}


/**********************************************************
 * COG init sequence
 **********************************************************/
void cogInit(void)
{
  while ( GPIO_PinInGet(EPD_PIN_BUSY) );
  
  /* Channel select */
  spiSend(0x01, epdConfig.channelSelect, 8);
  
  /* DC/DC frequency setting */
  spiSend1(0x06, 0xFF);
  
  /* High Power Mode Oscillator Setting */
  spiSend1(0x07, 0x9D);
  
  /* Disable ADC */
  spiSend1(0x08, 0x00);
  
  /* Set Vcom Level */
  spiSend2(0x09, 0xD000);
  
  /* Gate and Source Voltage Level*/
  spiSend1(0x04, epdConfig.voltageLevel);
  
  delayMs(5);
  
  /* Driver latch on (cancel register noise) */
  spiSend1(0x03, 0x01);
  
  /* Driver latch off */
  spiSend1(0x03, 0x00);
  
  /* Start chargepump positive voltage. VGH and VDH on */
  spiSend1(0x05, 0x01);
  
  delayMs(30);
  
  /* Stop PWM */
  pwmDisable();
  
  /* Start chargepump on negative voltage. VGL and VDL on */
  spiSend1(0x05, 0x03);
  
  delayMs(30);
  
  /* Set chargepump Vcom driver to ON */
  spiSend1(0x05, 0x0F);
  
  delayMs(30);
  
  /* Output enable to disable */
  spiSend1(0x02, 0x24);
}


/**********************************************************
 * Sends a dummy line to the COG driver. A dummy
 * line has all pixel bytes set to 0x55 and all
 * scan bytes are 0x00. 
 **********************************************************/
void sendDummyLine(void)
{
  int i;
  uint8_t lineBuffer[PANEL_LINE_SIZE];
  uint8_t *p = lineBuffer;
  
  /* Dummy byte */
  if ( epdConfig.borderByte )
  {
    *p++ = 0x00;
  }
  
  /* Even bytes */
  for ( i=0; i<epdConfig.horizontalSize/(4*2); i++ )
  {
    *p++ = 0x55;
  }
  
  /* Scan bytes */
  for ( i=0; i<epdConfig.verticalSize/4; i++ )
  {
    *p++ = 0x00;
  }
  
  /* Odd bytes */
  for ( i=0; i<epdConfig.horizontalSize/(4*2); i++ )
  {
    *p++ = 0x55;
  }

  /* Dummy byte */
  if ( epdConfig.dummyByte )
  {
    *p++ = 0x00;
  }
  
  /* Set chargepump voltage level */
  spiSend1(0x04, epdConfig.voltageLevel);

  /* Send one line */
  spiSend(0x0A, lineBuffer, epdConfig.lineSize);

  /* Output data from COG to panel */
  spiSend1(0x02, 0x2F);
}



/**********************************************************
 * Sends one frame to the COG driver
 * 
 * @param frameBuffer
 *   Pointer to the COG/EPD frame buffer
 * 
 **********************************************************/
void sendFrame(uint8_t *frameBuffer)
{
  int i;
  
  for ( i=0; i<epdConfig.verticalSize; i++ )
  {
    /* Set chargepump voltage level */
    spiSend1(0x04, epdConfig.voltageLevel);
 
    /* Send one line */
    spiSend(0x0A, frameBuffer + i * epdConfig.lineSize, epdConfig.lineSize);
 
    /* Output data from COG to panel */
    spiSend1(0x02, 0x2F);
  }
}


/**********************************************************
 * Sends a nothing frame to the COG driver. A nothing
 * frame has all pixel bytes set to 0x55 and all
 * scan bytes are 0x00. 
 **********************************************************/
void sendNothingFrame(void)
{
  int i;
  uint8_t lineBuffer[PANEL_LINE_SIZE];
  uint8_t *p = lineBuffer;
  uint8_t *scanBytes;
  uint32_t scanByte; 
  uint32_t scanShift;
  
  /* Dummy byte */
  if ( epdConfig.borderByte )
  {
    *p++ = 0x00;
  }
  
  /* Even bytes */
  for ( i=0; i<epdConfig.horizontalSize/(4*2); i++ )
  {
    *p++ = 0x55;
  }
  
  /* Save pointer */
  scanBytes = p;
  
  /* Scan bytes */
  for ( i=0; i<epdConfig.verticalSize/4; i++ )
  {
    *p++ = 0x00;
  }
  
  /* Odd bytes */
  for ( i=0; i<epdConfig.horizontalSize/(4*2); i++ )
  {
    *p++ = 0x55;
  }

  /* Dummy byte */
  if ( epdConfig.dummyByte )
  {
    *p++ = 0x00;
  }
  
  /* Loop over all lines */
  for ( i=0; i<epdConfig.verticalSize; i++ )
  {
    /* Set chargepump voltage level */
    spiSend1(0x04, epdConfig.voltageLevel);
    
    /* Calculate the position of the current line in the scan lines buffer. */
    scanByte = i / 4;
    scanShift = 6 - (i%4)*2;
    
    /* The scan line should have 0b11 at the current line and 0 for the rest. */
    scanBytes[scanByte] = 0x3 << scanShift;
 
    /* Send one line */
    spiSend(0x0A, lineBuffer, epdConfig.lineSize);
 
    /* Output data from COG to panel */
    spiSend1(0x02, 0x2F);
    
    /* Clear scan byte */
    scanBytes[scanByte] = 0x00; 
  }
}

/**********************************************************
 * Sends a frame multiple times to the COG driver. A frame
 * is sent multiple times to improve contrast and 
 * reduce ghosting. The number of times the frame
 * is resent is dependent on the type of display
 * and current temperature. 
 * 
 * @param frameBuffer
 *   Pointer to the COG/EPD frame buffer
 **********************************************************/
void updateStage(uint8_t *frameBuffer)
{
  /* Calculate the stage time in ms */
  uint32_t endTime = getTempAdjustedStageTime(epdConfig.stageTime);
  
  /* Resend the frame for the entire stage time */
  rtcStart();
  do 
  {
    sendFrame(frameBuffer);
  }
  while ( rtcGetMs() < endTime );
  rtcStop();
}

/**********************************************************
 * COG power-down sequence. 
 **********************************************************/
void cogPowerOff(void)
{  
  sendNothingFrame();
  
  sendDummyLine();
  
  delayMs(25);
  
  GPIO_PinOutClear(EPD_PIN_BORDER);
  
  delayMs(200);
  
  GPIO_PinOutSet(EPD_PIN_BORDER);
  
  /* Latch reset turn on */
  spiSend1(0x03, 0x01);
  
  /* Turn off output enable */
  spiSend1(0x02, 0x05);
  
  /* Power off chargepump Vcom */
  spiSend1(0x05, 0x0E);
  
  /* Power off chargepump negative voltage */
  spiSend1(0x05, 0x02);
  
  /* Discharge */
  spiSend1(0x04, 0x0C);
  
  delayMs(120);
  
  /* Turn off all chargepumps */
  spiSend1(0x05, 0x00);
  
  /* Turn off oscillator */
  spiSend1(0x07, 0x0D);
  
  /* Discharge internal */
  spiSend1(0x04, 0x50);
  
  delayMs(40);
  
  /* Discharge internal */
  spiSend1(0x04, 0xA0);
  
  delayMs(40);
  
  /* Discharge internal */
  spiSend1(0x04, 0x00);
  
  GPIO_PinOutClear(EPD_PIN_RESET);
  GPIO_PinOutClear(EPD_PIN_CS);
  GPIO_PinOutClear(EPD_PIN_PANEL_ON);
  GPIO_PinOutClear(EPD_PIN_BORDER);
  
  GPIO_PinOutSet(EPD_PIN_DISCHARGE);
  delayMs(150);
  GPIO_PinOutClear(EPD_PIN_DISCHARGE);
  
  /* Turn off panel VDD */
  GPIO_PinOutClear(EPD_PIN_PANEL_VDD);
}
