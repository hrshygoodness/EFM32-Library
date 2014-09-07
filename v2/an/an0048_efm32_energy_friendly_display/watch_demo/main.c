/**************************************************************************//**
 * @file main.c
 * @brief Energy Friendly Display Example
 * @author Silicon Labs
 * @version 1.05
 * 
 * This example shows how to optimize your code in order to drive 
 * a graphical display in an energy friendly way. 
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
#include "em_prs.h"
#include "em_letimer.h"
#include "memlcd.h"


/* The 'high' and 'low' frequency bands used */
#define CORE_FREQ_HIGH cmuHFRCOBand_14MHz
#define CORE_FREQ_LOW  cmuHFRCOBand_1MHz

/* The number to use for USART CLKDIV when in 'low frequency' mode */
#define USART_LOW_FREQ_CLKDIV (0 << _USART_CLKDIV_DIV_SHIFT)
  
/* The different application modes. The user can toggle between these using PB0 */
#define MODE_ANALOG  1
#define MODE_DIGITAL 2


/* DMA prototypes */
void dmaStartFrameTransfer(int firstLine, int lastLine);
void initDMA(MEMLCD_Config *config);

/* Analog watch prototypes */
RowLimits analog_getDrawingLimits(struct tm *t);
void ANALOG_init(void);
void ANALOG_drawWatchFace(struct tm *t);

/* Digital watch prototypes */
RowLimits DIGITAL_drawWatchFace(struct tm *);
void DIGITAL_init(void);

/* Frame buffer control prototypes */
void FB_copyBackgroundToFrameBuffer(void);
void FB_flipBuffer(void);



/* Flag to check if the SPI transfer is currently active */
extern volatile bool spiTransferActive;

/* The current time reference. Number of seconds since midnight
 * January 1, 1970.  */
static time_t curTime = 1357810550;

/* Display configuration. See memlcd.h */
MEMLCD_Config memlcdConf = MEMLCD_CONFIG_STK3700_EXPBOARD;

/* Clip rectangle for emWin. Since the frame buffer is wider than the display 
 * itself to accommodate control signals we need to clip the display 
 * so that emWin will not overwrite the control signals */
static GUI_RECT clipRect = {0, 0, 127, 127};

/* Flag to check when we should redraw a frame */
static volatile bool updateDisplay = true;

/* Temporary storage for the USART CLKDIV register. Needed when changing clock 
 * frequency. */
static uint32_t usart_clk_div;

/* The current display mode (analog or digital watch face). 
 * Can be changed by the user by pushing PB0 */
static int mode = MODE_ANALOG;




/** Sets up an interrupt handler to let the user change between
  * analog and digital display mode */
void initGPIO(void)
{
  CMU_ClockEnable(cmuClock_GPIO, true);
  GPIO_PinModeSet(gpioPortB, 9, gpioModeInputPull, 1);
  GPIO_IntConfig(gpioPortB, 9, false, true, true);
  NVIC_EnableIRQ(GPIO_ODD_IRQn);
}

void initLETIMER(void)
{
  CMU_ClockEnable(cmuClock_CORELE, true);
  CMU_ClockEnable(cmuClock_LETIMER0, true);  
  
  GPIO_PinModeSet(memlcdConf.extcomin.port, memlcdConf.extcomin.pin, gpioModePushPull, 0);
    
  /* Set initial compare values for COMP0 */
  LETIMER_CompareSet(LETIMER0, 0, 32768);

  /* Route LETIMER to location 3 (OUT0 - PC4) and enable output */
  LETIMER0->ROUTE = LETIMER_ROUTE_OUT0PEN | LETIMER_ROUTE_LOCATION_LOC3;
    
  /* The current version of emlib (3.0.0) does not properly set 
   * REP0 to a nonzero value while enabling LETIMER in Free mode. 
   * Therefore we set REP0 manually here */
  LETIMER0->REP0 = 0x01;
  
  /* Set configurations for LETIMER 0 */
  const LETIMER_Init_TypeDef letimerInit = 
  {
  .enable         = true,                   /* Don't start counting when init completed - only with RTC compare match */
  .debugRun       = false,                  /* Counter shall not keep running during debug halt. */
  .rtcComp0Enable = false,                  /* Don't start counting on RTC COMP0 match. */
  .rtcComp1Enable = false,                  /* Don't start counting on RTC COMP1 match. */
  .comp0Top       = true,                   /* Load COMP0 register into CNT when counter underflows. COMP is used as TOP */
  .bufTop         = false,                  /* Don't load COMP1 into COMP0 when REP0 reaches 0. */
  .out0Pol        = 0,                      /* Idle value for output 0. */
  .out1Pol        = 0,                      /* Idle value for output 1. */
  .ufoa0          = letimerUFOAToggle,      /* Pulse output on output 0 */
  .ufoa1          = letimerUFOANone,        /* No output on output 1*/
  .repMode        = letimerRepeatFree       /* Repeat indefinitely */
  };
  
  /* Initialize LETIMER */
  LETIMER_Init(LETIMER0, &letimerInit); 
}

/* Set up RTC to generate an interrupt every second */
void initRTC(void)
{
  RTC_Init_TypeDef rtcInit = RTC_INIT_DEFAULT;

  /* Enable LE domain registers */
  CMU_ClockEnable(cmuClock_CORELE, true);

  /* Enable LFRCO as LFACLK in CMU to use for the RTC */
  CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFRCO);

  /* Enable RTC clock */
  CMU_ClockEnable(cmuClock_RTC, true);
  
  /* Set RTC prescaling */
  CMU_ClockDivSet(cmuClock_RTC, cmuClkDiv_16384);

  /* Initialize RTC */
  rtcInit.enable   = false;  /* Do not start RTC after initialization is complete. */
  rtcInit.debugRun = false;  /* Halt RTC when debugging. */
  rtcInit.comp0Top = true;   /* Wrap around on COMP0 match. */
  RTC_Init(&rtcInit);

  /* Interrupt every second */
  RTC_CompareSet(0, 1);

  /* Enable interrupt */
  NVIC_EnableIRQ(RTC_IRQn);
  RTC_IntEnable(RTC_IEN_COMP0);

  /* Start counter */
  RTC_Enable(true);
}

/* Set the HFRCO band to a 'low' frequency. This is used when transferring
 * the frame over SPI with DMA */
void setLowFrequency(void)
{
  usart_clk_div = memlcdConf.usart->CLKDIV;
  
  CMU_HFRCOBandSet(CORE_FREQ_LOW);  
  memlcdConf.usart->CLKDIV = USART_LOW_FREQ_CLKDIV;
}

/* Set the HFRCO to 'high frequency'. The high frequency is used 
 * when the CPU is working */
void setHighFrequency(void)
{
  CMU_HFRCOBandSet(CORE_FREQ_HIGH);
  memlcdConf.usart->CLKDIV = usart_clk_div;
}

/** This function is called to tell that the 
  * the display should be redrawn */
void invalidateDisplay(void)
{
  updateDisplay = true;
}
    
/** Shows a digital watch on the display */
void showDigitalWatch(void)
{
  uint32_t frameCounter = 0;
  RowLimits limits;
  struct tm *time = localtime(&curTime);
  
  /* Disable clipping. Needed because digital init will draw to a larger number buffer */
  GUI_SetClipRect(NULL);
  
  DIGITAL_init();
  
  /* Set clipping to avoid emWin overwriting frame buffer control signals */
  GUI_SetClipRect(&clipRect); 
       
  while (mode == MODE_DIGITAL)
  {
    if ( updateDisplay ) {

      limits = DIGITAL_drawWatchFace(time);

      
      /* Set low core frequency while transferring data over SPI */
      setLowFrequency();
      
      /* Only update the part of display that was changed */
      if ( frameCounter == 0 ) {
        dmaStartFrameTransfer(0,127);
      } else {
        dmaStartFrameTransfer(limits.min, limits.max);
      }
      
      /* Wait until SPI transfer is done */
      while ( spiTransferActive ) {
        EMU_EnterEM1();
      }
      
      setHighFrequency();
      
      /* Increase time with 1s */
      curTime++;
      time = localtime(&curTime);

      frameCounter++;
      updateDisplay = false;
    }
  
    /* Sleep between each frame update */
    EMU_EnterEM2(false);
  }  
} 
  
/** Shows an analog watch on the display */
void showAnalogWatch(void) 
{
  uint32_t frameCounter = 0;
  RowLimits limits;
  struct tm *time = localtime(&curTime);
  
  GUI_SetClipRect(&clipRect); 
  
  ANALOG_init();
  
  ANALOG_drawWatchFace(time);
  
  while (mode == MODE_ANALOG)
  {
    if ( updateDisplay ) {
      
      /* Only update the part of display that was changed */
      limits = analog_getDrawingLimits(time);
      if ( frameCounter == 0  ) {
        dmaStartFrameTransfer(0,127);
      } else {
        dmaStartFrameTransfer(limits.min, limits.max);
      }
      
      FB_flipBuffer();
      
      FB_copyBackgroundToFrameBuffer();

      /* Increase time with 1s */
      curTime++;
      time = localtime(&curTime);
      
      /* Draw watch face to frame buffer */
      ANALOG_drawWatchFace(time);
      
      /* Set low core frequency while transferring data over SPI */
      setLowFrequency();

      /* Wait until SPI transfer is done */
      while ( spiTransferActive ) {
        EMU_EnterEM1();
      }
      
      setHighFrequency();

      frameCounter++;
      updateDisplay = false;
    }
    
    /* Sleep between each frame update */
    EMU_EnterEM2(false);
  }
}  



int main(void)
{
  CHIP_Init();
  
  initDMA(&memlcdConf);
  
  /* Enable interrupt on push button PB0 */
  initGPIO();
  
  MEMLCD_Init(&memlcdConf);
  
  /* Set RTC to generate interrupt every second */
  initRTC();
   
  /* Start LETIMER to toggle EXTCOMIN @ 1 Hz */
  initLETIMER();
  
  /* Initialize emWin */
  GUI_Init();
  
  while (1) {
    if ( mode == MODE_ANALOG ) {
      showAnalogWatch();
    } else if ( mode == MODE_DIGITAL ) {
      showDigitalWatch();
    }
  }
}


/* This interrupt is triggered every second by the RTC */
void RTC_IRQHandler(void)
{
  RTC_IntClear(RTC_IF_COMP0); 
  
  /* Redraw frame */
  invalidateDisplay();
}

/** Triggered when user presses push button 0 (connected to 
  * pin PB9 on STK3700). Toggles display mode. */
void GPIO_ODD_IRQHandler(void)
{
  uint32_t flags = GPIO_IntGet();
  GPIO_IntClear(flags);
  if ( flags & (1 << 9) ) 
  {
    if ( mode == MODE_ANALOG ) {
      mode = MODE_DIGITAL;
    } else {
      mode = MODE_ANALOG;
    }
  }
}
