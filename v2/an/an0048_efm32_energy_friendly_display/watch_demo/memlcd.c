/**************************************************************************//**
 * @file memlcd.c
 * @brief Sharp Memory LCD Serial Interface
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

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_usart.h"
#include "em_timer.h"
#include "em_gpio.h"
#include "em_emu.h"
#include "memlcd.h"


/* Globals */
static MEMLCD_Config  *pConfig;
static int            timerCyclesPerMicrosecond;
static uint8_t        comPolarity = 0;


static void usDelay(uint16_t time)
{
   uint16_t timerTop;

   /* Calculate timeout value */
   timerTop = timerCyclesPerMicrosecond * time;
   TIMER_TopSet(pConfig->timer, timerTop);
   
   /* Clear overflow */
   pConfig->timer->IFC = TIMER_IFC_OF;

   /* Start timer */
   TIMER_Enable(pConfig->timer, true);

   /* Wait for overflow */
   while (!(pConfig->timer->IF & TIMER_IF_OF));
}


/***********************************************************
 * Initialize driver
 **********************************************************/
EMSTATUS MEMLCD_Init(MEMLCD_Config *cfg)
{
   USART_InitSync_TypeDef usartInit = USART_INITSYNC_DEFAULT;
   TIMER_Init_TypeDef timerInit     = TIMER_INIT_DEFAULT;

   /* Setup clocks */
   CMU_ClockEnable( cmuClock_GPIO, true );
   CMU_ClockEnable( cfg->timerClock, true );
   CMU_ClockEnable( cfg->usartClock, true );

   /* Setup GPIO's */
   GPIO_PinModeSet( cfg->sclk.port, cfg->sclk.pin, gpioModePushPull, 0 );
   GPIO_PinModeSet( cfg->si.port,   cfg->si.pin,   gpioModePushPull, 0 );
   GPIO_PinModeSet( cfg->scs.port,  cfg->scs.pin,  gpioModePushPull, 0 );

   GPIO_PinModeSet( cfg->extcomin.port, cfg->extcomin.pin, gpioModePushPull, 0 );
   GPIO_PinModeSet( cfg->extmode.port,  cfg->extmode.pin,  gpioModePushPull, 0 );
   GPIO_PinModeSet( cfg->disp.port,     cfg->disp.pin,     gpioModePushPull, 0 );  

   /* Copy configuration */
   pConfig = cfg;

   /* Setup timer. For usDelay() function */
   timerInit.enable  = false;
   timerInit.oneShot = true;
   TIMER_Init( cfg->timer, &timerInit );
   timerCyclesPerMicrosecond = CMU_ClockFreqGet( cfg->timerClock ) / 1000000;

   /* Setup USART */
   usartInit.baudrate = 1100000;
   usartInit.databits = usartDatabits16;
   
   USART_InitSync( cfg->usart, &usartInit );
   cfg->usart->ROUTE = (USART_ROUTE_CLKPEN | USART_ROUTE_TXPEN | cfg->usartLocation);
   
   

   if (cfg->enable)
   {
     GPIO_PinOutSet(cfg->disp.port, cfg->disp.pin);
   }

  return MEMLCD_OK;
}

/***********************************************************
 * Enable or disable the display. Disabling the display
 * does not make it lose it's data.
 **********************************************************/
void MEMLCD_Enable(bool enable)
{
  if (enable)
  {
    GPIO_PinOutSet(pConfig->disp.port, pConfig->disp.pin);
  }
  else
  {
    GPIO_PinOutClear(pConfig->disp.port, pConfig->disp.pin);
  }
}


/***********************************************************
 * Clear display
 **********************************************************/
void MEMLCD_Clear( void )
{
   uint8_t cmd;

   /* Set SCS */
   GPIO_PinOutSet( pConfig->scs.port, pConfig->scs.pin );
   
   /* SCS setup time: min 6us */
   usDelay(6);

   /* Send command */
   cmd = (MEMLCD_CMD_ALL_CLEAR | comPolarity);
   USART_TxDouble( pConfig->usart, cmd );

   /* Wait for transfer to finish */
   while ( !(pConfig->usart->STATUS & USART_STATUS_TXC) );

   /* SCS hold time: min 2us */
   usDelay(2);
   
   /* Clear SCS */
   GPIO_PinOutClear( pConfig->scs.port, pConfig->scs.pin );
}


/*
 * COM inversion function. Must be called at least every second
 */
void MEMLCD_ComInv(void)
{
   switch (pConfig->mode)
   {
   case memlcdMode_Extcom:
      /* Toggle extcomin gpio */
      GPIO_PinOutToggle( pConfig->extcomin.port, pConfig->extcomin.pin );
      break;

   case memlcdMode_Serial:
      /* Send a packet with inverted com */
      GPIO_PinOutSet( pConfig->scs.port, pConfig->scs.pin );
      
      /* SCS setup time: min 6us */
      usDelay(6);

      /* Send polarity command including dummy bits */
      USART_TxDouble( pConfig->usart, comPolarity );
      
      /* Wait for transfer to finish */
      while (!(pConfig->usart->STATUS & USART_STATUS_TXC)) ;
      
      /* SCS hold time: min 2us */
      usDelay(2);
      
      GPIO_PinOutClear( pConfig->scs.port, pConfig->scs.pin );

      /* Invert com polarity */
      if (comPolarity == 0x00)
      {
        comPolarity = 0x02;
      }
      else
      {
        comPolarity = 0x00;
      }
      break;
   }
}


/** @brief Update the display
  * @param firstLine The first line to update
  * @param lastLine The last line to update
  */
EMSTATUS MEMLCD_Update(  uint16_t *data, int firstLine, int lastLine )
{
  int i,j;
  
  /* Assert SCS */
  GPIO_PinOutSet( pConfig->scs.port, pConfig->scs.pin );

  /* SCS setup time: min 6us */
  usDelay(6);
  
  /* Send update command and first line address */
  USART_TxDouble(pConfig->usart, MEMLCD_CMD_UPDATE | (firstLine + 1) << 8);
  
  /* Get start address to draw from */
  uint16_t *p = (uint16_t *)data;
  p += firstLine * 10;
  
  
  for ( i=firstLine; i<=lastLine; i++ ) {
        
    /* Send pixels for this line */
    for ( j=0; j<9; j++ ) {
      USART_TxDouble(pConfig->usart, *p);
      p++;
    }
       
    /* Skip padding data in frame buffer */
    p += 1;
  }
  
  
  /* Wait for USART to finish */
  while (!(pConfig->usart->STATUS & USART_STATUS_TXC)) ;
  
  /* SCS hold time: min 2us */
  usDelay(2);
  
  /* De-assert SCS */
  GPIO_PinOutClear( pConfig->scs.port, pConfig->scs.pin );

  return MEMLCD_OK;
}



