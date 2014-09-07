/**************************************************************************//**
 * @file memlcd.h
 * @brief Configuration for the Memory LCD
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
#ifndef MEMLCD_H_
#define MEMLCD_H_

#include "em_gpio.h"
#include "em_cmu.h"

typedef uint32_t   EMSTATUS;

#define MEMLCD_SIZE_Y   128
#define MEMLCD_SIZE_X   128

#define MEMLCD_CMD_UPDATE                 0x01
#define MEMLCD_CMD_ALL_CLEAR              0x04


#define MEMLCD_OK 0x00000000


typedef struct _MEMLCD_Pin
{
  GPIO_Port_TypeDef port;
  unsigned int      pin;
} MEMLCD_Pin;

typedef enum _MEMLCD_Mode {
  memlcdMode_Extcom,
  memlcdMode_Serial
} MEMLCD_Mode;

typedef struct _MEMLCD_Config
{
   /* GPIO connections */
  MEMLCD_Pin sclk;
  MEMLCD_Pin si;
  MEMLCD_Pin scs;
  MEMLCD_Pin extcomin;
  MEMLCD_Pin disp;
  MEMLCD_Pin extmode;

  /* USART module */
  CMU_Clock_TypeDef usartClock;
  USART_TypeDef     *usart;
  unsigned int      usartLocation;

  /* TIMER module used for precise timing */
  CMU_Clock_TypeDef timerClock;
  TIMER_TypeDef     *timer;

  /* Operational mode */
  MEMLCD_Mode mode;

  /* Enable when done */
  bool enable;
} MEMLCD_Config;


#define MEMLCD_CONFIG_STK3700_EXPBOARD   \
{  {gpioPortD, 2},   /* sclk */           \
   {gpioPortD, 0},   /* si   */           \
   {gpioPortD, 3},   /* scs  */           \
   {gpioPortC, 4},  /* extcomin */       \
   {gpioPortD, 5},  /* disp */           \
   {gpioPortD, 4},  /* extmode */        \
   cmuClock_USART1,  /* usartClock */     \
   USART1,           /* usart */          \
   USART_ROUTE_LOCATION_LOC1, /* usartLocation */  \
   cmuClock_TIMER0,  /* timerClock */     \
   TIMER0,           /* timer */          \
   memlcdMode_Extcom,/* mode */           \
   true              /* enable */         \
}


EMSTATUS MEMLCD_Init(MEMLCD_Config *cfg);
void MEMLCD_Enable( bool enable );
void MEMLCD_ComInv( void );
void MEMLCD_Clear( void );
EMSTATUS MEMLCD_Update(uint16_t *data, int firstLine, int lastLine);


void MEMLCD_CreateTestFrameBufferDMA(void);
EMSTATUS MEMLCD_Update_DMA( uint8_t firstline, uint8_t lastline );

#endif /* MEMLCD_H_ */
