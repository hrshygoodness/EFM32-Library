/**************************************************************************//**
 * @file hijack.c
 * @brief Hijack demo for EFM32TG-STK3300
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

#include <stdio.h>
/* Chip specific header file(s). */
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_timer.h"
#include "em_int.h"
/* Driver header file(s). */
#include "segmentlcd.h"

/* Module header file. */
#include "hijack.h"


/*******************************************************************************
 *******************************   MACROS   ************************************
 ******************************************************************************/

#define HIJACK_RX_SHORTINTERVAL     (56)
#define HIJACK_RX_LONGINTERVAL      (150)
#define HIJACK_TX_INTERVAL          (45)

#define HIJACK_TIMER_RESOLUTION     timerPrescale256


#define HIJACK_TX_TIMER             TIMER1
#define HIJACK_TX_TIMERCLK          cmuClock_TIMER1
#define HIJACK_TX_GPIO_PORT         gpioPortD
#define HIJACK_TX_GPIO_PIN          6

#define HIJACK_RX_TIMER             TIMER0
#define HIJACK_RX_TIMERCLK          cmuClock_TIMER0
#define HIJACK_RX_GPIO_PORT         gpioPortD
#define HIJACK_RX_GPIO_PIN          1
/*******************************************************************************
 ********************************   ENUMS   ************************************
 ******************************************************************************/

/* HiJack states. */
typedef enum
{
  STARTBIT = 0U,
  STARTBIT_FALL,
  DECODE,
  STOPBIT,
  BYTE,
  IDLE,
} HIJACK_State_t;


/*******************************************************************************
 ******************************   TYPEDEFS   ***********************************
 ******************************************************************************/


/*******************************************************************************
 ******************************   CONSTANTS   **********************************
 ******************************************************************************/


/*******************************************************************************
 *******************************   STATICS   ***********************************
 ******************************************************************************/

static volatile HIJACK_State_t txState = IDLE;
static volatile uint8_t        txBit;
static volatile uint8_t        sendingByteTx;
static HIJACK_TxDoneFuncPtr_t pTxDoneFunc = NULL;

/*******************************************************************************
 ***************************   LOCAL FUNCTIONS   *******************************
 ******************************************************************************/

static void HIJACK_ManchesterDecoder(uint32_t time);
static void HIJACK_CaptureConfig(HIJACK_EdgeMode_t edgeMode);
static void HIJACK_CompareConfig(HIJACK_OutputMode_t outputMode);
/*******************************************************************************
 ***************************   GLOBAL FUNCTIONS   ******************************
 ******************************************************************************/

/***************************************************************************//**
 * @brief
 *   Initialize HiJack.
 ******************************************************************************/
void HIJACK_Init(HIJACK_TxDoneFuncPtr_t pTxDone)
{
  static const TIMER_Init_TypeDef rxTimerInit =
  { false,                  /* Don't enable timer when init complete. */
    false,                  /* Stop counter during debug halt. */
    HIJACK_TIMER_RESOLUTION,/* ... */
    timerClkSelHFPerClk,    /* Select HFPER clock. */
    false,                  /* Not 2x count mode. */
    false,                  /* No ATI. */
    timerInputActionNone,   /* No action on falling input edge. */
    timerInputActionNone,   /* No action on rising input edge. */
    timerModeUp,            /* Up-counting. */
    false,                  /* Do not clear DMA requests when DMA channel is active. */
    false,                  /* Select X2 quadrature decode mode (if used). */
    false,                  /* Disable one shot. */
    false                   /* Not started/stopped/reloaded by other timers. */
  };

  static const TIMER_Init_TypeDef txTimerInit =
  { false,                  /* Don't enable timer when init complete. */
    false,                  /* Stop counter during debug halt. */
    HIJACK_TIMER_RESOLUTION,/* ... */
    timerClkSelHFPerClk,    /* Select HFPER clock. */
    false,                  /* Not 2x count mode. */
    false,                  /* No ATI. */
    timerInputActionNone,   /* No action on falling input edge. */
    timerInputActionNone,   /* No action on rising input edge. */
    timerModeUp,            /* Up-counting. */
    false,                  /* Do not clear DMA requests when DMA channel is active. */
    false,                  /* Select X2 quadrature decode mode (if used). */
    false,                  /* Disable one shot. */
    false                   /* Not started/stopped/reloaded by other timers. */
  };

  /* Store Tx Done callback function pointer */
   pTxDoneFunc = pTxDone;
  /* Ensure core frequency has been updated */
  SystemCoreClockUpdate();

  /* Enable peripheral clocks. */
  CMU_ClockEnable(cmuClock_HFPER, true);
  CMU_ClockEnable(HIJACK_RX_TIMERCLK, true); 
  CMU_ClockEnable(HIJACK_TX_TIMERCLK, true); 

  /* Configure Rx timer. */
  TIMER_Init(HIJACK_RX_TIMER, &rxTimerInit);
  /* Configure Rx timer. */
  TIMER_Init(HIJACK_TX_TIMER, &txTimerInit);

  /* Configure Rx timer input capture channel 0. */
  HIJACK_CaptureConfig(hijackEdgeModeRising);
  /* Configure Tx timer output compare channel 0. */
  HIJACK_CompareConfig(hijackOutputModeSet);
  TIMER_CompareSet(HIJACK_TX_TIMER, 0, HIJACK_TX_INTERVAL);

  /* Route the capture channels to the correct pins, enable CC0. */
  HIJACK_RX_TIMER->ROUTE = TIMER_ROUTE_LOCATION_LOC3 | TIMER_ROUTE_CC0PEN;
  /* Route the capture channels to the correct pins, enable CC0. */
  HIJACK_TX_TIMER->ROUTE = TIMER_ROUTE_LOCATION_LOC4 | TIMER_ROUTE_CC0PEN;

  /* Rx: Configure the corresponding GPIO pin (PortD, Ch1) as an input. */
  GPIO_PinModeSet(HIJACK_RX_GPIO_PORT, HIJACK_RX_GPIO_PIN, gpioModeInput, 0);  
  /* Tx: Configure the corresponding GPIO pin (PortD, Ch6) as an input. */
  GPIO_PinModeSet(HIJACK_TX_GPIO_PORT, HIJACK_TX_GPIO_PIN, gpioModePushPull, 0);  


  
  /* Enable Rx timer CC0 interrupt. */
  NVIC_EnableIRQ(TIMER0_IRQn);
  TIMER_IntEnable(HIJACK_RX_TIMER, TIMER_IF_CC0);

  /* Enable Tx timer CC0 interrupt. */
  NVIC_EnableIRQ(TIMER1_IRQn);
  TIMER_IntEnable(HIJACK_TX_TIMER, TIMER_IF_CC0);

  /* Enable the timer. */
  TIMER_Enable(HIJACK_RX_TIMER, true);
  TIMER_Enable(HIJACK_TX_TIMER, true);
}


/***************************************************************************//**
 * @brief
 *   Transmit a byte over the HiJack interface.
 ******************************************************************************/
bool HIJACK_ByteTx(uint8_t byte)
{
  if (txState != IDLE)
  {
    return false;
  }
  INT_Disable();
  txState = STARTBIT;
  sendingByteTx = byte;
  txBit = 0;
  INT_Enable();
  return true;

}


/***************************************************************************//**
 * @brief
 *   Receive a byte over the HiJack interface.
 ******************************************************************************/
void HIJACK_ByteRx(uint8_t *pByte)
{
  /* Write the received value on the LCD. */
  SegmentLCD_Number(*pByte);
}


/***************************************************************************//**
 * @brief
 *   Manchester decoder
 ******************************************************************************/
  static HIJACK_State_t state = STARTBIT;
  static uint32_t ones = 0;
static void HIJACK_ManchesterDecoder(uint32_t time)
{
  static uint32_t lastTime;
  static uint32_t bitCounter = 0;
  static uint8_t  uartByteRx = 0;
  uint32_t diff;


  /* Calculate the difference between the last time and the current time. */
  diff = time - lastTime;

  /* Decoder state machine. */
  switch(state)
  {
  case STARTBIT:
    /* Configure for falling edge. */
    HIJACK_CaptureConfig(hijackEdgeModeFalling);
    state = STARTBIT_FALL;
    break;

  case STARTBIT_FALL:    
    if ((HIJACK_RX_SHORTINTERVAL < diff) && (diff < HIJACK_RX_LONGINTERVAL))
    {
      if (ones < 3)
      {
        /* We didn't have enough ones. */
        ones = 0;
        HIJACK_CaptureConfig(hijackEdgeModeRising);
        state = STARTBIT;
      }
      else
      {
        /* Looks like we got a 1->0 transition. Its needed to detect for both
         * edges from now. */
        HIJACK_CaptureConfig(hijackEdgeModeBoth);
        bitCounter = 0;
        uartByteRx = 0;

        state = DECODE;
      }
    }
    else
    {
      /* No, we have to search again. */
      HIJACK_CaptureConfig(hijackEdgeModeRising);
      state = STARTBIT;
      if(diff < HIJACK_RX_SHORTINTERVAL)
      {
        /* Count the number of shorts for robustness. */
        ones++;
      }
    }
    break;

  case DECODE:
    if ((HIJACK_RX_SHORTINTERVAL < diff) && (diff < HIJACK_RX_LONGINTERVAL))
    {
      if (bitCounter >= 8)
      {
        /* We got the whole byte, output stop bit and search for startbit. */
        HIJACK_CaptureConfig(hijackEdgeModeRising);
                
        /* Pass the byte to the Rx function. Note: queue shall be added later! */
        HIJACK_ByteRx(&uartByteRx);

        state = STARTBIT;
        ones = 0;
        return;
      }

      /* Check what transition it was. */
      if (GPIO_PinInGet(HIJACK_RX_GPIO_PORT, HIJACK_RX_GPIO_PIN))
      {
        /* We read a 1. */
        uartByteRx = (uartByteRx >> 1) + (1 << 7);
      }
      else
      {
        /* We got a 0. */
        uartByteRx = (uartByteRx >> 1);
      }

      bitCounter++;
    }
    else if (diff >= HIJACK_RX_LONGINTERVAL)
    {
      /* Something is wrong, start search again. */
      HIJACK_CaptureConfig(hijackEdgeModeRising);
      state = STARTBIT;
      ones = 0;
    }
    else
    {
      /* Return here and don't update the time! */
      return;
    }
    break;

  default:
    break;
  }

  lastTime = time;
}

/***************************************************************************//**
 * @brief
 *   Configure the capture channel for the HiJack.
 ******************************************************************************/
static void HIJACK_CaptureConfig(HIJACK_EdgeMode_t edgeMode)
{
  TIMER_InitCC_TypeDef rxTimerCapComChConf =
  { timerEventEveryEdge,      /* Event on every capture. */
    timerEdgeRising,          /* Input capture edge on rising edge. */
    timerPRSSELCh0,           /* Not used by default, select PRS channel 0. */
    timerOutputActionNone,    /* No action on underflow. */
    timerOutputActionNone,    /* No action on overflow. */
    timerOutputActionNone,    /* No action on match. */
    timerCCModeCapture,       /* Configure capture channel. */
    false,                    /* Disable filter. */
    false,                    /* Select TIMERnCCx input. */
    false,                    /* Clear output when counter disabled. */
    false                     /* Do not invert output. */
  };


  if (hijackEdgeModeRising == edgeMode)
  {
    rxTimerCapComChConf.edge = timerEdgeRising;
  }
  else if (hijackEdgeModeFalling == edgeMode)
  {
    rxTimerCapComChConf.edge = timerEdgeFalling;
  }
  else if (hijackEdgeModeBoth == edgeMode)
  {
    rxTimerCapComChConf.edge = timerEdgeBoth;
  }
  else
  {
    /* Config error. */
    rxTimerCapComChConf.edge = timerEdgeNone;
  }

  TIMER_InitCC(HIJACK_RX_TIMER, 0, &rxTimerCapComChConf);
}

/***************************************************************************//**
 * @brief
 *   Configure the compare channel for the HiJack.
 ******************************************************************************/
static void HIJACK_CompareConfig(HIJACK_OutputMode_t outputMode)
{
  TIMER_InitCC_TypeDef txTimerCapComChConf =
  { timerEventEveryEdge,      /* Event on every capture. */
    timerEdgeRising,          /* Input capture edge on rising edge. */
    timerPRSSELCh0,           /* Not used by default, select PRS channel 0. */
    timerOutputActionNone,    /* No action on underflow. */
    timerOutputActionNone,    /* No action on overflow. */
    timerOutputActionSet,     /* No action on match. */
    timerCCModeCompare,       /* Configure capture channel. */
    false,                    /* Disable filter. */
    false,                    /* Select TIMERnCCx input. */
    true,                     /* Output high when counter disabled. */
    false                     /* Do not invert output. */
  };


  if (hijackOutputModeSet == outputMode)
  {
    txTimerCapComChConf.cmoa = timerOutputActionSet;
  }
  else if (hijackOutputModeClear == outputMode)
  {
    txTimerCapComChConf.cmoa = timerOutputActionClear;
  }
  else if (hijackOutputModeToggle == outputMode)
  {
    txTimerCapComChConf.cmoa = timerOutputActionToggle;
  }
  else
  {
    /* Config error. */
    txTimerCapComChConf.cmoa = timerOutputActionNone;
  }

  TIMER_InitCC(HIJACK_TX_TIMER, 0, &txTimerCapComChConf);
}

/***************************************************************************//**
 * @brief
 *   Timer0 IRQHandler.
 ******************************************************************************/
void TIMER0_IRQHandler(void)
{  
  uint32_t irqFlags;

  irqFlags = TIMER_IntGet(HIJACK_RX_TIMER);
  TIMER_IntClear(HIJACK_RX_TIMER, irqFlags);

  if (TIMER_IF_CC0 & irqFlags)
  {
    HIJACK_ManchesterDecoder(TIMER_CaptureGet(HIJACK_RX_TIMER, 0));
  }
}


/***************************************************************************//**
 * @brief
 *   Timer1 IRQHandler.
 ******************************************************************************/
void TIMER1_IRQHandler(void)
{
  static volatile uint8_t currentPin = 1;
  static volatile uint8_t currentSym = 1;
  static volatile uint8_t txParity = 0;
  uint8_t tmp;
  uint32_t irqFlags;

  /* Clear all pending IRQ flags. */
  irqFlags = TIMER_IntGet(HIJACK_TX_TIMER);
  TIMER_IntClear(HIJACK_TX_TIMER, irqFlags);

  /* Reset the counter and the compare value. */
  TIMER_CounterSet(HIJACK_TX_TIMER, 0);
  TIMER_CompareSet(HIJACK_TX_TIMER, 0, HIJACK_TX_INTERVAL);

  if (currentPin == 1)
  {
    /* First iteration check for symbol. */
    if( currentSym == 0 )
    {
      /* Have to set the output pin */
      HIJACK_CompareConfig(hijackOutputModeSet);
      currentPin = 2;
    }
    else
    {
      /* Have to reset the pin */
      HIJACK_CompareConfig(hijackOutputModeClear);
      currentPin = 2;
    }
  }
  else
  {
    /* Second time, just toggle the pin */
    HIJACK_CompareConfig(hijackOutputModeToggle);


    currentPin = 1;

    switch(txState)
    {
    case STARTBIT:
      currentSym = 0;
      txState = BYTE;
      txBit = 0;
      txParity = 0;
      break;

    case BYTE:

      if (txBit < 8)
      {
        uint8_t tempsendingByteTx = sendingByteTx;
        currentSym = (tempsendingByteTx >> txBit) & 0x01;
        txBit++;
        tmp = txParity;
        txParity = tmp + currentSym;
      }
      else if (txBit == 8)
      {
        currentSym = txParity & 0x01;
        txBit++;
      }
      else if (txBit > 8)
      {
        /* Next bit is the stop bit. */
        currentSym = 1;
        txState = STOPBIT;
      }
      break;

    case STOPBIT:
      /* This is where an application can be signaled that transmit is done. */ 
      if (pTxDoneFunc != NULL)
      {
        pTxDoneFunc();
      }
    case IDLE:
      currentSym = 1;
      txState = IDLE;
      break;

    /* These states are not used in TX and default is to do nothing. */
    case STARTBIT_FALL:
    case DECODE:
    default:
      break;
    }
  }
}
