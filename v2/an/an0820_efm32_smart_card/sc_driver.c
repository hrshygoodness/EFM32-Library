/******************************************************************************
 * @file sc_driver.c
 * @brief Smart Card Driver for usbccid example
 * @author Silicon Labs
 * @version 1.01
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


#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_timer.h"
#include "em_gpio.h"
#include "em_usb.h"
#include "hw_config.h"
#include "sc_driver.h"
#include "usart_driver.h"
#include "usb_ccid_device.h"

/***************************************************************************//**
 * @brief
 *   Initialize GPIO, TIMER and UART necessary for smart card communication
 *
 ******************************************************************************/
void SC_Init(void){

  /* Enable clocks */
  /* Enable clock for GPIO module */
  CMU_ClockEnable(cmuClock_GPIO, true);
  
  /* Configure Power and ground pins to card. */
  /* Power off */
  SC_PowerOff();
  /* Enable GND */
  GPIO_PinModeSet(SC_GPIO_GND_PORT, SC_GPIO_GND_PIN, gpioModePushPull, 0);
    
  /* Pullup with filter on insertion pin, should be shorted to ground when card is not inserted. */
  GPIO_PinModeSet(SC_GPIO_INSERT_DETECTION_PORT, SC_GPIO_INSERT_DETECTION_PIN, gpioModeInputPullFilter, 1);
  
  /* Configure GPIO for activity LED */
  GPIO_PinModeSet(SC_GPIO_ACTIVITY_LED_PORT, SC_GPIO_ACTIVITY_LED_PIN, gpioModePushPull, 0);
  
  /* Enable clock for TIMER0 module */
  CMU_ClockEnable(SC_TIMER_CMU_CLOCK, true);  
 
  /* Configure timer to output clock signal to smart card. */
  
  /* Select CC channel parameters */
  TIMER_InitCC_TypeDef timerCCInit = 
  {
    .eventCtrl  = timerEventEveryEdge,
    .edge       = timerEdgeBoth,
    .prsSel     = timerPRSSELCh0,
    .cufoa      = timerOutputActionNone,
    .cofoa      = timerOutputActionSet,
    .cmoa       = timerOutputActionClear,
    .mode       = timerCCModePWM,
    .filter     = false,
    .prsInput   = false,
    .coist      = false,
    .outInvert  = false,
  };
  
  /* Configure CC channel 1 */
  TIMER_InitCC(SC_TIMER, 1, &timerCCInit);

  /* Route CC1 to location 3 (PD2) and enable pin */  
  SC_TIMER->ROUTE |= (TIMER_ROUTE_CC1PEN | TIMER_ROUTE_LOCATION_LOC3); 
  
  /* Set Top Value */
  TIMER_TopSet(SC_TIMER, CMU_ClockFreqGet(cmuClock_HFPER)/SC_CLK_FREQ);
  
  /* Set compare value starting at 0 - it will be incremented in the interrupt handler */
  TIMER_CompareBufSet(SC_TIMER, 1,  CMU_ClockFreqGet(cmuClock_HFPER)/SC_CLK_FREQ/2);

  /* Select timer parameters */  
  TIMER_Init_TypeDef timerInit =
  {
    .enable     = false,
    .debugRun   = true,
    .prescale   = timerPrescale1,
    .clkSel     = timerClkSelHFPerClk,
    .fallAction = timerInputActionNone,
    .riseAction = timerInputActionNone,
    .mode       = timerModeUp,
    .dmaClrAct  = false,
    .quadModeX4 = false,
    .oneShot    = false,
    .sync       = false,
  };
  
    
  /* Configure timer */
  TIMER_Init(SC_TIMER, &timerInit);

  /* Assert Reset on card. Pull it low. */
  SC_AssertReset();
  
  
}

/***************************************************************************//**
 * @brief
 *   Turn on smartcard clock
 *
 ******************************************************************************/
void SC_ClockOn(void)
{
  if(!(SC_TIMER->STATUS & TIMER_STATUS_RUNNING)){
    TIMER_CounterSet(SC_TIMER, 0);
  }
  
  GPIO_PinModeSet(SC_GPIO_CLOCK_PORT, SC_GPIO_CLOCK_PIN, gpioModePushPull, 0);
  SC_TIMER->CMD = TIMER_CMD_START;  
}

/***************************************************************************//**
 * @brief
 *   Turn off smartcard clock
 *
 ******************************************************************************/
void SC_ClockOff(void)
{
  if(SC_TIMER->STATUS & TIMER_STATUS_RUNNING){
    while(TIMER_CounterGet(SC_TIMER) <= TIMER_CaptureGet(SC_TIMER, 1));
  }
  
  SC_TIMER->CMD = TIMER_CMD_STOP;
  GPIO_PinModeSet(SC_GPIO_CLOCK_PORT, SC_GPIO_CLOCK_PIN, gpioModeDisabled, 0);
}

/***************************************************************************//**
 * @brief
 *   Turn on smartcard power
 *
 ******************************************************************************/
void SC_PowerOn(void)
{

  /* Vdd */
  GPIO_PinModeSet(SC_GPIO_VDD_PORT, SC_GPIO_VDD_PIN, gpioModePushPull, 1);

}

/***************************************************************************//**
 * @brief
 *   Turn off smartcard power
 *
 ******************************************************************************/
void SC_PowerOff(void)
{
  /* Vdd */
  GPIO_PinModeSet(SC_GPIO_VDD_PORT, SC_GPIO_VDD_PIN, gpioModeDisabled, 0);
}

/***************************************************************************//**
 * @brief
 *   Disable Reset
 *
 ******************************************************************************/
void SC_DisableReset(void)
{
  /* Reset low */
  GPIO_PinModeSet(SC_GPIO_RESET_PORT, SC_GPIO_RESET_PIN, gpioModeDisabled, 0);
}

/***************************************************************************//**
 * @brief
 *   Pull smartcard reset low
 *
 ******************************************************************************/
void SC_AssertReset(void)
{
  /* Reset low */
  GPIO_PinModeSet(SC_GPIO_RESET_PORT, SC_GPIO_RESET_PIN, gpioModePushPull, 0);
}

/***************************************************************************//**
 * @brief
 *   Release smartcard reset to high state
 *
 ******************************************************************************/
void SC_DeassertReset(void)
{
  /* Reset high */
  GPIO_PinModeSet(SC_GPIO_RESET_PORT, SC_GPIO_RESET_PIN, gpioModePushPull, 1);

}

/***************************************************************************//**
 * @brief
 *   Check gpio for smart card insertion
 *
 * @return
 *   1 if smartcard inserted, 0 otherwise.
 ******************************************************************************/    
uint8_t SC_IsCardInserted(void)
{ 
  return (uint8_t) GPIO_PinInGet(SC_GPIO_INSERT_DETECTION_PORT, SC_GPIO_INSERT_DETECTION_PIN);
}

/***************************************************************************//**
 * @brief
 *   Turn activity LED on
 *
 ******************************************************************************/
void SC_LedOn(void)
{
  GPIO_PinOutSet(SC_GPIO_ACTIVITY_LED_PORT, SC_GPIO_ACTIVITY_LED_PIN);
}

/***************************************************************************//**
 * @brief
 *   Turn activity LED off
 *
 ******************************************************************************/
void SC_LedOff(void)
{
  GPIO_PinOutClear(SC_GPIO_ACTIVITY_LED_PORT, SC_GPIO_ACTIVITY_LED_PIN);
}

/***************************************************************************//**
 * @brief
 *   Perform smart card reset sequence and get Answer to Reset
 *
 * @param[out] atr_buf
 *   pointer to buffer for ATR.
 *
 * @param[in] len
 *   max length of ATR to wait for.
 *
 * @param[out] receivedATRLength
 *   pointer to uint32 for actual received ATR length.
 *
 * @return
 *   APDU status enum.
 ******************************************************************************/
uint8_t SC_Reset(uint8_t *atr_buf, uint32_t * receivedATRLength)
{

  uint8_t uartStatus;
  
  /* Perform reset sequence to get ATR as per 7816 smart card spec. */
  SC_PowerOn();
  SC_AssertReset();
  USBTIMER_DelayMs( SC_PWR_ON_DELAY );
  SC_ClockOn();  
  USBTIMER_DelayMs( SC_CLK_DELAY );  
    
  /* Unblock receiver and accept RX before releasing reset. */
  usartBlockReceiver(false);
  usartFlushBuffer();
  usartAcceptRX(true);
  
  SC_DeassertReset();
  /* Wait for ATR to be received. */
  USBTIMER_DelayMs( SC_ATR_DELAY );
    
  uartStatus = usartGetData(atr_buf, 0, receivedATRLength);
  
  if(uartStatus == UART_STATUS_FRAMING_OR_PARITY_ERROR)
  {
    usartFlushBuffer();
    usartAcceptRX(false);
    return APDU_STATUS_XFR_ERROR;    
  }
  
  if(*receivedATRLength < 2)
  {
    usartFlushBuffer();
    usartAcceptRX(false);
    return APDU_STATUS_TIMEOUT;    
  }
  
  usartFlushBuffer();
  usartAcceptRX(false);
  return APDU_STATUS_SUCCESS;
  
}

/***************************************************************************//**
 * @brief
 *   Perform smart card APDU transaction sequence and gets smart card response.
 *   This function operates according to T=0 protocol with standard length APDU.
 *   It expects 5 command bytes and data in/out, then accepts and acts upon 
 *   procedure byte and status byte (SW1 and SW2) from smart card.  
 *
 * @param[in] XfrBlock
 *   pointer to buffer for transfer block which contains APDU to send to card.
 *
 * @param[out] response_abData
 *   Pointer to buffer to write smart card output data to. 
 *
 * @param[out] response_dwLength
 *   pointer to uint32 for actual received response length.
 *
 * @return
 *   APDU status enum.
 ******************************************************************************/
uint8_t SC_SendAPDU(void* XfrBlockStruct, uint8_t* response_abData, uint32_t* response_dwLength){

  uint8_t i, temp_var, CLA, INS, P1, P2, P3, SW1, SW2, procedureByte, writeFlag, uartResult;
  uint16_t ReadDataCount = 0; 
  uint16_t dataLength;
  uint16_t dataSendIndex;
  uint32_t bytesReceived;
  uint16_t msTimeOutCounter = 0;
  
  PC_to_RDR_XfrBlock_BULK_OUT* XfrBlock = (PC_to_RDR_XfrBlock_BULK_OUT*) XfrBlockStruct;
    
  CLA = XfrBlock->abData[0];
  INS = XfrBlock->abData[1];
  P1 = XfrBlock->abData[2];
  P2 = XfrBlock->abData[3];
  P3 = XfrBlock->abData[4];

  /* unused variables, remove warning. */
  (void) P1;
  (void) P2;  
  
  /* Set data index to byte after P3 in XfrBlock. */
  dataSendIndex = 5;
  
  if(CLA == 0xff){
    return APDU_STATUS_INVALID_CMD;
  }
  if((INS >> 4) == 6 || (INS >> 4) == 9){
    return APDU_STATUS_INVALID_INS;  
  }
  
  /* If P3 is 0, transfer length is 256, otherwise it is P3. */
  if(P3 == 0){
    dataLength = 256;
  }else{
    dataLength = P3;
  }
  
    /* If XfrBlock length is greater than 5, this is a write command. */
  if(XfrBlock->dwLength > 5){
    writeFlag = 1;
  }else{
    /* This is a read command. */
    writeFlag = 0;
  }
  
  usartFlushBuffer();
  usartAcceptRX(false);
  
  /* Send instructions to card, same as CLA, INS, P1, P2, P3. */
  /* Return xfr error on error. */
  for(i = 0; i<5; i++)
  {
    uartResult = usartPutChar(XfrBlock->abData[i]);
    if(uartResult != UART_STATUS_SUCCESS)
    {
      usartFlushBuffer();
      usartAcceptRX(false);
      return APDU_STATUS_XFR_ERROR;
    }  
  }
  
  /* Flush uart buffer before changing between TX/RX */
  usartFlushBuffer();
  usartAcceptRX(true);
  
  /* Please see an0820 application note document for more thorough documentation of this while-loop. */
  /* Upon any error-condition it will just return immediately with error code. */
  while(1)
  {
    /* Wait until the reception of one byte is successful, return on any error. */ 
    do
    {
      USBTIMER_DelayMs(1);
      uartResult = usartGetChar(&procedureByte);
      
      if(uartResult == UART_STATUS_FRAMING_OR_PARITY_ERROR)
      {
        usartFlushBuffer();
        usartAcceptRX(false);
        return APDU_STATUS_XFR_ERROR;
      }
      
      msTimeOutCounter++;
      if(msTimeOutCounter > SC_CMD_TIMEOUT)
      {
        usartFlushBuffer();
        usartAcceptRX(false);
        return APDU_STATUS_TIMEOUT;      
      }
    }
    while(uartResult != UART_STATUS_SUCCESS);
        
    /* Check what the procedure byte is and act accordingly, see 7816-standard, part 3. */   
    if(procedureByte == 0x60){
      /* Null character, just wait for new procedure byte. */
      continue;
    }
    
    if((procedureByte & 0xf0) == 0x60 || (procedureByte & 0xf0) == 0x90)
    {
      /* No datatransfer will take place, just fetch SW2 and return. SW1 indicate error condition.  */
      SW1 = procedureByte;
      
      /* Wait for SW2 until the reception of one byte is successful, return on any error. */ 
      do
      {
        USBTIMER_DelayMs(1);
        uartResult = usartGetChar(&SW2);
        
        if(uartResult == UART_STATUS_FRAMING_OR_PARITY_ERROR)
        {
          usartFlushBuffer();
          usartAcceptRX(false);
          return APDU_STATUS_XFR_ERROR;
        }
                
        msTimeOutCounter++;
        if(msTimeOutCounter > SC_CMD_TIMEOUT)
        {
          usartFlushBuffer();
          usartAcceptRX(false);
          return APDU_STATUS_TIMEOUT;      
        }
      }
      while(uartResult != UART_STATUS_SUCCESS);
      
      
      response_abData[ReadDataCount + 0] = SW1;
      response_abData[ReadDataCount + 1] = SW2;
      *response_dwLength = ReadDataCount + 2;
        
      usartFlushBuffer();
      usartAcceptRX(false);
      
      return SW1;
                 
    }
    /* Check what the procedure byte is and act accordingly, see 7816-standard, part 3. */ 
    if(procedureByte == INS){
      /* send remaining bytes or receive them, and wait for new procedure byte. */
      
      if(writeFlag){
        /* Send data to card. */
        usartFlushBuffer();
        usartAcceptRX(false);
        uartResult = usartPutData(&(XfrBlock->abData[dataSendIndex]) , dataLength);
        
        if(uartResult != UART_STATUS_SUCCESS)
        {
          usartFlushBuffer();
          usartAcceptRX(false);
          return APDU_STATUS_XFR_ERROR;
        }          
        
        dataSendIndex += dataLength; 
        dataLength = 0;
        
        usartFlushBuffer();
        usartAcceptRX(true);
        
      }
      else
      {
        /* Read data from card */
           
        do
        {
          USBTIMER_DelayMs(1);
          uartResult = usartGetData(response_abData + ReadDataCount, dataLength, &bytesReceived);
          
          if(uartResult == UART_STATUS_FRAMING_OR_PARITY_ERROR)
          {
            usartFlushBuffer();
            usartAcceptRX(false);
            return APDU_STATUS_XFR_ERROR;
          }          
          
          msTimeOutCounter++;
          if(msTimeOutCounter > SC_CMD_TIMEOUT)
          {
            usartFlushBuffer();
            usartAcceptRX(false);
            return APDU_STATUS_TIMEOUT;      
          }
        }
        while(uartResult != UART_STATUS_SUCCESS);
                    
        dataLength = 0;
        
        ReadDataCount += bytesReceived;
        
      }
            
      
       /* Then wait for new procedure byte. */
      continue;
    }
    
    /* Check what the procedure byte is and act accordingly, see 7816-standard, part 3. */ 
    /* Fix issue with gcc that causes compare warning, use temp variable. */
    temp_var = INS ^ (uint8_t)0xff;
    if(procedureByte == temp_var)
    {
      if(dataLength == 0){
        usartFlushBuffer();
        usartAcceptRX(false);
        /* Shouldn't be here with zero bytes to send/receive. */
        return APDU_STATUS_INVALID_PROCEDURE_BYTE;
      }
      
      /* Send/receive one byte in this case, then wait for procedure byte. */
      if(writeFlag){
        /* Send data to card. */
        usartFlushBuffer();
        usartAcceptRX(false);
        uartResult = usartPutChar(XfrBlock->abData[dataSendIndex]);
        
        if(uartResult != UART_STATUS_SUCCESS)
        {
          usartFlushBuffer();
          usartAcceptRX(false);
          return APDU_STATUS_XFR_ERROR;
        }         
        
        dataSendIndex++;
        dataLength--;
        
        usartFlushBuffer();
        usartAcceptRX(true);
       
      }else{
          
        do
        {
          USBTIMER_DelayMs(1);
          uartResult = usartGetChar(response_abData + ReadDataCount);
          
          if(uartResult == UART_STATUS_FRAMING_OR_PARITY_ERROR)
          {
            usartFlushBuffer();
            usartAcceptRX(false);
            return APDU_STATUS_XFR_ERROR;
          }            
          
          msTimeOutCounter++;
          if(msTimeOutCounter > SC_CMD_TIMEOUT)
          {
            usartFlushBuffer();
            usartAcceptRX(false);
            return APDU_STATUS_TIMEOUT;      
          }
        }
        while(uartResult != UART_STATUS_SUCCESS);        
            
        dataLength--;
        
        ReadDataCount += 1;        

      }
      
              
       /* Then wait for new procedure byte. */
      continue;
    }
 
    usartFlushBuffer();
    usartAcceptRX(false);
    return APDU_STATUS_INVALID_PROCEDURE_BYTE;
  }
  
}
