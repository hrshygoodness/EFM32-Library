/*****************************************************************************
 * @file i2c_master.c
 * @brief DMA I2C Master Example
 * @author Silicon Labs
 * @version 2.06
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

#include <stdbool.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_gpio.h"
#include "em_dma.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_int.h"
#include "em_i2c.h"
#include "dmactrl.h"
#include "bsp.h"

/* The I2C read bit is OR'ed with the address for a read operation */
#define I2C_READ_BIT 0x01

/* DMA channels */
#define DMA_CHANNEL_I2C_TX 0
#define DMA_CHANNEL_I2C_RX 1

/* The I2C address of the EEPROM */
#define EEPROM_I2C_ADDR 0xA0

/* I2C configuration. Corresponds to the I2C bus on the EFM32GG-DK3750. */
#define I2C_SCL_PORT gpioPortD
#define I2C_SCL_PIN  15
#define I2C_SDA_PORT gpioPortD
#define I2C_SDA_PIN  14
#define I2C_LOCATION 3

/* DMA callback structure */
static DMA_CB_TypeDef dmaCallback;

/* Transfer flag */
static volatile bool transferActive = false;

/* I2C error flag */
static volatile bool i2cError = false;

/* Transmit data */
static const uint8_t txData[] = { 0x1, 0x2, 0x3, 0x4, 0x5 };

/* Receive buffer */
static uint8_t rxData[sizeof(txData)];

/* Helper variables. These are used by I2C interrupt handler to 
 * retrieve the last bytes when doing a DMA read. */
static uint8_t *rxPointer;
static uint8_t bytesLeft;


/*****************************************************************************
 * @brief  Called when DMA transfer is complete
 * Enables interrupts to finish the transfer.
 *****************************************************************************/
void transferComplete(unsigned int channel, bool primary, void *user)
{
  /* Ignore unused parameters */
  (void) primary;
  (void) user;
 
  if ( channel == DMA_CHANNEL_I2C_TX )
  {
    /* Enable MSTOP interrupt */
    I2C0->IEN |= I2C_IEN_MSTOP;
  }
  else if ( channel == DMA_CHANNEL_I2C_RX )
  {
    /* Stop automatick ACK'ing bytes */
    I2C0->CTRL &= ~I2C_CTRL_AUTOACK;
    
    /* Enable RX and MSTOP interrupt */
    I2C0->IEN |= I2C_IEN_RXDATAV | I2C_IEN_MSTOP;
  }
}

/*****************************************************************************
 * @brief  Aborts the current transfer and sets the error flag
 *****************************************************************************/
void i2cErrorAbort(void)
{
  I2C0->CMD = I2C_CMD_ABORT;
  transferActive = false;
  i2cError = true;  
}

/*****************************************************************************
 * @brief  Handles various I2C events and errors
 * 
 * When a STOP condition has been successfully sent, the MSTOP
 * interrupt is triggered and the transfer is marked as complete. 
 * 
 * When receiving with DMA, the last two bytes must be fetched 
 * manually by software. They are fetched by listening for the
 * RXDATAV interrupt. The last byte should per the I2C standard
 * be NACK'ed, to inform the slave that it is the last byte of
 * the transfer. 
 * 
 * The three errors: ARBLOST, BUSERR and CLTO are handled here. 
 * In all cases, the current transfer is aborted, and the error
 * flag is set to inform the main loop that an error occured
 * during the transfer. 
 * 
 *****************************************************************************/
void I2C0_IRQHandler(void)
{
  uint32_t flags = I2C0->IF;
  
  if ( flags & (I2C_IF_ARBLOST | I2C_IF_BUSERR | I2C_IF_CLTO) )
  {
    I2C0->IFC = flags;
    i2cErrorAbort();
  }
  else if ( flags & I2C_IF_MSTOP )
  {
    /* Stop condition has been sent. Transfer is complete. */
    I2C0->IFC = I2C_IFC_MSTOP;
    I2C0->IEN &= ~I2C_IEN_MSTOP;
    transferActive = false;
        
    /* Clear AUTOSE if set */
    if ( I2C0->CTRL & I2C_CTRL_AUTOSE )
    {
      I2C0->CTRL &= ~I2C_CTRL_AUTOSE;
    }
  }
  else if ( I2C0->IF & I2C_IF_RXDATAV )
  {
    /* Read out the last two bytes here. Reading RXDATA
     * clears the interrupt flag. */
    *rxPointer++ = I2C0->RXDATA;
    
    if ( --bytesLeft == 0 )
    {
      /* Transfer is complete, NACK last byte and send STOP condition */
      I2C0->CMD = I2C_CMD_NACK;
      I2C0->IEN &= ~I2C_IEN_RXDATAV;
      I2C0->CMD = I2C_CMD_STOP;
    }
    else
    {
      /* ACK the second last byte */
      I2C0->CMD = I2C_CMD_ACK;
    }
  }
}

/*****************************************************************************
 * @brief Configure DMA to send and receive over I2C
 *****************************************************************************/
void dmaInit(void)
{
  CMU_ClockEnable(cmuClock_DMA, true);  
  
  DMA_Init_TypeDef        dmaInit;
  DMA_CfgChannel_TypeDef  txChannelConfig;
  DMA_CfgDescr_TypeDef    txDescriptorConfig;
  DMA_CfgChannel_TypeDef  rxChannelConfig;
  DMA_CfgDescr_TypeDef    rxDescriptorConfig;
  
  /* Initializing the DMA */
  dmaInit.hprot        = 0;
  dmaInit.controlBlock = dmaControlBlock;
  DMA_Init(&dmaInit);
  
  /* Setup call-back function */  
  dmaCallback.cbFunc  = transferComplete;
  dmaCallback.userPtr = NULL;

  /* Setting up TX channel */
  txChannelConfig.highPri   = false;
  txChannelConfig.enableInt = true;
  txChannelConfig.select    = DMAREQ_I2C0_TXBL;
  txChannelConfig.cb        = &dmaCallback;
  DMA_CfgChannel(DMA_CHANNEL_I2C_TX, &txChannelConfig);

  /* Setting up TX channel descriptor */
  txDescriptorConfig.dstInc  = dmaDataIncNone;
  txDescriptorConfig.srcInc  = dmaDataInc1;
  txDescriptorConfig.size    = dmaDataSize1;
  txDescriptorConfig.arbRate = dmaArbitrate1;
  txDescriptorConfig.hprot   = 0;
  DMA_CfgDescr(DMA_CHANNEL_I2C_TX, true, &txDescriptorConfig);
  
  /* Setting up RX channel */
  rxChannelConfig.highPri   = false;
  rxChannelConfig.enableInt = true;
  rxChannelConfig.select    = DMAREQ_I2C0_RXDATAV;
  rxChannelConfig.cb        = &dmaCallback;
  DMA_CfgChannel(DMA_CHANNEL_I2C_RX, &rxChannelConfig);

  /* Setting up RX channel descriptor */
  rxDescriptorConfig.dstInc  = dmaDataInc1;
  rxDescriptorConfig.srcInc  = dmaDataIncNone;
  rxDescriptorConfig.size    = dmaDataSize1;
  rxDescriptorConfig.arbRate = dmaArbitrate1;
  rxDescriptorConfig.hprot   = 0;
  DMA_CfgDescr(DMA_CHANNEL_I2C_RX, true, &rxDescriptorConfig);
}


/*****************************************************************************
 * @brief  Initialize I2C master
 *****************************************************************************/
void i2cInit(void)
{
  CMU_ClockEnable(cmuClock_I2C0, true);
  CMU_ClockEnable(cmuClock_GPIO, true);

  GPIO_PinModeSet(I2C_SCL_PORT, I2C_SCL_PIN, gpioModeWiredAnd, 1);
  GPIO_PinModeSet(I2C_SDA_PORT, I2C_SDA_PIN, gpioModeWiredAnd, 1);
  I2C0->ROUTE = I2C_ROUTE_SDAPEN | I2C_ROUTE_SCLPEN |
                (I2C_LOCATION << _I2C_ROUTE_LOCATION_SHIFT);
  
  I2C_Init_TypeDef i2cInit = I2C_INIT_DEFAULT;  
  I2C_Init(I2C0, &i2cInit);
  
  
  /* Exit the BUSY state. The I2C will be in this state out of RESET. */
  if (I2C0->STATE & I2C_STATE_BUSY)
  {
    I2C0->CMD = I2C_CMD_ABORT;
  }
  
  /* Enable the Clock Low Timeout counter */
  I2C0->CTRL = (I2C0->CTRL & ~_I2C_CTRL_CLTO_MASK) | I2C_CTRL_CLTO_160PCC;
  
  /* Enable error interrupts */
  I2C0->IEN |= I2C_IEN_ARBLOST | I2C_IEN_BUSERR | I2C_IEN_CLTO;
  
  /* Enable interrupts in NVIC */
  NVIC_EnableIRQ(I2C0_IRQn);
}


/*****************************************************************************
 * @brief  Waits for ACK or NACK response. Returns true on ACK, false on
 * NACK or I2C error.
 *****************************************************************************/
bool i2cWaitForAckNack(void)
{
  while ( !i2cError )
  {
    if ( I2C0->IF & I2C_IF_ACK )
    {
      I2C0->IFC = I2C_IFC_ACK;  
      return true;
    }
    else if ( I2C0->IF & I2C_IF_NACK )
    {
      I2C0->IFC = I2C_IFC_NACK;
      return false;
    }
  }
  
  /* I2C error has occurred */
  return false;
}
  

/*****************************************************************************
 * @brief  Sleep in EM1 until DMA transfer is done
 *****************************************************************************/
void sleepUntilTransferDone(void)
{
  /* Enter EM1 while DMA transfer is active to save power. Note that
   * interrupts are disabled to prevent the ISR from being triggered
   * after checking the transferActive flag, but before entering
   * sleep. If this were to happen, there would be no interrupt to wake
   * the core again and the MCU would be stuck in EM1. While the 
   * core is in sleep, pending interrupts will still wake up the 
   * core and the ISR will be triggered after interrupts are enabled
   * again. 
   */   
  while(1)
  {
    INT_Disable();
    if ( transferActive )
    {
      EMU_EnterEM1(); 
    }
    INT_Enable();
    
    /* Exit the loop if transfer has completed */
    if ( !transferActive )
    {
      break;
    }
  }  
}

/*****************************************************************************
 * @brief  Performs 'ACK polling' of the EEPROM. When the EEPROM device is busy
 * writing its write buffer to EEPROM it will NACK all requests until it 
 * is complete. This function retries a request until it receives an ACK. 
 *****************************************************************************/
void ackPoll(uint8_t deviceAddress)
{
  bool done = false;
  uint32_t flags;
      
  /* Clear any previously set flags before starting transfer */
  I2C0->IFC = _I2C_IFC_MASK;

  /* Retry sending a write request until receiving an ACK or error */
  while ( !done && !i2cError )
  {
    I2C0->TXDATA = deviceAddress; 
    I2C0->CMD = I2C_CMD_START;
    
    while ( 1 )
    {
      flags = I2C0->IF;
      
      if ( i2cError )
      {
        /* Abort on I2C error */
        return;
      }
      else if ( flags & I2C_IF_NACK )
      {
        I2C0->IFC = I2C_IFC_NACK;
        break;
      }
      else if ( flags & I2C_IF_ACK )
      {
        /* ACK received, device is ready */
        I2C0->CMD = I2C_CMD_STOP;
        done = true;
        break;
      }
    }
  }
  
  /* Wait for and clear the MSTOP flag */
  while ( !(I2C0->IF & I2C_IF_MSTOP) && !i2cError );
  I2C0->IFC = I2C_IFC_MSTOP;
}

/*****************************************************************************
 * @brief  Writes one byte to the I2C slave at the given device address. 
 * This function does not send or wait for the STOP condition and is
 * meant as a helper function to send the EEPROM offset before 
 * reading/writing with DMA. 
 *****************************************************************************/
void i2cWriteByte(uint8_t deviceAddress, uint8_t data)
{
  /* Abort if an error has been detected */
  if ( i2cError )
  {
    return;
  }  

  /* Clear any previously set flags before starting transfer */
  I2C0->IFC = _I2C_IFC_MASK;
  
  /* Send the address */
  I2C0->TXDATA     = deviceAddress;
  I2C0->CMD        = I2C_CMD_START;
  
  if ( !i2cWaitForAckNack() )
  {
    i2cErrorAbort();
    return;
  }
  
  /* Send the data */
  I2C0->TXDATA = data;
  
  if ( !i2cWaitForAckNack() )
  {
    i2cErrorAbort();
    return;
  }
}


/*****************************************************************************
 * @brief  Writes bytes to I2C EEPROM using DMA. 
 * 
 * @param deviceAddress 
 *      I2C address of EEPROM
 * 
 * @param offset
 *      The offset (address) to start writing from
 * 
 * @param data
 *      Pointer to the data buffer
 * 
 * @param length
 *      Number of bytes to write
 *****************************************************************************/
void i2cDmaWrite(uint8_t deviceAddress, uint8_t offset, uint8_t *data, uint8_t length)
{ 
  /* Abort if an error has been detected */
  if ( i2cError )
  {
    return;
  }
  
  /* Send address to write to. Note that this is specific to the EEPROM on the 
   * EFM32GG-DK3750 and may need to be changed when using a different device. */
  i2cWriteByte(deviceAddress, offset);  
  
  /* Abort if an error has been detected */
  if ( i2cError )
  {
    return;
  }

  /* Automatically generate a STOP condition when there is no
   * more data to send. The DMA must be fast enough to 
   * keep up (normally not a problem unless the DMA is
   * been prescaled). */
  I2C0->CTRL |= I2C_CTRL_AUTOSE;
   
  /* Set transfer active flag. Cleared by interrupt handler
   * when STOP condition has been sent. */
  transferActive = true;
  
  /* Activate DMA */
  DMA_ActivateBasic(DMA_CHANNEL_I2C_TX,         /* TX DMA channel */
                    true,                       /* Primary descriptor */
                    false,                      /* No burst */
                    (void *)&(I2C0->TXDATA),    /* Write to TXDATA */
                    (void *)data,               /* Read from txBuffer */
                    length - 1 );               /* Number of transfers */
  
}


/*****************************************************************************
 * @brief  Reads from I2C EEPROM using DMA. 
 * 
 * @param deviceAddress 
 *      I2C address of EEPROM
 * 
 * @param offset
 *      The offset (address) to start reading from
 * 
 * @param data
 *      Pointer to the data buffer
 * 
 * @param length
 *      Number of bytes to read
 *****************************************************************************/
void i2cDmaRead(uint8_t deviceAddress, uint8_t offset, uint8_t *data, uint8_t length)
{ 
  /* Wait for any previous transfer to finish */
  while ( transferActive )
  {
    EMU_EnterEM1();
  }
  
  /* Abort if an error has occured */
  if ( i2cError )
  {
    return;
  }
 
  /* Clear all pending interrupts prior to starting transfer. */
  I2C0->IFC = _I2C_IFC_MASK;

  /* Write address to read from. Note that this is specific to the EEPROM on the 
   * EFM32GG-DK3750 and may need to be changed when using a different device. */
  i2cWriteByte(deviceAddress, offset);
    
  /* Send the device address. I2C_CMD_START must be written before
   * TXDATA since this is a repeated start.  */
  I2C0->CMD        = I2C_CMD_START;
  I2C0->TXDATA     = deviceAddress | I2C_READ_BIT;
  
  /* Wait for ACK on the address */
  if ( !i2cWaitForAckNack() )
  {
    i2cErrorAbort();
    return;
  }
  
  /* Do not start DMA if an error has occured */
  if ( i2cError )
  {
    return;
  }
  
  /* Automatically ACK received bytes */
  I2C0->CTRL |= I2C_CTRL_AUTOACK;
  
  /* These are used by the RX interrupt handler
   * to fetch the last two bytes of the transaction */
  rxPointer = data + length - 2;
  bytesLeft = 2;
  
  /* Set transfer active flag. Cleared by interrupt handler
   * when STOP condition has been sent. */
  transferActive = true;
  
  /* Activate DMA */
  DMA_ActivateBasic(DMA_CHANNEL_I2C_RX,         /* RX DMA channel */
                    true,                       /* Primary descriptor */
                    false,                      /* No burst */
                    (void *)data,               /* Write to rx buffer */        
                    (void *)&(I2C0->RXDATA),    /* Read from RXDATA */
                    length - 3 );               /* Number of transfers */
}


int main(void)
{ 
  CHIP_Init();
  
  /* Enable access to the I2C bus on the DK */
  BSP_Init(BSP_INIT_DEFAULT);
  BSP_PeripheralAccess(BSP_I2C, true);
  
  /* Set up DMA and I2C */
  dmaInit();
  i2cInit();
  
  /* Clear error flag. Will be set on any error during transmission. */
  i2cError = false;
     
  i2cDmaWrite(EEPROM_I2C_ADDR, 0, (uint8_t *)txData, sizeof(txData));
  
  /* Wait until the I2C transfer is complete */
  sleepUntilTransferDone();
  
  /* The EEPROM will be busy for a while after writing to it. 
   * Do an 'ACK poll' until it is ready */
  ackPoll(EEPROM_I2C_ADDR);
    
  /* Read the sequence back from the EEPROM */
  i2cDmaRead(EEPROM_I2C_ADDR, 0, (uint8_t *)rxData, sizeof(txData));
  
  /* Wait until the I2C transfer is complete */
  sleepUntilTransferDone();
  
  if ( i2cError )
  {
    /* An error occured during the transfer */
  }
  
  /* Done */
  while (1);
}
