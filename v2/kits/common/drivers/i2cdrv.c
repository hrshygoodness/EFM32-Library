/***************************************************************************//**
 * @file
 * @brief I2C0 poll based driver for master mode operation on DK.
 * @version 3.20.5
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



#include <stddef.h>
#include "em_cmu.h"
#include "em_gpio.h"
#include "bsp.h"
#include "i2cdrv.h"
#include "i2cdrvconfig.h"

/*******************************************************************************
 **************************   GLOBAL FUNCTIONS   *******************************
 ******************************************************************************/

/***************************************************************************//**
 * @brief
 *   Initalize basic I2C master mode driver for use on the DK.
 *
 * @details
 *   This driver only supports master mode, single bus-master. In addition
 *   to configuring the EFM32 I2C peripheral module, it also configures DK
 *   specific setup in order to use the I2C bus.
 *
 * @param[in] init
 *   Pointer to I2C initialization structure.
 ******************************************************************************/
void I2CDRV_Init(const I2C_Init_TypeDef *init)
{
  int i;

#ifndef BSP_STK
  BSP_PeripheralAccess(BSP_I2C, true);
#endif
  CMU_ClockEnable(cmuClock_HFPER, true);
  CMU_ClockEnable(cmuClock_I2C0, true);

  /* Output value must be set to 1 to not drive lines low. Set */
  /* SCL first, to ensure it is high before changing SDA. */
  GPIO_PinModeSet(I2CDRV_SCL_PORT, I2CDRV_SCL_PIN, gpioModeWiredAndPullUp, 1);
  GPIO_PinModeSet(I2CDRV_SDA_PORT, I2CDRV_SDA_PIN, gpioModeWiredAndPullUp, 1);

  /* In some situations (after a reset during an I2C transfer), the slave */
  /* device may be left in an unknown state. Send 9 clock pulses just in case. */
  for (i = 0; i < 9; i++)
  {
    /*
     * TBD: Seems to be clocking at appr 80kHz-120kHz depending on compiler
     * optimization when running at 14MHz. A bit high for standard mode devices,
     * but DK only has fast mode devices. Need however to add some time
     * measurement in order to not be dependable on frequency and code executed.
     */
    GPIO_PinOutSet(I2CDRV_SCL_PORT, I2CDRV_SCL_PIN);
    GPIO_PinOutClear(I2CDRV_SCL_PORT, I2CDRV_SCL_PIN);
  }

  /* Enable pins at config location (3 is default which is the location used on the DK) */
  I2C0->ROUTE = I2C_ROUTE_SDAPEN |
                I2C_ROUTE_SCLPEN |
                (I2CDRV_PORT_LOCATION << _I2C_ROUTE_LOCATION_SHIFT);

  I2C_Init(I2C0, init);
}


/***************************************************************************//**
 * @brief
 *   Perform I2C transfer.
 *
 * @details
 *   This driver only supports master mode, single bus-master. It does not
 *   return until the transfer is complete, polling for completion.
 *
 * @param[in] seq
 *   Pointer to sequence structure defining the I2C transfer to take place. The
 *   referenced structure must exist until the transfer has fully completed.
 ******************************************************************************/
I2C_TransferReturn_TypeDef I2CDRV_Transfer(I2C_TransferSeq_TypeDef *seq)
{
  I2C_TransferReturn_TypeDef ret;
  uint32_t                   timeout = I2CDRV_TRANSFER_TIMEOUT;
  /* Do a polled transfer */
  ret = I2C_TransferInit(I2C0, seq);
  while (ret == i2cTransferInProgress && timeout--)
  {
    ret = I2C_Transfer(I2C0);
  }

  return(ret);
}
