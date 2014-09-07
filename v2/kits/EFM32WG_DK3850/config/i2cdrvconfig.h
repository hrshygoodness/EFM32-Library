/***************************************************************************//**
 * @file
 * @brief Provide I2CDRV configuration parameters.
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

#ifndef __I2CDRVCONFIG_H
#define __I2CDRVCONFIG_H

/// I2C pin/port configuration. Note that this driver supports only one
/// driver instance.
#define I2CDRV_PORT_LOCATION    3                 /// Location used on DK

#define I2CDRV_SCL_PORT         gpioPortD       
#define I2CDRV_SCL_PIN          15

#define I2CDRV_SDA_PORT         gpioPortD
#define I2CDRV_SDA_PIN          14

#define I2CDRV_TRANSFER_TIMEOUT 300000

#endif
