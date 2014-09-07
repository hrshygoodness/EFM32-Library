/***************************************************************************//**
 * @file i2cdrvconfig.h
 * @brief I2CDRV configuration file.
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

#ifndef __SILICON_LABS_I2CDRV_CONFIG_H__
#define __SILICON_LABS_I2CDRV_CONFIG_H__

/***************************************************************************//**
 * @addtogroup EM_Drivers
 * @{
 ******************************************************************************/

 /***************************************************************************//**
 * @addtogroup I2CDRV
 * @{
 ******************************************************************************/
/* Use location 6: SDA - Pin D6, SCL - Pin D7 */
#define I2CDRV_SCL_PORT gpioPortD
#define I2CDRV_SCL_PIN  7
#define I2CDRV_SDA_PORT gpioPortD
#define I2CDRV_SDA_PIN  6
#define I2CDRV_PORT_LOCATION 1
#define I2CDRV_TRANSFER_TIMEOUT 300000

/** @} (end addtogroup I2CDRV) */
/** @} (end addtogroup EM_Drivers) */

#endif /* __SILICON_LABS_I2CDRV_CONFIG_H__ */
