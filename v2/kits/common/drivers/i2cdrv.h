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



#ifndef __I2CDRV_H
#define __I2CDRV_H

#include "em_i2c.h"

/***************************************************************************//**
 * @addtogroup Drivers
 * @{
 ******************************************************************************/

/***************************************************************************//**
 * @addtogroup I2c
 * @{
 ******************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 *****************************   PROTOTYPES   **********************************
 ******************************************************************************/

void I2CDRV_Init(const I2C_Init_TypeDef *init);
I2C_TransferReturn_TypeDef I2CDRV_Transfer(I2C_TransferSeq_TypeDef *seq);

#ifdef __cplusplus
}
#endif

/** @} (end group I2c) */
/** @} (end group Drivers) */

#endif /* __I2CDRV_H */
