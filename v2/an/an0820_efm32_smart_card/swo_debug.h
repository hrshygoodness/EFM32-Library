/******************************************************************************
 * @file swo_debug.h
 * @brief SWO output functions for printf over swo
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

/* Need to implement the  two Retarget IO functions with the read/write functions we want to use. */
/* Use ITM for printf over SWO. */
int RETARGET_WriteChar(char c);

int RETARGET_ReadChar(void);

/**************************************************************************//**
 * @brief Configure SWO - serial wire output
 *****************************************************************************/
void SWO_Setup(void);
