/**************************************************************************//**
 * @file flash.h
 * @brief Flash erasing and writing functions for Speex record
 * @author Silicon Labs
 * @version 1.06
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

#ifndef __FLASH_H
#define __FLASH_H

void flashInit512(void);
void flashDeinit(void);
void flashMassErase(uint32_t eraseCmd);

#endif
