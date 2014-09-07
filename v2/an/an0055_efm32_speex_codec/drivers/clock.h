/**************************************************************************//**
 * @file clock.h
 * @brief Setup core and peripheral clocks for Speex Codec
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

#ifndef __CLOCK_H
#define __CLOCK_H

#define FREQ8M        8000000
#define FREQ16M       16000000
#define FREQ48M       48000000
#define FREQ32M       32000000

void initClock(void);

#endif
