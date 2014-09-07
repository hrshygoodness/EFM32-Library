/**************************************************************************//**
 * @file state_machine.h
 * @brief Handles the state machine where user can select and execute an action
 * @author Silicon Labs
 * @version 1.03
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
#ifndef _STATE_MACHINE_H_
#define _STATE_MACHINE_H_


#define STATE_BUSY             -1
#define STATE_CONNECT           0
#define STATE_FLASH_MSC         1
#define STATE_FLASH_FL          2
#define STATE_IDLE              3
#define STATE_LOCK              4
#define STATE_UNLOCK            5
#define STATE_UNLOCK_GECKO      6
#define STATE_UNLOCK_ZERO       7
#define STATE_TEST              8

void stateLoop(void);
uint32_t getNextState(uint32_t curState);

#endif