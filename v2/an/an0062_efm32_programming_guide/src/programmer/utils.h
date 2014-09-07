/**************************************************************************//**
 * @file utils.h
 * @brief Various utility functions for the debug interface
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
#ifndef _UTILS_H_
#define _UTILS_H_

uint64_t readUniqueId(void);

void resetTarget(void);
void resetAndHaltTarget(void);
void stepTarget(void);
void runTarget(void);
void haltTarget(void);
void hardResetTarget(void);

int getPageSize(void);
void initAhbAp(void);

uint32_t readMem(uint32_t addr);
void writeMem(uint32_t addr, uint32_t data);
void waitForRegReady(void);

bool verifyFirmware(uint32_t *addr, uint32_t size);
void connectToTarget(void);

void writeCpuReg(int reg, uint32_t value);

bool verifyDpId(uint32_t dpId);
bool verifyAhbApId(uint32_t dpId);

int getFlashSize(void);
uint32_t getTarWrap(void);
void checkIfZeroGeckoIsLocked(void);

#endif