/*!
 * File:
 *  board.h
 *  author: Shaoxian Luo
 *
 * Description:
 *  This file is the driver of the MCU.
 *
 * Silicon Laboratories Confidential
 * Copyright 2013 Silicon Laboratories, Inc.
 */
#ifndef __BOARD_H__
#define __BOARD_H__


void Delay(uint32_t dlyTicks);
uint8_t BSP_GetPB(uint8_t biPbNum);
uint32_t SysTick_GetTick(void);

#ifdef EFM32LG990F256
#define PUSH_BUTTON0_PORT  gpioPortB
#define PUSH_BUTTON0_PIN   9
#define PUSH_BUTTON1_PORT  gpioPortB
#define PUSH_BUTTON1_PIN   10
#endif

#ifdef EFM32TG840F32
#define PUSH_BUTTON0_PORT  gpioPortD
#define PUSH_BUTTON0_PIN   8
#define PUSH_BUTTON1_PORT  gpioPortB
#define PUSH_BUTTON1_PIN   11
#endif

#endif
