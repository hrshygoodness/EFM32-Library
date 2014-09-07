/*!
 * File:
 *  radio_hal.h
 *
 * Description:
 *  This file contains RADIO HAL.
 *
 * Silicon Laboratories Confidential
 * Copyright 2013 Silicon Laboratories, Inc.
 */

#ifndef _RADIO_HAL_H_
#define _RADIO_HAL_H_

                /* ======================================= *
                 *              I N C L U D E              *
                 * ======================================= */

                /* ======================================= *
                 *          D E F I N I T I O N S          *
                 * ======================================= */
#ifdef EFM32LG990F256
#define RF_SDN_PORT       gpioPortC
#define RF_SDN_PIN        3
#define RF_nIRQ_PORT      gpioPortC
#define RF_nIRQ_PIN       0
                   
#define RF_GPIO0_PORT     gpioPortC
#define RF_GPIO0_PIN      4
#define RF_GPIO1_PORT     gpioPortC
#define RF_GPIO1_PIN      5                   
#define RF_GPIO2_PORT     gpioPortB
#define RF_GPIO2_PIN      11
#define RF_GPIO3_PORT     gpioPortB
#define RF_GPIO3_PIN      12
#endif

#ifdef EFM32TG840F32
#define RF_SDN_PORT       gpioPortC
#define RF_SDN_PIN        5
#define RF_nIRQ_PORT      gpioPortC
#define RF_nIRQ_PIN       4
                   
#define RF_GPIO0_PORT     gpioPortC
#define RF_GPIO0_PIN      12
#define RF_GPIO1_PORT     gpioPortC
#define RF_GPIO1_PIN      13                   
#define RF_GPIO2_PORT     gpioPortB
#define RF_GPIO2_PIN      11
#define RF_GPIO3_PORT     gpioPortB
#define RF_GPIO3_PIN      12
#endif

                /* ======================================= *
                 *     G L O B A L   V A R I A B L E S     *
                 * ======================================= */

                /* ======================================= *
                 *  F U N C T I O N   P R O T O T Y P E S  *
                 * ======================================= */

void radio_hal_AssertShutdown(void);
void radio_hal_DeassertShutdown(void);
void radio_hal_ClearNsel(void);
void radio_hal_SetNsel(void);
uint8_t radio_hal_NirqLevel(void);

void radio_hal_SpiWriteByte(uint8_t byteToWrite);
uint8_t radio_hal_SpiReadByte(void);

void radio_hal_SpiWriteData(uint8_t byteCount, uint8_t* pData);
void radio_hal_SpiReadData(uint8_t byteCount, uint8_t* pData);

#ifdef DRIVERS_EXTENDED_SUPPORT
  uint8_t radio_hal_Gpio0Level(void);
  uint8_t radio_hal_Gpio1Level(void);
  uint8_t radio_hal_Gpio2Level(void);
  uint8_t radio_hal_Gpio3Level(void);
#endif

#endif //_RADIO_HAL_H_
