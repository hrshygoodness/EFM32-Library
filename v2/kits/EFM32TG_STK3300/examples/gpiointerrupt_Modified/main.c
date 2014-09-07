/**************************************************************************//**
 * @file
 * @brief Simple GPIO interrupt dispatcher Demo for EFM32TG_STK3300
 * @version 3.20.5
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

#include <stdint.h>
#include <stdbool.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "bsp.h"
#include "bsp_trace.h"
#include "gpiointerrupt.h"
#include "termio.h"

/**************************************************************************//**
 * @brief  Gpio callback
 * @param  pin - pin which triggered interrupt
 *****************************************************************************/
void gpioCallback(uint8_t pin)
{
  static bool ledOn = false;
  if (pin == 8)
  {
    if (ledOn) {
      dputs("Turning board LED off");
      BSP_LedClear(0);
      ledOn = false;
    }
  }
  else if (pin == 11)
  {
    if (!ledOn) {
      dputs("Turning board LED on");
      BSP_LedSet(0);
      ledOn = true;
    }
  }
}

/**************************************************************************//**
 * @brief  Gpio setup. Setup button pins to trigger falling edge interrupts.
 *  Register callbacks for that interrupts.
 *****************************************************************************/
void gpioSetup(void)
{
  /* Enable GPIO in CMU */
  CMU_ClockEnable(cmuClock_GPIO, true);

  /* Initialize GPIO interrupt dispatcher */
  GPIOINT_Init();
  
  /* Configure PD8 and PB11 as input */
  GPIO_PinModeSet(gpioPortD, 8, gpioModeInput, 0);
  GPIO_PinModeSet(gpioPortB, 11, gpioModeInput, 0);

  /* Register callbacks before setting up and enabling pin interrupt. */
  GPIOINT_CallbackRegister(8, gpioCallback);
  GPIOINT_CallbackRegister(11, gpioCallback);

  /* Set falling edge interrupt for both ports */
  GPIO_IntConfig(gpioPortD, 8, false, true, true);
  GPIO_IntConfig(gpioPortB, 11, false, true, true);
}

/*
#define BOOTLOADER_TEST_PIN gpioPortD, 8 

void enterLowPowerWait(void)
{
    // uncomment
    static int sleepCount = 0;
    
    // use test to check condition
    // adjust to odd/even pin interrupt as needed
    GPIO_IntConfig(BOOTLOADER_TEST_PIN, false, true, true);
    NVIC_EnableIRQ(GPIO_EVEN_IRQn);
    
    // Wait in EM3 (check if need to go to EM2 instead until pin pulled low
    while (GPIO_PinInGet(BOOTLOADER_TEST_PIN)) {
        dputs("\n\rEntering EM3\r");
        dprintf("sleepCount: %d\n\r", sleepCount);
        EMU_EnterEM3(true);
        sleepCount++;
    }
    
    // Disable GPIO interrupt
    GPIO_IntConfig(BOOTLOADER_TEST_PIN, false, false, false);
    NVIC_DisableIRQ(GPIO_EVEN_IRQn);  
}
*/

/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
  SCB->VTOR = 0x2900;
  /* Chip errata */
  CHIP_Init();
  

  /* If first word of user data page is non-zero, enable eA Profiler trace */
  //BSP_TraceProfilerSetup();

  /* Initialize gpio */
  gpioSetup();

  
  TERMIO_Init();
  TERMIO_SerialCrLf(1);
  clrScreen();
  dputs("\rStdio initialized");
  
  /* Initialize LED driver */
  BSP_LedsInit();
  
  dprintf("ntoh and hton test\n");
  
  uint8_t dummyBigEndian16t[] = { 0xde, 0xad };
  uint8_t dummyLilEndian16t[] = { 0xad, 0xde };
  uint8_t dummyBigEndian32t[] = { 0xde, 0xad, 0xbe, 0xef };
  uint8_t dummyLilEndian32t[] = { 0xef, 0xbe, 0xad, 0xde };
  uint16_t testLilEndian = 0xdead;
  uint32_t testLilEndian32t = 0xdeadbeef;
  printf("host 0xdeadbeef: %#x\n", 0xdeadbeef);
  printf("host htonl(0xdeadbeef): %#x\n", htonl(0xdeadbeef));
  printf("network 0xefbeadde | htonl: %#x\n", htonl(0xefbeadde));
  uint8_t *ptr = (uint8_t *)&testLilEndian32t;
  printf("ptr[0] = %#x\n", ptr[0]);
  uint16_t *ptrBigEnd = (uint16_t *)&dummyBigEndian16t; 
  uint16_t testVal = *ptrBigEnd;
  uint32_t *ptrBigEnd32 = (uint32_t *)&dummyBigEndian32t;
  uint32_t testVal32 = *ptrBigEnd32;
  
  printf("Original BigEndian: %#x | ntohs: %#x\n", \
         testVal, ntohs(testVal));
    printf("Original BigEndian32: %#x | ntohl: %#x\n", \
         testVal32, ntohl(testVal32));
  /*
  while (1) {
     if ( GPIO_PinInGet(BOOTLOADER_TEST_PIN) ) {
        enterLowPowerWait();
     }
  }
  */

  /* Infinite loop */
  while (1);
}
