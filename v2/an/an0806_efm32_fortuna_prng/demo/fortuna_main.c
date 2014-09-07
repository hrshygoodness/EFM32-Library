/**************************************************************************//**
 * @file fortuna_main.c
 * @brief Fortuna Pseudo Random Number Generator (PRNG) demo for EFM32.
 * @author Silicon Labs
 * @version 1.03
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

/* EMLIB headers */
#include "em_chip.h"
#include "em_cmu.h"

#ifdef FORTUNA_DEBUG
/* BSP interface. */
#include "bsp.h"
#include "retargetserial.h"
#endif

/* Fortuna PRNG headers*/
#include "fortuna.h"
#include "fortuna_adc.h"

/* Defines */
#define ADC_ENTROPY_SOURCE_ID    (0)  /* Entropy source identifier of ADC is 0.
                                         The user should assign unique
                                         identifiers to every new entropy
                                         source that he may add. */

#define PRN_BUFFER_SIZE       (0x20)  /* Size of PRN buffer. */
#define PRN_REQUEST_SIZE       (100)  /* Number of pseudo number random bytes
                                         to request. */
#ifdef FORTUNA_DEBUG
#define DEBUG_PRINT(...)  iprintf
#else
#define DEBUG_PRINT(...) {}
#endif

/* System clock tick counter. */
static volatile uint32_t sysTickCounter = 0;


/**************************************************************************//**
 * @brief SysTick_Handler
 * Interrupt Service Routine for system tick counter
 *****************************************************************************/

void SysTick_Handler(void)
{
  sysTickCounter++;
}


/**************************************************************************//**
 * @brief SysTickGet
 *   Return system tick counter
 *****************************************************************************/

uint32_t SysTickGet(void)
{
  return sysTickCounter;
}


/**************************************************************************//**
 * @brief  Configure and initiaze chip, clocks, peripherals
 *****************************************************************************/
static void EFM32_Init(void)
{
  /* Initialize chip */
  CHIP_Init();

  /* Configure for 48MHz HFXO operation of core clock in order for the
     serial port I/O to work properly for high baudrates. HFRCO is inaccurate
     and may cause data loss at high baudrates. */
  CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);

  /*Update Core frequency */
  SystemCoreClockUpdate();

  /* Enable required clocks */
  CMU_ClockEnable(cmuClock_HFPER, true);
  CMU_ClockEnable(cmuClock_CORELE, true);
  CMU_ClockEnable(cmuClock_GPIO, true);

  /* Start LFXO, and use LFXO for low-energy modules */
  CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);

  /* Setup SysTick Timer for 1 msec interrupts  */
  if (SysTick_Config(CMU_ClockFreqGet(cmuClock_CORE) / 1000))
    while (1)
      ;
}


/**************************************************************************//**
 * @brief  Main starts here
 *****************************************************************************/
void AddInitialEntropy (void)
{
  int i;
  int retval;

  /* Add 8192 random events here because the entropy of the ADC is estimated
     to about one bit per 32 bit sample returned from AdcSampleGet.
     I.e. 32 pools x 8 words x 32 bits / 1 bit of entropy per event =
     8192 events. */
  for (i=0; i<8192; i++)
  {
    retval = FORTUNA_AddRandomEvent(ADC_ENTROPY_SOURCE_ID, AdcSampleGet());
    if (FORTUNA_OK != retval)
    {
      DEBUG_PRINT("FORTUNA_AddRandomEvent failed with error code %d\n", retval);
      while(1);
    }
  }
}


/**************************************************************************//**
 * @brief  Main starts here
 *****************************************************************************/
int main(void)
{
  uint32_t  randomData[PRN_BUFFER_SIZE/sizeof(uint32_t)];
  int       i;
  uint32_t  prn;
  int       retval;

  /* Initialize EFM32 core and required clocks. */
  EFM32_Init();

#ifdef FORTUNA_DEBUG
  /* Enable debuging on serial port.
     Redirect standard I/O (stdin/stdout) to the serial port.
     Please refer to kits/<kit>/config/retargetserialconfig.h where <kit> is
     the kit you are using in order to determine which serial port is being
     used by RETARGET_SerialInit on your kit. */
  BSP_Init(BSP_INIT_DEFAULT);
  RETARGET_SerialInit();
  RETARGET_SerialCrLf(true);
#endif

  /* Initialize the ADC to sample "random" data from the temperature sensor. */
  AdcRandomInitialize();
  
  /* Initialize the Fortuna PRNG without specifying a seed. */
  if (FORTUNA_OK != (retval = FORTUNA_Init()))
  {
    DEBUG_PRINT("FORTUNA_Init failed with error code %d.\n", retval);
    while(1);
  }

  /* Add some initial entropy to the Fortuna PRNG now in order to arm it for
     pseudo random number generation.
     The user may implement additional entropy sources that call the
     FORTUNA_AddRandomEvent function at any point in time. */
  AddInitialEntropy();

  /* First, demonstrate a call to the FORTUNA_RandomDataGet function, and
     display the data. */
  while (FORTUNA_OK !=
         (retval = FORTUNA_RandomDataGet (randomData, PRN_BUFFER_SIZE)))
  {
    /* The PRNG was unable to generate a pseudo random number, probably
       because there is too little entropy. The user should add more entropy
       for example with the following call:
       FORTUNA_AddRandomEvent(ADC_ENTROPY_SOURCE_ID, AdcSampleGet());
    */
    DEBUG_PRINT("FORTUNA_RandomDataGet failed with error code %d.\n", retval);
    while(1);
  }


  /* 
     The user should make sure to add more entropy during the life of the 
     Fortuna PRNG. Implementation of additional entropy sources is
     probably a good idea because it probably generates even more random
     entropy events. At a minimum the AdcSampleGet and FORTUNA_AddRandomEvent
     should be called once in a while. E.g. like this:
     ...
     FORTUNA_AddRandomEvent(ADC_ENTROPY_SOURCE_ID, AdcSampleGet());
     ...
  */

  /* Display the data. */
  for (i = 0; i < (int) PRN_BUFFER_SIZE/(int)sizeof(uint32_t); i++)
  {
    /* Print a newline for every 8th 32 bit word. */
    if (i && !(i%8))
    {
      DEBUG_PRINT("\n");
    }

    /* Print the generated random number to stdout in hexadecimal format. */
    DEBUG_PRINT("%08x ", (unsigned int) randomData[i]);
  }

  /* Second, demonstrate the FORTUNA_Rand by calling it several times and
     display the data. */
  for (i = 0; i < (int) PRN_REQUEST_SIZE/(int)sizeof(uint32_t); i++)
  {
    if (FORTUNA_OK != (retval = FORTUNA_Rand(&prn)))
    {
      /* The PRNG was unable to generate a pseudo random number, probably
         because there is too little entropy. The user should add more entropy
         for example with the following call:
         FORTUNA_AddRandomEvent(ADC_ENTROPY_SOURCE_ID, AdcSampleGet());
      */
      DEBUG_PRINT("FORTUNA_Rand failed with error code %d.\n", retval);
      while(1);
    }

    /* Print a newline for every 8th 32 bit word. */
    if (!(i%8))
    {
      DEBUG_PRINT("\n");
    }

    /* Print the generated random number to stdout in hexadecimal format. */
    DEBUG_PRINT("%08x ", (unsigned int) prn);
  }

  /* Shutdown the Fortuna PRNG. */
  FORTUNA_Shutdown();

#ifndef FORTUNA_DEBUG
  (void) retval;  /* Suppress compiler warning when debug is disabled. */
#endif

  while (1);
}
