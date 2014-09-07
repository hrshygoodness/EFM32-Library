/**************************************************************************//**
 * @file vcmp_monitor.c
 * @brief VCMP Voltage Monitoring
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
#include <stdint.h>
#include <stdbool.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_vcmp.h"
#include "segmentlcd.h"


/* Defines */
#define VOLTAGE_LEVEL_HIGH    3.0
#define VOLTAGE_LEVEL_LOW     1.9
#define PRIM_NUMS    1024


/* Global variables */
bool awake = true; /* keeps track of VCMP state */
uint32_t vcmpTriggerLevelHigh;
uint32_t vcmpTriggerLevelLow;


#if defined (__ICCARM__)
  /* Declare, but do not initialize */
  __no_init uint32_t primes[PRIM_NUMS];
#else
  /*  
   *  For other compilers, the equivalent
   *  of noinit should be added here.
   */
  uint32_t primes[PRIM_NUMS];
#endif



/***************************************************************************//**
 * @brief
 *   Initializes VCMP to trigger on rising and falling supply voltage
 ******************************************************************************/
void initVCMP(void)
{
  /* Configure VCMP Init struct */
  VCMP_Init_TypeDef vcmpInit;
  vcmpInit.halfBias     = true;
  vcmpInit.biasProg     = 0; 
  vcmpInit.irqFalling   = true;
  vcmpInit.irqRising    = true;
  vcmpInit.warmup       = vcmpWarmTime256Cycles;
  vcmpInit.hyst         = vcmpHyst20mV;
  vcmpInit.inactive     = 1;
  vcmpInit.lowPowerRef  = false;
  vcmpInit.triggerLevel = vcmpTriggerLevelLow;
  vcmpInit.enable       = false;
 
  /* Initialize VCMP */
  CMU_ClockEnable(cmuClock_VCMP, true);
  VCMP_Init(&vcmpInit);

  /* Enable VCMP interrupt lines */
  NVIC_EnableIRQ(VCMP_IRQn);
  VCMP_IntEnable(VCMP_IEN_WARMUP);

  /* Enable VCMP and wait for warm-up complete */
  VCMP_Enable();
}



/***************************************************************************//**
 * @brief
 *   Compute prime numbers and print to LCD forever
 ******************************************************************************/
void computePrimesForever(void)
{
  uint32_t i, d, n;

  /* Find prime numbers forever */
  while (1)
  {
    primes[0] = 1;
    for (i = 1; i < PRIM_NUMS;)
    {
      for (n = primes[i - 1] + 1;; n++)
      {
        for (d = 2; d <= n; d++)
        {
          if (n == d)
          {
            primes[i] = n;
            SegmentLCD_Number(n);
            goto nexti;
          }
          if (n % d == 0) break;
        }
      }
 nexti:
      i++;
    }
  }
}


/***************************************************************************//**
 * @brief
 *   VCMP interrupt handler, triggers on EDGE and WARMUP events
 ******************************************************************************/
void VCMP_IRQHandler()
{
  /* Read and clear pending interrupts */
  uint32_t intFlags = VCMP->IF;
  VCMP_IntClear(intFlags);
  
  /* Execute on WARMUP interrupt */
  if (intFlags & VCMP_IF_WARMUP)
  {
    /* Enable Low Power Reference */
    VCMP_LowPowerRefSet(true);
    
    /* Enable VCMP Edge interrupt */
    VCMP_IntEnable(VCMP_IEN_EDGE);
  }

  /* Execute on EDGE interrupt */
  if (intFlags & VCMP_IF_EDGE)
  {
    if (awake)
    {
      /* Prepare application for a low-voltage situation */

      /* Go to Deep Sleep (EM2), or Stop (EM3) if no LF clocks are enabled.
       * Setting SLEEPONEXIT causes the controller to enter EM2/3 immediately after
       *         exiting this interrupt handler */
      SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
      SCB->SCR |= SCB_SCR_SLEEPONEXIT_Msk;

      /* Manually set new trigger level to wake-up threshold*/
      VCMP_TriggerSet( vcmpTriggerLevelHigh );
    }
    else
    {
      /* Wake up */
      SCB->SCR = (SCB->SCR & ~SCB_SCR_SLEEPONEXIT_Msk);

      /* Manually set new trigger level to sleep threshold */
      VCMP_TriggerSet( vcmpTriggerLevelLow );
    }

    awake = !awake;
  }
}

/**************************************************************************//**
 * @brief  
 *   Initializes the uninitialized global variables
 *****************************************************************************/
void globalVarInit( void )
{
  /* Initialize primes to zero */
  int i;
  
  for( i = 0; i < PRIM_NUMS; i++ )
  {
    primes[i] = 0x0;
  }
}


/**************************************************************************//**
 * @brief  
 *   Main function
 *****************************************************************************/
int main(void)
{
  /* Chip errata */
  CHIP_Init();

  
  /* Calculate VCMP thresholds */
  vcmpTriggerLevelHigh = VCMP_VoltageToLevel(VOLTAGE_LEVEL_HIGH);
  vcmpTriggerLevelLow  = VCMP_VoltageToLevel(VOLTAGE_LEVEL_LOW);
  

  /* Setup VCMP for supply voltage monitoring */
  initVCMP();

  /* Enter EM1 while VCMP is warming up */
  EMU_EnterEM1();
  
  /* Initialize uninitialized global variables */
  globalVarInit();

  /* Initialize LCD controller */
  SegmentLCD_Init(false);
  SegmentLCD_Write("PRIMES");

  /* Reduce HF clock frequency to 1 MHz to give a more readable output on LCD */
  CMU_HFRCOBandSet(cmuHFRCOBand_1MHz);

  /* Compute primes */
  while (1)
  {
    computePrimesForever();
  }
}
