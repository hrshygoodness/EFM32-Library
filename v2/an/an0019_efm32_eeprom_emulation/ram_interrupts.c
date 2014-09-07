/**************************************************************************//**
 * @file ram_interrupts.c
 * @brief Moves the interrupt vector to RAM
 * @author Silicon Labs
 * @version 1.10
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



#include <string.h>
#include "em_device.h"

/* Find the correct size of the interrupt vector */
#if defined(_EFM32_GECKO_FAMILY)
#define VECTOR_SIZE (16+30)
#elif defined(_EFM32_TINY_FAMILY)
#define VECTOR_SIZE (16+23)
#elif defined(_EFM32_GIANT_FAMILY)
#define VECTOR_SIZE (16+38)
#else
#error "Unknown Vector size"
#endif


/* Align the RAM vector table */
#if defined (__ICCARM__)
#pragma data_alignment=256
uint32_t vectorTable[VECTOR_SIZE];
#elif defined (__CC_ARM)
uint32_t vectorTable[VECTOR_SIZE] __attribute__ ((aligned(256)));
#elif defined (__GNUC__)
uint32_t vectorTable[VECTOR_SIZE] __attribute__ ((aligned(256)));
#else
#error "Undefined toolkit, need to define alignment"
#endif



/* SysTick interrupt handler. Counts ms ticks */
volatile uint32_t msTicks = 0; 


/* Place interrupt handler in RAM */
#if defined (__ICCARM__)               			/* IAR compiler */
__ramfunc
#elif defined(__CC_ARM)
 __attribute__ ((section ("ram_code"))) 
#elif defined(__GNUC__)                  		/* GCC based compilers */
#if defined (__CROSSWORKS_ARM)         			/* Rowley Crossworks */
__attribute__ ((section(".fast")))
#else                           						/* Other GCC */
__attribute__ ((section(".ram")))
#endif
#endif

void SysTick_Handler(void)
{
  msTicks++;
}


void moveInterruptVectorToRam(void)
{
  memcpy(vectorTable, (uint32_t*)SCB->VTOR, sizeof(uint32_t) * VECTOR_SIZE);
  SCB->VTOR = (uint32_t)vectorTable;
}
